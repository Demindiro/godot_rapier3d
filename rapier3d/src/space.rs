use crate::area::Area;
use crate::server;
use crate::server::Body;
use crate::server::{AreaIndex, BodyIndex, MapIndex, SpaceIndex};
use crate::util::*;
use gdnative::prelude::*;
use rapier3d::crossbeam::channel::{self, Receiver};
use rapier3d::dynamics::{
	CCDSolver, IntegrationParameters, Joint, JointHandle, JointParams, JointSet, RigidBody,
	RigidBodyHandle, RigidBodySet,
};
use rapier3d::geometry::{
	BroadPhase, Collider, ColliderHandle, ColliderSet, ContactEvent, InteractionGroups,
	IntersectionEvent, NarrowPhase, Ray, SolverFlags,
};
use rapier3d::na::Point3;
use rapier3d::pipeline::{
	ChannelEventCollector, ContactModificationContext, PairFilterContext, PhysicsHooks,
	PhysicsHooksFlags, PhysicsPipeline, QueryPipeline,
};
use std::collections::BTreeMap;

pub struct Space {
	pub enabled: bool,

	physics_pipeline: PhysicsPipeline,
	query_pipeline: QueryPipeline,
	query_pipeline_out_of_date: bool,

	gravity: Vector3,
	integration_parameters: IntegrationParameters,
	default_linear_damp: f32,
	default_angular_damp: f32,
	broad_phase: BroadPhase,
	narrow_phase: NarrowPhase,

	bodies: RigidBodySet,
	colliders: ColliderSet,
	joints: JointSet,

	ccd_solver: CCDSolver,

	body_exclusions: BodyExclusionHooks,
	event_handler: ChannelEventCollector,
	contact_recv: Receiver<ContactEvent>,
	intersection_recv: Receiver<IntersectionEvent>,

	area_map: BTreeMap<i32, Vec<RigidBodyHandle>>,

	index: Option<SpaceIndex>,
}

pub struct RayCastResult {
	pub position: Vector3,
	pub normal: Vector3,
	pub body: BodyIndex,
	pub shape: u32,
}

struct BodyExclusionHooks {
	// Reasoning for using Vec and Box instead of HashMap & HashSet:
	// * SparseVec is likely densely packed -> not many "holes" in the Vec.
	// * Amount of body exclusions is likely small -> Vec is compact and maybe faster.
	exclusions: Vec<Vec<BodyIndex>>,
}

impl Space {
	pub fn new() -> Self {
		let (contact_send, contact_recv) = channel::unbounded();
		let (intersection_send, intersection_recv) = channel::unbounded();
		Space {
			physics_pipeline: PhysicsPipeline::new(),
			query_pipeline: QueryPipeline::new(),
			gravity: Vector3::new(0.0, -9.81, 0.0),
			integration_parameters: IntegrationParameters::default(),
			broad_phase: BroadPhase::new(),
			narrow_phase: NarrowPhase::new(),
			bodies: RigidBodySet::new(),
			colliders: ColliderSet::new(),
			joints: JointSet::new(),
			ccd_solver: CCDSolver::new(),
			body_exclusions: BodyExclusionHooks::new(),
			event_handler: ChannelEventCollector::new(intersection_send, contact_send),
			intersection_recv,
			contact_recv,
			query_pipeline_out_of_date: false,
			index: None,
			enabled: true,

			area_map: BTreeMap::new(),

			default_linear_damp: 0.0,
			default_angular_damp: 0.0,
		}
	}

	pub fn step(&mut self, delta: f32) {
		self.integration_parameters.dt = delta;
		self.physics_pipeline.step(
			&vec_gd_to_na(self.gravity),
			&self.integration_parameters,
			&mut self.broad_phase,
			&mut self.narrow_phase,
			&mut self.bodies,
			&mut self.colliders,
			&mut self.joints,
			&mut self.ccd_solver,
			&self.body_exclusions,
			&self.event_handler,
		);
		self.query_pipeline_out_of_date = true;

		let mut areas = AreaIndex::write_all();
		let mut bodies = BodyIndex::write_all();

		// Clear area events
		let mut remove = Vec::new();
		for (&prio, vec) in self.area_map.iter() {
			let mut rm = Vec::new();
			for (i, &area) in vec.iter().enumerate() {
				if let Some(area) = self.bodies.get(area) {
					let area = Area::get_rigidbody_userdata(area).expect("Invalid area userdata");
					let area = areas
						.get_mut(area.index(), area.generation())
						.expect("Invalid area index");
					area.clear_events();
				} else {
					// u32 is slightly more efficient and cannot be exceeded anyways, as AreaIndex
					// also uses u32.
					rm.push(i as u32);
				}
			}
			if !rm.is_empty() {
				remove.push((prio, rm));
			}
		}

		// Remove stale areas
		for (p, rm) in remove.into_iter() {
			let v = self.area_map.get_mut(&p).unwrap();
			for i in rm.into_iter().rev() {
				v.swap_remove(i as usize);
			}
			if v.is_empty() {
				self.area_map.remove(&p);
			}
		}

		// Process area intersections
		while let Ok(event) = self.intersection_recv.try_recv() {
			let a = &self.colliders[event.collider1];
			let b = &self.colliders[event.collider2];
			let (area, body) = if let Some(a) = Area::get_collider_userdata(a) {
				if let Some(b) = Area::get_collider_userdata(b) {
					let (area_a, area_b) =
						areas.get_mut2(a.index(), a.generation(), b.index(), b.generation());
					let area_a = area_a.expect("Invalid area A index");
					let area_b = area_b.expect("Invalid area B index");
					if area_b.monitorable() {
						area_a.push_area_event(b, event.intersecting);
					}
					if area_a.monitorable() {
						area_b.push_area_event(a, event.intersecting);
					}
					continue;
				} else {
					(a, Body::get_shape_userdata(&b).0)
				}
			} else if let Some(b) = Area::get_collider_userdata(b) {
				(b, Body::get_shape_userdata(&a).0)
			} else {
				// I suppose it makes sense to avoid poisoning the locks?
				drop(areas);
				drop(bodies);
				panic!("Neither collider is an area");
			};
			let area = areas
				.get_mut(area.index(), area.generation())
				.expect("Invalid area index");
			area.push_body_event(body, event.intersecting);
		}

		// Process body contacts (TODO)
		while self.contact_recv.try_recv().is_ok() {}

		// Register area space overrides to bodies
		for &area in self.area_map.values().rev().flat_map(|v| v.iter()) {
			let rb = &self.bodies[area];
			let area = Area::get_rigidbody_userdata(rb).expect("Area body has invalid userdata");
			let area = areas
				.get_mut(area.index(), area.generation())
				.expect("Invalid area index");
			area.apply_events(&rb, &mut bodies, &self.bodies);
		}

		// Apply space overrides to bodies
		//for rb in self.bodies.iter_mut().map(|v| v.1) {
		for (_, rb) in self.bodies.iter_mut() {
			// FIXME *puke*
			if Area::get_rigidbody_userdata(rb).is_none() {
				let body = Body::get_index(rb);
				let body = bodies
					.get_mut(body.index(), body.generation())
					.expect("Invalid body index");
				body.apply_area_overrides(rb, self.default_linear_damp, self.default_angular_damp);
			}
		}
	}

	/// Gets the index of this space
	///
	/// # Panics
	///
	/// Panics if index is not set
	#[allow(dead_code)]
	pub fn index(&self) -> SpaceIndex {
		self.index.unwrap()
	}

	/// Sets the index of this space once.
	///
	/// # Panics
	///
	/// Panics if index is already set
	pub fn set_index(&mut self, index: SpaceIndex) {
		assert_eq!(self.index, None);
		self.index = Some(index);
	}

	/// Gets the RigidBodySet of this space
	#[allow(dead_code)]
	pub fn bodies(&self) -> &RigidBodySet {
		&self.bodies
	}

	/// Gets the RigidBodySet of this space mutably
	#[allow(dead_code)]
	pub fn bodies_mut(&mut self) -> &mut RigidBodySet {
		&mut self.bodies
	}

	/// Gets the ColliderSet of this space
	#[allow(dead_code)]
	pub fn colliders(&self) -> &ColliderSet {
		&self.colliders
	}

	/// Gets the ColliderSet of this space mutably
	#[allow(dead_code)]
	pub fn colliders_mut(&mut self) -> &mut ColliderSet {
		&mut self.colliders
	}

	/// Gets the ColliderSet of this space.
	#[allow(dead_code)]
	pub fn joints(&self) -> &JointSet {
		&self.joints
	}

	/// Gets the ColliderSet of this space mutably.
	pub fn joints_mut(&mut self) -> &mut JointSet {
		&mut self.joints
	}

	/// Make two bodies not interact with each other.
	/// Returns `Ok` if the bodies did not exclude each other already, `Err` otherwise.
	pub fn add_body_exclusion(&mut self, index_a: BodyIndex, index_b: BodyIndex) -> Result<(), ()> {
		self.body_exclusions.add_exclusion(index_a, index_b)
	}

	/// Make two bodies interact with each other again.
	/// Returns `Ok` if the bodies did exclude each other, `Err` otherwise.
	#[allow(dead_code)] // FIXME body_remove_collision_exception needs to be implemented
	pub fn remove_body_exclusion(
		&mut self,
		index_a: BodyIndex,
		index_b: BodyIndex,
	) -> Result<(), ()> {
		self.body_exclusions.remove_exclusion(index_a, index_b)
	}

	/// Ensure the pipeline is up to date.
	pub fn update_query_pipeline(&mut self) {
		if self.query_pipeline_out_of_date {
			self.query_pipeline.update(&self.bodies, &self.colliders);
			self.query_pipeline_out_of_date = false;
		}
	}

	/// Adds a body and returns a handle to it.
	pub fn add_body(&mut self, body: RigidBody) -> RigidBodyHandle {
		self.bodies.insert(body)
	}

	/// Removes the body with the given handle. Returns the body if it existed
	pub fn remove_body(&mut self, body: RigidBodyHandle) -> Option<RigidBody> {
		self.bodies
			.remove(body, &mut self.colliders, &mut self.joints)
	}

	/// Adds a collider and returns a handle to it.
	pub fn add_collider(&mut self, collider: Collider, body: RigidBodyHandle) -> ColliderHandle {
		self.colliders.insert(collider, body, &mut self.bodies)
	}

	/// Removes the collider with the given handle. Returns the collider if it existed.
	pub fn remove_collider(&mut self, collider: ColliderHandle) -> Option<Collider> {
		self.colliders.remove(collider, &mut self.bodies, true)
	}

	/// Adds a joint and returns a handle to it.
	pub fn add_joint(
		&mut self,
		joint: JointParams,
		body_a: RigidBodyHandle,
		body_b: RigidBodyHandle,
	) -> JointHandle {
		self.joints.insert(&mut self.bodies, body_a, body_b, joint)
	}

	/// Removes the joint with the given handle. Returns the joint if it existed
	pub fn remove_joint(&mut self, joint: JointHandle) -> Option<Joint> {
		self.joints.remove(joint, &mut self.bodies, true)
	}

	pub fn cast_ray(
		&mut self,
		from: Vector3,
		to: Vector3,
		mask: u16,
		exclude: &[BodyIndex],
	) -> Option<RayCastResult> {
		self.update_query_pipeline();
		let dir = to - from;
		let (dir, max_toi) = (dir.normalize(), dir.length());
		let filter = if !exclude.is_empty() {
			Some(|_, c: &'_ _| !exclude.contains(&server::Body::get_shape_userdata(c).0))
		} else {
			None
		};
		let filter = filter.as_ref();
		// TODO account for excluded colliders
		// Rapier 0.7 added a `filter` parameter which can be used for the exclusion list.
		let intersection = self.query_pipeline.cast_ray_and_get_normal(
			&self.colliders,
			&Ray::new(Point3::new(from.x, from.y, from.z), vec_gd_to_na(dir)),
			max_toi,
			// TODO what is the solid parameter for?
			false,
			InteractionGroups::new(u16::MAX, mask),
			filter.map(|v| v as &dyn Fn(_, &'_ _) -> bool),
		);
		intersection.map(|(collider, intersection)| {
			let position = dir * intersection.toi + from; // MulAdd not implemented :(
			let normal = vec_na_to_gd(intersection.normal);
			let collider = self.colliders.get(collider).unwrap();
			let (body, shape) = server::Body::get_shape_userdata(collider);
			RayCastResult {
				position,
				normal,
				body,
				shape,
			}
		})
	}

	/// Returns the gravity vector of this space
	pub fn gravity(&self) -> Vector3 {
		self.gravity
	}

	/// Sets the gravity vector of this space
	pub fn set_gravity(&mut self, gravity: Vector3) {
		self.gravity = gravity;
	}

	/// Sets the default linear damp of rigid bodies in this space
	pub fn set_default_linear_damp(&mut self, damp: f32) {
		self.default_linear_damp = damp;
	}

	/// Sets the default angular damp of rigid bodies in this space
	pub fn set_default_angular_damp(&mut self, damp: f32) {
		self.default_angular_damp = damp;
	}

	/// Registers an area if needed and stores the priority of said area
	/// [`RigidBodyHandle`]s are used as those can be used directly to check if
	/// the area is still present in this space.
	pub fn set_area_priority(&mut self, area: RigidBodyHandle, priority: i32) {
		// FIXME move userdata code to area.rs, this may be overwritten by accident in the future
		let rb = self.bodies.get_mut(area).expect("Invalid handle");
		let prio = (rb.user_data >> 96) as i32;
		if let Some(v) = self.area_map.get_mut(&prio) {
			if let Some(i) = v.iter().position(|e| *e == area) {
				v.remove(i);
			}
		}

		rb.user_data |= (priority as u32 as u128) << 96;
		self.area_map
			.entry(priority)
			.or_insert_with(Vec::new)
			.push(area);
	}
}

impl BodyExclusionHooks {
	fn new() -> Self {
		Self {
			exclusions: Vec::<Vec<BodyIndex>>::new(),
		}
	}

	fn add_exclusion(&mut self, index_a: BodyIndex, index_b: BodyIndex) -> Result<(), ()> {
		// TODO how should we handle self-exclusions? (index_a == index_b)
		let (a, b) = (index_a, index_b);
		let (a, b) = if a.index() < b.index() {
			(a, b)
		} else {
			(b, a)
		};
		if let Some(vec) = self.exclusions.get_mut(a.index() as usize) {
			if vec.contains(&b) {
				Err(())
			} else {
				vec.push(b);
				Ok(())
			}
		} else {
			self.exclusions
				.resize_with(a.index() as usize + 1, Vec::new);
			self.exclusions[a.index() as usize] = Vec::from([b]);
			Ok(())
		}
	}

	// TODO implement body_remove_exception and remove the allow
	#[allow(dead_code)]
	fn remove_exclusion(&mut self, index_a: BodyIndex, index_b: BodyIndex) -> Result<(), ()> {
		let (a, b) = (index_a, index_b);
		let (a, b) = if a.index() < b.index() {
			(a, b)
		} else {
			(b, a)
		};
		if let Some(vec) = self.exclusions.get_mut(a.index() as usize) {
			vec.swap_remove(vec.iter().position(|&e| e == b).ok_or(())?);
			Ok(())
		} else {
			Err(())
		}
	}
}

impl PhysicsHooks for BodyExclusionHooks {
	fn active_hooks(&self) -> PhysicsHooksFlags {
		PhysicsHooksFlags::FILTER_CONTACT_PAIR
	}

	fn filter_contact_pair(&self, context: &PairFilterContext) -> Option<SolverFlags> {
		let a = server::Body::get_index(context.rigid_body1);
		let b = server::Body::get_index(context.rigid_body2);
		let (a, b) = if a.index() < b.index() {
			(a, b)
		} else {
			(b, a)
		};
		if let Some(indices) = self.exclusions.get(a.index() as usize) {
			for &i in indices.iter() {
				if i == b {
					return None;
				}
			}
		}
		Some(SolverFlags::default())
	}

	fn filter_intersection_pair(&self, _: &PairFilterContext) -> bool {
		false
	}

	fn modify_solver_contacts(&self, _: &mut ContactModificationContext) {}
}
