use crate::area::Area;
use crate::server::{AreaIndex, BodyIndex, MapIndex, ShapeIndex, SpaceIndex};
use crate::util::*;
use crate::{area, body};
use core::convert::TryFrom;
use gdnative::prelude::*;
use rapier3d::crossbeam::channel::{self, Receiver, Sender};
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
	ContactModificationContext, EventHandler, PairFilterContext, PhysicsHooks, PhysicsPipeline,
	QueryPipeline,
};
use rapier3d::prelude::*;
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

	islands: IslandManager,
	bodies: RigidBodySet,
	colliders: ColliderSet,
	joints: JointSet,

	ccd_solver: CCDSolver,

	body_exclusions: BodyExclusionHooks,
	event_handler: IntersectionEventCollector,
	intersection_recv: Receiver<IntersectionEvent>,
	contact_recv: Receiver<(BodyIndex, body::ContactEvent)>,

	area_map: BTreeMap<i32, Vec<RigidBodyHandle>>,

	debug_contacts: Vec<Vector3>,
	debug_contact_count: usize,

	index: Option<SpaceIndex>,
}

pub enum BodyOrAreaIndex {
	Body(BodyIndex),
	Area(AreaIndex),
}

pub struct RayCastResult {
	pub position: Vector3,
	pub normal: Vector3,
	pub index: BodyOrAreaIndex,
	pub shape: u32,
}

pub struct IntersectShapeResult {
	pub index: BodyOrAreaIndex,
	pub shape: u32,
}

struct BodyExclusionHooks {
	// Reasoning for using Vec and Box instead of HashMap & HashSet:
	// * SparseVec is likely densely packed -> not many "holes" in the Vec.
	// * Amount of body exclusions is likely small -> Vec is compact and maybe faster.
	exclusions: Vec<Vec<BodyIndex>>,
	contacts_sender: Sender<(BodyIndex, body::ContactEvent)>,
}

struct IntersectionEventCollector {
	sender: Sender<IntersectionEvent>,
}

impl Space {
	pub fn new() -> Self {
		let (intersection_send, intersection_recv) = channel::unbounded();
		let (contact_send, contact_recv) = channel::unbounded();
		let query_pipeline = QueryPipeline::new();
		let narrow_phase = NarrowPhase::new();
		Space {
			physics_pipeline: PhysicsPipeline::new(),
			query_pipeline,
			gravity: Vector3::new(0.0, -9.81, 0.0),
			integration_parameters: IntegrationParameters::default(),
			broad_phase: BroadPhase::new(),
			narrow_phase,

			islands: IslandManager::new(),
			bodies: RigidBodySet::new(),
			colliders: ColliderSet::new(),
			joints: JointSet::new(),

			ccd_solver: CCDSolver::new(),
			body_exclusions: BodyExclusionHooks::new(contact_send),
			event_handler: IntersectionEventCollector {
				sender: intersection_send,
			},
			intersection_recv,
			contact_recv,
			query_pipeline_out_of_date: false,
			index: None,
			enabled: true,

			area_map: BTreeMap::new(),

			debug_contacts: Vec::new(),
			debug_contact_count: 0,

			default_linear_damp: 0.0,
			default_angular_damp: 0.0,
		}
	}

	pub fn step(&mut self, delta: f32) {
		let mut areas = AreaIndex::write_all();
		let mut bodies = BodyIndex::write_all();
		let mut shapes = ShapeIndex::write_all();

		// Update rigidbodies with stale state
		for rb in self.bodies.iter_mut() {
			if let Ok(body) = body::RigidBodyUserdata::try_from(&*rb.1) {
				let body = bodies.get_mut(body.into()).expect("Invalid body index");
				body.refresh_state(rb.1, &mut shapes);
			}
		}

		// Step
		self.integration_parameters.dt = delta;
		self.physics_pipeline.step(
			&vec_gd_to_na(self.gravity),
			&self.integration_parameters,
			&mut self.islands,
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

		// Clear area events
		let mut remove = Vec::new();
		for (&prio, vec) in self.area_map.iter() {
			let mut rm = Vec::new();
			for (i, &area) in vec.iter().enumerate() {
				if let Some(area) = self.bodies.get(area) {
					let area = Area::get_rigidbody_userdata(area).expect("Invalid area userdata");
					let area = areas.get_mut(area.into()).expect("Invalid area index");
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
					let (a, b) = (a.index(), b.index());
					let (area_a, area_b) = areas.get2_mut(a.into(), b.into());
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
					(a, body::ColliderUserdata::try_from(b).unwrap().index())
				}
			} else if let Some(b) = Area::get_collider_userdata(b) {
				(b, body::ColliderUserdata::try_from(a).unwrap().index())
			} else {
				panic!("Neither collider is an area");
			};
			let area = areas.get_mut(area.into()).expect("Invalid area index");
			area.push_body_event(body, event.intersecting);
		}

		// Process body contacts
		while let Ok((body, event)) = self.contact_recv.try_recv() {
			bodies
				.get_mut(body.into())
				.expect("Invalid body index")
				.add_contact(event);
		}

		// Register area space overrides to bodies
		for &area in self.area_map.values().rev().flat_map(|v| v.iter()) {
			let rb = &self.bodies[area];
			let area = Area::get_rigidbody_userdata(rb).expect("Area body has invalid userdata");
			let area = areas.get_mut(area.into()).expect("Invalid area index");
			area.apply_events(&rb, &mut bodies, &self.bodies);
		}

		// Apply space overrides to bodies
		for rb in self.bodies.iter_mut().map(|v| v.1) {
			if let Ok(body) = body::RigidBodyUserdata::try_from(&*rb) {
				let body = bodies.get_mut(body.into()).expect("Invalid body index");
				body.apply_area_overrides(rb, self.default_linear_damp, self.default_angular_damp);
			}
		}

		// Clear debug contacts
		self.debug_contacts.clear();
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
	pub fn add_body_exclusion(
		&mut self,
		index_a: BodyIndex,
		index_b: BodyIndex,
	) -> Result<(), ExclusionAlreadyExists> {
		self.body_exclusions.add_exclusion(index_a, index_b)
	}

	/// Make two bodies interact with each other again.
	/// Returns `Ok` if the bodies did exclude each other, `Err` otherwise.
	pub fn remove_body_exclusion(
		&mut self,
		index_a: BodyIndex,
		index_b: BodyIndex,
	) -> Result<(), ExclusionDoesntExist> {
		self.body_exclusions.remove_exclusion(index_a, index_b)
	}

	/// Ensure the pipeline is up to date.
	pub fn update_query_pipeline(&mut self) {
		if self.query_pipeline_out_of_date {
			self.query_pipeline
				.update(&self.islands, &self.bodies, &self.colliders);
			self.query_pipeline_out_of_date = false;
		}
	}

	/// Adds a body and returns a handle to it.
	pub fn add_body(&mut self, body: RigidBody) -> RigidBodyHandle {
		self.bodies.insert(body)
	}

	/// Removes the body with the given handle. Returns the body if it existed
	pub fn remove_body(&mut self, body: RigidBodyHandle) -> Option<RigidBody> {
		self.bodies.remove(
			body,
			&mut self.islands,
			&mut self.colliders,
			&mut self.joints,
		)
	}

	/// Adds a collider and returns a handle to it.
	pub fn add_collider(&mut self, collider: Collider, body: RigidBodyHandle) -> ColliderHandle {
		self.query_pipeline_out_of_date = true;
		self.colliders
			.insert_with_parent(collider, body, &mut self.bodies)
	}

	/// Removes the collider with the given handle. Returns the collider if it existed.
	pub fn remove_collider(&mut self, collider: ColliderHandle) -> Option<Collider> {
		self.colliders
			.remove(collider, &mut self.islands, &mut self.bodies, true)
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
		self.joints
			.remove(joint, &mut self.islands, &mut self.bodies, true)
	}

	pub fn cast_ray(
		&mut self,
		from: Vector3,
		to: Vector3,
		mask: u32,
		exclude_bodies: Option<&[BodyIndex]>,
		exclude_areas: Option<&[AreaIndex]>,
		ray_pickable_only: bool,
	) -> Option<RayCastResult> {
		self.update_query_pipeline();
		let dir = to - from;
		let (dir, max_toi) = (dir.normalize(), dir.length());
		// TODO avoid box
		let s = &*self;
		let filter = match (exclude_bodies, exclude_areas) {
			(Some(bodies), Some(areas)) => Box::new(move |c| {
				let c = s.colliders.get(c).unwrap();
				if let Ok(ud) = body::ColliderUserdata::try_from(c) {
					(ud.ray_pickable() || !ray_pickable_only) && !bodies.contains(&ud.index())
				} else {
					let ud =
						area::ColliderUserdata::try_from(c).expect("Invalid collider userdata");
					(ud.ray_pickable() || !ray_pickable_only) && !areas.contains(&ud.index())
				}
			}) as Box<dyn Fn(ColliderHandle) -> bool>,
			(Some(bodies), None) => Box::new(move |c| {
				let c = s.colliders.get(c).unwrap();
				if let Ok(ud) = body::ColliderUserdata::try_from(c) {
					(ud.ray_pickable() || !ray_pickable_only) && !bodies.contains(&ud.index())
				} else {
					false
				}
			}) as Box<dyn Fn(ColliderHandle) -> bool>,
			(None, Some(areas)) => Box::new(move |c| {
				let c = s.colliders.get(c).unwrap();
				if let Ok(ud) = area::ColliderUserdata::try_from(c) {
					(ud.ray_pickable() || !ray_pickable_only) && !areas.contains(&ud.index())
				} else {
					false
				}
			}) as Box<dyn Fn(ColliderHandle) -> bool>,
			(None, None) => return None,
		};
		let intersection = self.query_pipeline.cast_ray_and_get_normal(
			&self.colliders,
			&Ray::new(Point3::new(from.x, from.y, from.z), vec_gd_to_na(dir)),
			max_toi,
			// TODO what is the solid parameter for?
			false,
			InteractionGroups::new(0, mask),
			Some(&filter),
		);
		intersection.map(|(collider, intersection)| {
			let position = dir * intersection.toi + from; // MulAdd not implemented :(
			let normal = vec_na_to_gd(intersection.normal);
			let collider = self.colliders.get(collider).unwrap();
			if let Ok(c) = body::ColliderUserdata::try_from(collider) {
				RayCastResult {
					position,
					normal,
					index: BodyOrAreaIndex::Body(c.index()),
					shape: c.shape(),
				}
			} else {
				let c =
					area::ColliderUserdata::try_from(collider).expect("Invalid collider userdata");
				RayCastResult {
					position,
					normal,
					index: BodyOrAreaIndex::Area(c.index()),
					shape: c.shape(),
				}
			}
		})
	}

	/// Return all colliders intersecting with the given shape at the given positions.
	pub fn intersect_shape(
		&mut self,
		shape: &dyn Shape,
		position: &Transform,
		mask: u32,
		exclude_bodies: Option<&[BodyIndex]>,
		exclude_areas: Option<&[AreaIndex]>,
		max_results: usize,
	) -> impl Iterator<Item = IntersectShapeResult> {
		self.update_query_pipeline();
		let position = transform_to_isometry(*position);
		let s = &*self;
		let filter = match (exclude_bodies, exclude_areas) {
			(Some(bodies), Some(areas)) => Box::new(move |c| {
				let c = s.colliders.get(c).unwrap();
				if let Ok(ud) = body::ColliderUserdata::try_from(c) {
					!bodies.contains(&ud.index())
				} else {
					let ud =
						area::ColliderUserdata::try_from(c).expect("Invalid collider userdata");
					!areas.contains(&ud.index())
				}
			}) as Box<dyn Fn(ColliderHandle) -> bool>,
			(Some(bodies), None) => Box::new(move |c| {
				let c = s.colliders.get(c).unwrap();
				if let Ok(ud) = body::ColliderUserdata::try_from(c) {
					!bodies.contains(&ud.index())
				} else {
					false
				}
			}) as Box<dyn Fn(ColliderHandle) -> bool>,
			(None, Some(areas)) => Box::new(move |c| {
				let c = s.colliders.get(c).unwrap();
				if let Ok(ud) = area::ColliderUserdata::try_from(c) {
					!areas.contains(&ud.index())
				} else {
					false
				}
			}) as Box<dyn Fn(ColliderHandle) -> bool>,
			(None, None) => return Vec::new().into_iter(),
		};

		let mut results = Vec::new();
		self.query_pipeline.intersections_with_shape(
			&self.colliders,
			&position,
			shape,
			InteractionGroups::new(0, mask),
			Some(&filter),
			|handle| {
				let c = &self.colliders[handle];
				results.push(if let Ok(c) = body::ColliderUserdata::try_from(c) {
					IntersectShapeResult {
						index: BodyOrAreaIndex::Body(c.index()),
						shape: c.shape(),
					}
				} else {
					let c = area::ColliderUserdata::try_from(c).expect("Invalid collider userdata");
					IntersectShapeResult {
						index: BodyOrAreaIndex::Area(c.index()),
						shape: c.shape(),
					}
				});
				results.len() <= max_results
			},
		);
		results.into_iter()
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

	/// Returns a reference to a body if it exists
	pub fn get_body(&self, body: RigidBodyHandle) -> Option<&RigidBody> {
		self.bodies.get(body)
	}

	/// Returns a mutable reference to a body if it exists
	pub fn get_body_mut(&mut self, body: RigidBodyHandle) -> Option<&mut RigidBody> {
		self.bodies.get_mut(body)
	}

	/// Returns a mutable reference to a collider if it exists
	pub fn get_collider_mut(&mut self, collider: ColliderHandle) -> Option<&mut Collider> {
		self.colliders.get_mut(collider)
	}

	/// Sets the amount of debug contacts to keep track of
	pub fn set_debug_contact_count(&mut self, count: usize) {
		self.debug_contact_count = count;
	}

	/// Gets the current debug contacts
	pub fn debug_contacts(&mut self) -> &[Vector3] {
		if self.debug_contact_count > 0 && self.debug_contacts.is_empty() {
			self.debug_contacts = self
				.narrow_phase
				.contact_pairs()
				.flat_map(|cp| {
					let pos = *self.colliders[cp.collider1].position();
					cp.manifolds.iter().map(move |cm| (cm, pos))
				})
				.flat_map(|(cm, pos)| {
					cm.points
						.iter()
						.map(move |p| pos.transform_point(&p.local_p1))
				})
				.map(|p| vec_na_to_gd(p.coords))
				.collect();
		}
		self.debug_contacts.as_slice()
	}

	/// Gets the total amount of collision pairs
	pub fn collision_pair_count(&self) -> usize {
		// contact_pairs: The description of all the contacts between a **pair of colliders**.
		self.narrow_phase.contact_pairs().count()
	}

	/// Return all colliders that intersect with a given ray.
	pub fn intersections_with_ray(
		&mut self,
		origin: Vector3,
		direction: Vector3,
		solid: bool,
		mask: u32,
		exclude_bodies: Option<&[BodyIndex]>,
		exclude_areas: Option<&[AreaIndex]>,
		mut callback: impl FnMut(BodyOrAreaIndex, u32, RayIntersection) -> bool,
	) {
		self.update_query_pipeline();
		self.query_pipeline.intersections_with_ray(
			&self.colliders,
			&Ray::new(
				Point::from(vec_gd_to_na(origin)),
				vec_gd_to_na(direction.normalize()),
			),
			direction.length(),
			solid,
			InteractionGroups::new(0, mask),
			Some(&|handle| {
				let ud = self.colliders[handle].user_data;
				body::ColliderUserdata::try_from(ud)
					.map(|ud| exclude_bodies.map_or(true, |eb| !eb.contains(&ud.index())))
					.or_else(|_| {
						area::ColliderUserdata::try_from(ud)
							.map(|ud| exclude_areas.map_or(true, |ea| !ea.contains(&ud.index())))
					})
					.expect("Collider without area or body index")
			}),
			|handle, ray_intersection| {
				let ud = self.colliders[handle].user_data;
				body::ColliderUserdata::try_from(ud)
					.map(|ud| (BodyOrAreaIndex::Body(ud.index()), ud.shape()))
					.or_else(|_| {
						area::ColliderUserdata::try_from(ud)
							.map(|ud| (BodyOrAreaIndex::Area(ud.index()), ud.shape()))
					})
					.map(|(index, shape_index)| callback(index, shape_index, ray_intersection))
					.expect("Collider without area or body index")
			},
		)
	}
}

#[derive(Debug)]
pub struct ExclusionAlreadyExists;

#[derive(Debug)]
pub struct ExclusionDoesntExist;

impl BodyExclusionHooks {
	fn new(contacts_sender: Sender<(BodyIndex, body::ContactEvent)>) -> Self {
		Self {
			exclusions: Vec::<Vec<BodyIndex>>::new(),
			contacts_sender,
		}
	}

	fn add_exclusion(
		&mut self,
		index_a: BodyIndex,
		index_b: BodyIndex,
	) -> Result<(), ExclusionAlreadyExists> {
		// TODO how should we handle self-exclusions? (index_a == index_b)
		let (a, b) = (index_a, index_b);
		let (a, b) = if a.index() < b.index() {
			(a, b)
		} else {
			(b, a)
		};
		if let Some(vec) = self.exclusions.get_mut(a.index() as usize) {
			if vec.contains(&b) {
				Err(ExclusionAlreadyExists)
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
	fn remove_exclusion(
		&mut self,
		index_a: BodyIndex,
		index_b: BodyIndex,
	) -> Result<(), ExclusionDoesntExist> {
		let (a, b) = (index_a, index_b);
		let (a, b) = if a.index() < b.index() {
			(a, b)
		} else {
			(b, a)
		};
		if let Some(vec) = self.exclusions.get_mut(a.index() as usize) {
			vec.swap_remove(
				vec.iter()
					.position(|&e| e == b)
					.ok_or(ExclusionDoesntExist)?,
			);
			Ok(())
		} else {
			Err(ExclusionDoesntExist)
		}
	}
}

impl PhysicsHooks<RigidBodySet, ColliderSet> for BodyExclusionHooks {
	fn filter_contact_pair(
		&self,
		context: &PairFilterContext<RigidBodySet, ColliderSet>,
	) -> Option<SolverFlags> {
		let mut monitor = false;
		let rb1 = &context.bodies[context.rigid_body1.unwrap()];
		if let Ok(a) = body::RigidBodyUserdata::try_from(rb1) {
			let rb2 = &context.bodies[context.rigid_body2.unwrap()];
			if let Ok(b) = body::RigidBodyUserdata::try_from(rb2) {
				monitor = a.monitoring() || b.monitoring();
				let (a, b) = (a.index(), b.index());
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
			}
		}
		Some(if monitor {
			SolverFlags::all()
		} else {
			SolverFlags::default()
		})
	}

	fn filter_intersection_pair(&self, _: &PairFilterContext<RigidBodySet, ColliderSet>) -> bool {
		false
	}

	fn modify_solver_contacts(
		&self,
		context: &mut ContactModificationContext<RigidBodySet, ColliderSet>,
	) {
		let c1 = &context.colliders[context.collider1];
		if let Ok(a) = body::ColliderUserdata::try_from(c1) {
			let c2 = &context.colliders[context.collider2];
			if let Ok(b) = body::ColliderUserdata::try_from(c2) {
				for c in context.solver_contacts.iter() {
					if a.monitoring() {
						let contact = body::ContactEvent::new(
							vec_na_to_gd(c.point.coords),
							b.index(),
							b.shape(),
							a.shape(),
							vec_na_to_gd(*context.normal),
						);
						let _ = self.contacts_sender.try_send((a.index(), contact));
					}
					if b.monitoring() {
						let contact = body::ContactEvent::new(
							vec_na_to_gd(c.point.coords),
							a.index(),
							a.shape(),
							b.shape(),
							vec_na_to_gd(*context.normal),
						);
						let _ = self.contacts_sender.try_send((b.index(), contact));
					}
				}
			}
		}
	}
}

impl EventHandler for IntersectionEventCollector {
	fn handle_intersection_event(&self, event: IntersectionEvent) {
		let _ = self.sender.try_send(event);
	}

	fn handle_contact_event(&self, _: ContactEvent, _: &ContactPair) {}
}
