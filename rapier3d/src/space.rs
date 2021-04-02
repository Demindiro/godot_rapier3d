use crate::server;
use crate::util::*;
use gdnative::prelude::*;
use rapier3d::dynamics::{
	CCDSolver, IntegrationParameters, Joint, JointHandle, JointParams, JointSet, RigidBody,
	RigidBodyHandle, RigidBodySet,
};
use rapier3d::geometry::{
	BroadPhase, Collider, ColliderHandle, ColliderSet, InteractionGroups, NarrowPhase, Ray,
	SolverFlags,
};
use rapier3d::na::Point3;
use rapier3d::pipeline::{
	ContactModificationContext, PairFilterContext, PhysicsHooks, PhysicsHooksFlags,
	PhysicsPipeline, QueryPipeline,
};

pub struct Space {
	physics_pipeline: PhysicsPipeline,
	query_pipeline: QueryPipeline,
	gravity: Vector3,
	integration_parameters: IntegrationParameters,
	broad_phase: BroadPhase,
	narrow_phase: NarrowPhase,
	bodies: RigidBodySet,
	colliders: ColliderSet,
	joints: JointSet,
	ccd_solver: CCDSolver,
	body_exclusions: BodyExclusionHooks,
	event_handler: (),
	query_pipeline_out_of_date: bool,
	index: u32,
	pub enabled: bool,
}

pub struct RayCastResult {
	pub position: Vector3,
	pub normal: Vector3,
	pub body: u32,
	pub shape: u32,
}

struct BodyExclusionHooks {
	// Reasoning for using Vec and Box instead of HashMap & HashSet:
	// * SparseVec is likely densely packed -> not many "holes" in the Vec.
	// * Amount of body exclusions is likely small -> Vec is compact and maybe faster.
	exclusions: Vec<Vec<u32>>,
}

impl Space {
	pub fn new() -> Self {
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
			event_handler: (),
			query_pipeline_out_of_date: false,
			index: u32::MAX,
			enabled: true,
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
	}

	/// Gets the index of this space
	#[allow(dead_code)]
	pub fn index(&self) -> u32 {
		self.index
	}

	/// Sets the index of this space once.
	///
	/// # Panics
	///
	/// Panics if index is already set or if index == u32::MAX
	pub fn set_index(&mut self, index: u32) {
		assert_ne!(index, u32::MAX);
		assert_eq!(self.index, u32::MAX);
		self.index = index;
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
	pub fn add_body_exclusion(&mut self, index_a: u32, index_b: u32) -> Result<(), ()> {
		self.body_exclusions.add_exclusion(index_a, index_b)
	}

	/// Make two bodies interact with each other again.
	/// Returns `Ok` if the bodies did exclude each other, `Err` otherwise.
	#[allow(dead_code)] // FIXME body_remove_collision_exception needs to be implemented
	pub fn remove_body_exclusion(&mut self, index_a: u32, index_b: u32) -> Result<(), ()> {
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
		exclude: &[u32],
	) -> Option<RayCastResult> {
		self.update_query_pipeline();
		let dir = to - from;
		let (dir, max_toi) = (dir.normalize(), dir.length());
		// TODO account for excluded colliders
		// Rapier 0.7 added a `filter` parameter which can be used for the exclusion list.
		let intersection = self.query_pipeline.cast_ray_and_get_normal(
			&mut self.colliders,
			&Ray::new(Point3::new(from.x, from.y, from.z), vec_gd_to_na(dir)),
			max_toi,
			// TODO what is the solid parameter for?
			false,
			InteractionGroups::new(u16::MAX, mask),
			None,
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
}

impl BodyExclusionHooks {
	fn new() -> Self {
		Self {
			exclusions: Vec::new(),
		}
	}

	fn add_exclusion(&mut self, index_a: u32, index_b: u32) -> Result<(), ()> {
		// TODO how should we handle self-exclusions? (index_a == index_b)
		let (a, b) = (index_a, index_b);
		let (a, b) = if a < b { (a, b) } else { (b, a) };
		if let Some(vec) = self.exclusions.get_mut(a as usize) {
			if vec.contains(&b) {
				Err(())
			} else {
				vec.push(b);
				Ok(())
			}
		} else {
			self.exclusions.resize_with(a as usize + 1, || Vec::new());
			self.exclusions[a as usize] = Vec::from([b]);
			Ok(())
		}
	}

	// TODO implement body_remove_exception and remove the allow
	#[allow(dead_code)]
	fn remove_exclusion(&mut self, index_a: u32, index_b: u32) -> Result<(), ()> {
		let (a, b) = (index_a, index_b);
		let (a, b) = if a < b { (a, b) } else { (b, a) };
		if let Some(vec) = self.exclusions.get_mut(a as usize) {
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
		let body_a = server::Body::get_index(context.rigid_body1);
		let body_b = server::Body::get_index(context.rigid_body2);
		if let Some(indices) = self.exclusions.get(body_a as usize) {
			for &i in indices.iter() {
				if i == body_b {
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
