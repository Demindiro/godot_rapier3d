#![feature(destructuring_assignment)]

mod server;
mod util;

// TODO find a proper workaround for E0446
pub use server::Index;

use gdnative::api::World;
use gdnative::prelude::*;
use lazy_static::lazy_static;
use rapier3d::dynamics::{
	CCDSolver, IntegrationParameters, JointSet, RigidBody, RigidBodyHandle, RigidBodySet,
};
use rapier3d::geometry::{BroadPhase, Collider, ColliderSet, NarrowPhase, SolverFlags};
use rapier3d::pipeline::{
	ContactModificationContext, PairFilterContext, PhysicsHooks, PhysicsHooksFlags,
	PhysicsPipeline, QueryPipeline,
};
use std::collections::HashMap;
use std::sync::RwLock;

extern "C" {
	fn feenableexcept(flags: i32);
}

/// Call this if you are getting NaN errors *somewhere*
#[allow(dead_code)]
fn enable_sigfpe() {
	unsafe {
		feenableexcept(9);
	}
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct SpaceHandle(usize);

use util::*;

lazy_static! {
	static ref SPACES: RwLock<Vec<Option<World3D>>> = RwLock::new(Vec::new());
	static ref WORLD_TO_SPACE: RwLock<HashMap<Ref<World>, SpaceHandle>> =
		RwLock::new(HashMap::new());
}

struct World3D {
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
	index: Option<Index>,
	enabled: bool,
}

struct BodyExclusionHooks {
	// Reasoning for using Vec and Box instead of HashMap & HashSet:
	// * SparseVec is likely densely packed -> not many "holes" in the Vec.
	// * Amount of body exclusions is likely small -> Vec is compact and maybe faster.
	exclusions: Vec<Vec<u32>>,
}

impl World3D {
	fn step(&mut self, delta: f32) {
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

	/// Ensures the query pipeline is up to date if any bodies were added
	fn update_query_pipeline(&mut self) {
		if self.query_pipeline_out_of_date {
			self.query_pipeline.update(&self.bodies, &self.colliders);
			self.query_pipeline_out_of_date = false;
		}
	}
}

macro_rules! get_spaces {
	() => {
		SPACES.read().expect("Failed to read WORLDS")
	};
}

macro_rules! get_spaces_mut {
	() => {
		SPACES.write().expect("Failed to write WORLDS")
	};
}

impl SpaceHandle {
	fn modify<F, R>(self, f: F) -> Result<R, ()>
	where
		F: FnOnce(&mut World3D) -> R,
	{
		modify_space(self, f)
	}

	fn read<F, R>(self, f: F) -> Result<R, ()>
	where
		F: FnOnce(&World3D) -> R,
	{
		let spaces = get_spaces!();
		let space = spaces.get(self.0).and_then(Option::as_ref).ok_or(())?;
		Ok(f(space))
	}

	/// Get the server index of this space
	fn index(&self) -> Result<Option<server::Index>, ()> {
		self.read(|space| space.index)
	}

	/// Set the server index of this space
	fn set_index(&self, index: Option<server::Index>) -> Result<(), ()> {
		self.modify(|space| space.index = index)
	}
}

impl BodyExclusionHooks {
	fn new() -> Self {
		Self {
			exclusions: Vec::new(),
		}
	}

	fn add_exclusion(&mut self, index_a: u32, index_b: u32) {
		// TODO how should we handle self-exclusions? (index_a == index_b)
		let (a, b) = (index_a, index_b);
		let (a, b) = if a < b { (a, b) } else { (b, a) };
		if let Some(vec) = self.exclusions.get_mut(a as usize) {
			//assert!(!vec.contains(&b));
			vec.push(b);
		} else {
			self.exclusions.resize_with(a as usize + 1, || Vec::new());
			self.exclusions[a as usize] = Vec::from([b]);
		}
	}

	// TODO implement body_remove_exception and remove the allow
	#[allow(dead_code)]
	fn remove_exclusion(&mut self, index_a: u32, index_b: u32) {
		let (a, b) = (index_a, index_b);
		let (a, b) = if a < b { (a, b) } else { (b, a) };
		if let Some(vec) = self.exclusions.get_mut(a as usize) {
			vec.swap_remove(
				vec.iter()
					.position(|&e| e == b)
					.expect("B not found in A's exclusions"),
			);
		} else {
			panic!("A has no exclusions");
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

fn step_all_spaces(delta: f32) {
	let mut worlds = SPACES.write().expect("Failed to write to WORLDS");
	for world in worlds
		.iter_mut()
		.filter_map(Option::as_mut)
		.filter(|w| w.enabled)
	{
		world.step(delta);
	}
}

fn modify_rigid_body<F>(space: SpaceHandle, body: RigidBodyHandle, f: F) -> Result<(), ()>
where
	F: FnOnce(&mut RigidBody),
{
	let mut spaces = get_spaces_mut!();
	let space = spaces.get_mut(space.0).and_then(Option::as_mut).ok_or(())?;
	f(space.bodies.get_mut(body).ok_or(())?);
	Ok(())
}

fn create_space() -> SpaceHandle {
	let mut spaces = get_spaces_mut!();
	let world = Some(create_world());
	for (i, e) in spaces.iter_mut().enumerate() {
		if let None = e {
			*e = world;
			return SpaceHandle(i);
		}
	}
	let index = spaces.len();
	spaces.push(world);
	SpaceHandle(index)
}

fn modify_space<F, R>(space: SpaceHandle, f: F) -> Result<R, ()>
where
	F: FnOnce(&mut World3D) -> R,
{
	let mut spaces = get_spaces_mut!();
	let space = spaces.get_mut(space.0).and_then(Option::as_mut).ok_or(())?;
	// TODO add some wrapper methods that will set this only when necessary
	space.query_pipeline_out_of_date = true;
	Ok(f(space))
}

fn create_world() -> World3D {
	World3D {
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
		index: None,
		enabled: true,
	}
}

/// Bogus method just so not everything gets optimized away
/// Use this with `godot_nativescript_init`
pub fn init(_: InitHandle) {}
