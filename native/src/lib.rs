#![allow(dead_code)]
#![allow(unused_imports)]

mod body;
mod collider;
mod server;
mod util;

// TODO find a proper workaround for E0446
pub use server::Index;

use gdnative::api::World;
use gdnative::prelude::*;
use lazy_static::lazy_static;
use rapier3d::dynamics::{
	IntegrationParameters, JointSet, RigidBody, RigidBodyHandle, RigidBodySet,
};
use rapier3d::geometry::{BroadPhase, Collider, ColliderHandle, ColliderSet, NarrowPhase};
use rapier3d::pipeline::PhysicsPipeline;
use std::collections::HashMap;
use std::sync::RwLock;

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct SpaceHandle(usize);

use util::*;

lazy_static! {
	static ref SPACES: RwLock<Vec<Option<World3D>>> = RwLock::new(Vec::new());
	static ref WORLD_TO_SPACE: RwLock<HashMap<Ref<World>, SpaceHandle>> =
		RwLock::new(HashMap::new());
}

struct World3D {
	pipeline: PhysicsPipeline,
	gravity: Vector3,
	integration_parameters: IntegrationParameters,
	broad_phase: BroadPhase,
	narrow_phase: NarrowPhase,
	bodies: RigidBodySet,
	colliders: ColliderSet,
	joints: JointSet,
	physics_hooks: (),
	event_handler: (),
}

#[derive(NativeClass)]
#[inherit(Node)]
struct Rapier3D;

#[methods]
impl Rapier3D {
	fn new(_owner: TRef<Node>) -> Self {
		Self
	}

	#[export]
	fn _physics_process(&mut self, owner: TRef<Node>, delta: f32) {
		let _ = (owner, delta);
		// TODO remove the autoload dummy
		//self.step(owner, delta);
	}

	#[export]
	fn step(&mut self, _owner: TRef<Node>, delta: f32) {
		step_all_spaces(delta);
	}
}

impl World3D {
	fn step(&mut self, delta: f32) {
		self.integration_parameters.dt = delta;
		self.pipeline.step(
			&vec_gd_to_na(self.gravity),
			&self.integration_parameters,
			&mut self.broad_phase,
			&mut self.narrow_phase,
			&mut self.bodies,
			&mut self.colliders,
			&mut self.joints,
			&self.physics_hooks,
			&self.event_handler,
		);
	}
}

impl SpaceHandle {
	fn modify<F, R>(self, f: F) -> Result<R, ()>
	where
		F: FnOnce(&mut World3D) -> R
	{
		modify_space(self, f)
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

macro_rules! get_world_to_space_mut {
	() => {
		WORLD_TO_SPACE
			.write()
			.expect("Failed to write WORLD_TO_SPACE")
	};
}

fn step_all_spaces(delta: f32) {
	let mut worlds = SPACES.write().expect("Failed to write to WORLDS");
	for world in worlds.iter_mut().filter_map(Option::as_mut) {
		world.step(delta);
	}
}

fn add_rigid_body(world: Ref<World>, body: RigidBody) -> (RigidBodyHandle, SpaceHandle) {
	let &mut handle = get_world_to_space_mut!()
		.entry(world)
		.or_insert_with(create_space);
	(
		get_spaces_mut!()[handle.0]
			.as_mut()
			.unwrap()
			.bodies
			.insert(body),
		handle,
	)
}

fn add_collider(
	space: SpaceHandle,
	body: RigidBodyHandle,
	collider: Collider,
) -> Result<ColliderHandle, ()> {
	let mut spaces = get_spaces_mut!();
	let space = spaces.get_mut(space.0).and_then(Option::as_mut).ok_or(())?;
	Ok(space.colliders.insert(collider, body, &mut space.bodies))
}

fn remove_rigid_body(space: SpaceHandle, body: RigidBodyHandle) -> Result<(), ()> {
	let mut worlds = get_spaces_mut!();
	let world = worlds.get_mut(space.0).and_then(Option::as_mut).ok_or(())?;
	world
		.bodies
		.remove(body, &mut world.colliders, &mut world.joints)
		.map(|_| ())
		.ok_or(())
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

fn remove_collider(space: SpaceHandle, collider: ColliderHandle) -> Result<(), ()> {
	let mut spaces = get_spaces_mut!();
	let space = spaces.get_mut(space.0).and_then(Option::as_mut).ok_or(())?;
	space
		.colliders
		.remove(collider, &mut space.bodies, true)
		.map(|_| ())
		.ok_or(())
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
	Ok(f(space))
}

fn create_world() -> World3D {
	World3D {
		pipeline: PhysicsPipeline::new(),
		gravity: Vector3::new(0.0, -9.81, 0.0),
		integration_parameters: IntegrationParameters::default(),
		broad_phase: BroadPhase::new(),
		narrow_phase: NarrowPhase::new(),
		bodies: RigidBodySet::new(),
		colliders: ColliderSet::new(),
		joints: JointSet::new(),
		// We ignore physics hooks and contact events for now.
		physics_hooks: (),
		event_handler: (),
	}
}

fn get_transform(space: SpaceHandle, handle: RigidBodyHandle) -> Result<Transform, ()> {
	let spaces = get_spaces!();
	let space = spaces.get(space.0).and_then(Option::as_ref).ok_or(())?;
	let isometry = space.bodies.get(handle).map(|b| b.position()).ok_or(())?;
	let rotation: rapier3d::na::Rotation3<_> = isometry.rotation.into();
	let mat = rotation.matrix();
	let basis = Basis {
		elements: [
			Vector3::new(mat.m11, mat.m12, mat.m13),
			Vector3::new(mat.m21, mat.m22, mat.m23),
			Vector3::new(mat.m31, mat.m32, mat.m33),
		],
	};
	let transform = Transform {
		basis,
		origin: vec_na_to_gd(isometry.translation.vector),
	};
	Ok(transform)
}

fn init(handle: InitHandle) {
	handle.add_class::<Rapier3D>();
	handle.add_class::<body::RigidBody>();
	handle.add_class::<body::StaticBody>();
	handle.add_class::<collider::Box>();
}

godot_gdnative_init!(_ as gd_rapier3d_gdnative_init);
godot_nativescript_init!(init as gd_rapier3d_nativescript_init);
godot_gdnative_terminate!(_ as gd_rapier3d_gdnative_terminate);
