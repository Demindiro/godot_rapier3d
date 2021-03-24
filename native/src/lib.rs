#![allow(dead_code)]
#![allow(unused_imports)]

mod body;
mod collider;
mod util;

use gdnative::api::World;
use gdnative::prelude::*;
use lazy_static::lazy_static;
use rapier3d::dynamics::{IntegrationParameters, JointSet, RigidBodySet, RigidBody, RigidBodyHandle};
use rapier3d::geometry::{BroadPhase, Collider, ColliderHandle, ColliderSet, NarrowPhase};
use rapier3d::pipeline::PhysicsPipeline;
use std::collections::HashMap;
use std::sync::RwLock;

use util::*;

lazy_static! {
    static ref WORLDS: RwLock<HashMap<Ref<World>, World3D>> = RwLock::new(HashMap::new());
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
        self.step(owner, delta);
    }

    #[export]
    fn step(&mut self, _owner: TRef<Node>, delta: f32) {
        let mut worlds = WORLDS.write().expect("Failed to write to WORLDS");
        for world in worlds.values_mut() {
            world.pipeline.step(
                &vec_gd_to_na(world.gravity),
                &world.integration_parameters,
                &mut world.broad_phase,
                &mut world.narrow_phase,
                &mut world.bodies,
                &mut world.colliders,
                &mut world.joints,
                &world.physics_hooks,
                &world.event_handler,
            );
        }
    }
}

macro_rules! get_worlds_mut {
    () => {
        WORLDS.write().expect("Failed to write WORLDS")
    };
}

macro_rules! get_worlds {
    () => {
        WORLDS.read().expect("Failed to read WORLDS")
    };
}

fn add_rigid_body(world: Ref<World>, body: RigidBody) -> RigidBodyHandle {
    get_worlds_mut!()
        .entry(world)
        .or_insert_with(create_world)
        .bodies
        .insert(body)
}

fn add_collider(
    world: &Ref<World>,
    body: RigidBodyHandle,
    collider: Collider,
) -> Result<ColliderHandle, ()> {
	let mut worlds = get_worlds_mut!();
    let world = worlds.get_mut(world).ok_or(())?;
    Ok(world.colliders.insert(collider, body, &mut world.bodies))
}

fn remove_rigid_body(world: &Ref<World>, body: RigidBodyHandle) -> Result<(), ()> {
	let mut worlds = get_worlds_mut!();
	let world = worlds.get_mut(world).ok_or(())?;
	world.bodies.remove(body, &mut world.colliders, &mut world.joints).map(|_| ()).ok_or(())
}

fn remove_collider(world: &Ref<World>, collider: ColliderHandle) -> Result<(), ()> {
	let mut worlds = get_worlds_mut!();
	let world = worlds.get_mut(world).ok_or(())?;
	world.colliders.remove(collider, &mut world.bodies, true).map(|_| ()).ok_or(())
}

fn create_world() -> World3D {
	let mut integration_parameters = IntegrationParameters::default();
	integration_parameters.dt = 1.0 / 60.0; // TODO get this value from project settings
    World3D {
        pipeline: PhysicsPipeline::new(),
        gravity: Vector3::new(0.0, -9.81, 0.0),
        integration_parameters,
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

fn get_transform(world: &Ref<World>, handle: RigidBodyHandle) -> Result<Transform, ()> {
	let worlds = get_worlds!();
	let world = worlds.get(world).ok_or(())?;
	let isometry = world.bodies.get(handle).map(|b| b.position()).ok_or(())?;
	let rotation: rapier3d::na::Rotation3<_> = isometry.rotation.into();
	let mat = rotation.matrix();
	let basis = Basis {
		elements: [
			Vector3::new(mat.m11, mat.m12, mat.m13),
			Vector3::new(mat.m21, mat.m22, mat.m23),
			Vector3::new(mat.m31, mat.m32, mat.m33),
		]
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
