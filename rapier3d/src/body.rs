use crate::area::Gravity;
use crate::indices;
use crate::indices::Indices;
use crate::server::{BodyIndex, Instance, MapIndex, ObjectID, Shape, ShapeIndex, SpaceIndex};
use crate::space::Space;
use crate::util::*;
use core::convert::{TryFrom, TryInto};
use gdnative::core_types::*;
use rapier3d::dynamics::{BodyStatus, MassProperties, RigidBody, RigidBodyHandle, RigidBodySet};
use rapier3d::geometry::{
	Collider, ColliderBuilder, ColliderHandle, InteractionGroups, SharedShape,
};
use rapier3d::math::Isometry;
use rapier3d::na::Point3;

pub struct BodyShape {
	index: ShapeIndex,
	transform: Isometry<f32>,
	scale: Vector3,
	enabled: bool,
}

#[derive(Debug)]
pub struct ContactEvent {
	other_index: BodyIndex,
	position: Vector3,
	other_shape: u32,
	self_shape: u32,
	normal: Vector3,
}

pub struct RigidBodyUserdata(u128);
pub struct ColliderUserdata(u128);

#[derive(Debug)]
pub struct UserdataMismatch;

pub struct Body {
	body: Instance<(RigidBodyHandle, Vec<Option<ColliderHandle>>), RigidBody>,
	object_id: Option<ObjectID>,
	shapes: Vec<BodyShape>,
	scale: Vector3,

	exclusions: Vec<BodyIndex>,
	collision_groups: InteractionGroups,
	#[allow(dead_code)]
	ray_pickable: bool,

	linear_damp: f32,
	angular_damp: f32,
	restitution: f32,
	friction: f32,

	inertia_stale: bool,

	area_gravity: Option<Vector3>,
	area_linear_damp: Option<(f32, u32)>,
	area_angular_damp: Option<(f32, u32)>,
	area_replace: bool,
	area_lock: bool,

	max_contacts: u32,
	contacts: Vec<ContactEvent>,

	index: Option<BodyIndex>,
}

#[derive(Debug)]
pub struct AlreadyExcluded;

#[derive(Debug)]
pub struct InvalidShape;

impl Body {
	pub fn new(body: RigidBody) -> Self {
		Self {
			body: Instance::Loose(body),
			object_id: None,
			shapes: Vec::new(),
			scale: Vector3::one(),

			exclusions: Vec::new(),
			collision_groups: InteractionGroups::new(1, 1),
			ray_pickable: true,

			angular_damp: -1.0,
			linear_damp: -1.0,
			restitution: 0.0,
			friction: 1.0,

			inertia_stale: false,

			area_gravity: None,
			area_linear_damp: None,
			area_angular_damp: None,
			area_lock: false,
			area_replace: false,

			max_contacts: 0,
			contacts: Vec::new(),

			index: None,
		}
	}

	pub fn as_attached(&self) -> Option<(RigidBodyHandle, SpaceIndex)> {
		if let Instance::Attached(body, space) = &self.body {
			Some((body.0, *space))
		} else {
			None
		}
	}

	/// Calls the given function with a reference to this body's [`RigidBody`].
	fn map_rigidbody<F, R>(&self, f: F) -> R
	where
		F: FnOnce(&RigidBody) -> R,
	{
		match &self.body {
			Instance::Attached((body, _), space) => space
				.map(|space| f(space.get_body(*body).expect("Invalid body handle")))
				.expect("Invalid space handle"),
			Instance::Loose(body) => f(body),
		}
	}

	/// Calls the given function with a reference to this body's [`RigidBody`].
	fn map_rigidbody_mut<F, R>(&mut self, f: F) -> R
	where
		F: FnOnce(&mut RigidBody) -> R,
	{
		match &mut self.body {
			Instance::Attached((body, _), space) => space
				.map_mut(|space| f(space.get_body_mut(*body).expect("Invalid body handle")))
				.expect("Invalid space handle"),
			Instance::Loose(body) => f(body),
		}
	}

	/// Calls the given function on all of the colliders of this body, if any.
	fn map_colliders<F>(&mut self, mut f: F)
	where
		F: FnMut(&mut Collider),
	{
		if let Instance::Attached((_, ch), space) = &mut self.body {
			space
				.map_mut(|space| {
					for ch in ch.iter().filter_map(Option::as_ref) {
						f(space
							.colliders_mut()
							.get_mut(*ch)
							.expect("Invalid collider handle"));
					}
				})
				.expect("Invalid space handle");
		}
	}

	/// Attaches the given shape to this body
	pub fn add_shape(&mut self, shape: &Shape, transform: &Transform, enabled: bool) {
		let (iso, scl) = transform_to_isometry_and_scale(transform);
		let body_shape = BodyShape {
			index: shape.index(),
			transform: iso,
			enabled,
			scale: scl,
		};
		let self_scale = self.scale;
		if let Instance::Attached((rb, colliders), space) = &mut self.body {
			if enabled {
				space
					.map_mut(|space| {
						let shape_scale = iso.rotation * vec_gd_to_na(scl);
						let shape_scale = vec_gd_to_na(self_scale).component_mul(&shape_scale);
						let shape_scale = vec_na_to_gd(shape_scale);
						let collider = shape.build(body_shape.transform, shape_scale, false);
						let collider = space.add_collider(collider, *rb);
						colliders.push(Some(collider));
					})
					.expect("Invalid space");
			} else {
				colliders.push(None);
			}
		}
		self.shapes.push(body_shape);
		self.inertia_stale = true
	}

	/// Recalculates the inertia based on all active shapes
	fn recalculate_inertia(&mut self, body: &mut RigidBody, shapes: &mut Indices<Shape>) {
		// TODO avoid calculating the MassProperties twice
		let mut mp = Vec::with_capacity(self.shapes.len());
		for shape in self.shapes.iter() {
			if shape.enabled {
				let trf = shape.transform;
				let shape = shapes.get(shape.index.into()).expect("Invalid shape index");
				mp.push((trf, shape.shape().clone()));
			}
		}
		let mp_f = MassProperties::from_compound(1.0, &mp[..]);
		let ratio = mp_f.inv_mass / body.mass_properties().inv_mass;
		let mut mp = MassProperties::from_compound(ratio, &mp[..]);
		// Workaround for https://github.com/dimforge/parry/issues/20
		if mp.local_com.x.is_nan() || mp.local_com.x.is_infinite() {
			mp.local_com = Point3::new(0.0, 0.0, 0.0);
		};
		body.set_mass_properties(mp, true);
		self.inertia_stale = false;
	}

	pub fn object_id(&self) -> Option<ObjectID> {
		self.object_id
	}

	/// Creates shapes according to shape enable status and transform
	fn create_shapes(&self) -> Vec<Option<(SharedShape, &Isometry<f32>)>> {
		let mut shapes = Vec::with_capacity(self.shapes.len());
		for shape in self.shapes.iter() {
			if shape.enabled {
				let transform = shape.transform;
				let shape_scale = transform.rotation * vec_gd_to_na(shape.scale);
				let shape_scale = vec_gd_to_na(self.scale).component_mul(&shape_scale);
				let shape_scale = vec_na_to_gd(shape_scale);
				shape
					.index
					.map(|s| shapes.push(Some((s.scaled(shape_scale), &shape.transform))))
					.expect("Shape is invalid");
			} else {
				shapes.push(None);
			}
		}
		shapes
	}

	/// Creates colliders according to shape enable status and transform
	fn create_colliders(&self, index: BodyIndex) -> Vec<Option<Collider>> {
		let mut colliders = Vec::with_capacity(self.shapes.len());
		for (i, e) in self.create_shapes().into_iter().enumerate() {
			if let Some((shape, transform)) = e {
				let mut collider = ColliderBuilder::new(shape)
					.position_wrt_parent(*transform)
					.collision_groups(self.collision_groups)
					.restitution(self.restitution)
					.friction(self.friction)
					.build();
				collider.user_data =
					ColliderUserdata::new(index, self.max_contacts > 0, self.ray_pickable, i as u32).into();
				colliders.push(Some(collider));
			} else {
				colliders.push(None);
			}
		}
		colliders
	}

	/// Store the given index in the userdata
	pub fn set_index(&mut self, index: BodyIndex) {
		assert_eq!(self.index, None, "Index is already set");
		self.index = Some(index);
		self.map_rigidbody_mut(|body| body.user_data = RigidBodyUserdata::new(index, false).into());
	}

	/// Returns the index of this body
	pub fn index(&self) -> BodyIndex {
		self.index.expect("Index is not set")
	}

	/// Frees this body, removing it from it's attached space (if any)
	pub fn free(self) {
		if let Instance::Attached((rb, _), space) = &self.body {
			space
				.map_mut(|space| {
					space.remove_body(*rb);
				})
				.expect("Invalid space");
		}
	}

	/// Adds any forces and damp overrides from an area
	pub fn area_apply_overrides(
		&mut self,
		bodies: &RigidBodySet,
		replace: bool,
		gravity: &Gravity,
		linear_damp: f32,
		angular_damp: f32,
		lock: bool,
	) {
		if !self.area_lock {
			let g = match gravity {
				Gravity::Direction(g) => g.gravity(),
				Gravity::Point(p) => {
					if let Instance::Attached((rb, _), _) = &self.body {
						// Based on code in rigid_body_bullet.cpp
						// I'm not sure what the logic behind the scale factor is but w/e
						let body = &bodies[*rb];
						let dir = p.point() - vec_na_to_gd(body.position().translation.vector);
						let dist = dir.length();
						if dist > 0.0 {
							let scale = p.distance_scale();
							if scale > 0.0 {
								let f = dist.mul_add(p.distance_scale(), 1.0);
								(dir * p.gravity()) / ((f * f) * dist)
							} else {
								(dir * p.gravity()) / dist
							}
						} else {
							// We can't divide by 0, so pretend there is no force in the middle of a thing
							Vector3::zero()
						}
					} else {
						unreachable!();
					}
				}
			};
			if replace {
				self.area_gravity = Some(g);
				self.area_linear_damp = if linear_damp >= 0.0 {
					Some((linear_damp, 1))
				} else {
					None
				};
				self.area_angular_damp = if angular_damp >= 0.0 {
					Some((angular_damp, 1))
				} else {
					None
				};
				self.area_replace = true;
			} else {
				self.area_gravity = if let Some(ag) = self.area_gravity {
					Some(ag + g)
				} else {
					Some(g)
				};
				self.area_linear_damp = if let Some((d, i)) = self.area_linear_damp {
					Some((linear_damp + d, i + 1))
				} else {
					Some((linear_damp, 1))
				};
				self.area_angular_damp = if let Some((d, i)) = self.area_linear_damp {
					Some((angular_damp + d, i + 1))
				} else {
					Some((angular_damp, 1))
				};
			}
			self.area_lock = lock;
		}
	}

	/// Applies any forces and damp overrides added by areas and clears the area lock
	pub fn apply_area_overrides(
		&mut self,
		body: &mut RigidBody,
		space_linear_damp: f32,
		space_angular_damp: f32,
	) {
		let current_gravity_scale = body.gravity_scale();
		#[allow(clippy::float_cmp)] // Shut up Clippy
		if let Some(g) = self.area_gravity {
			let s = if self.area_replace { 0.0 } else { 1.0 };
			body.set_gravity_scale(s, s != current_gravity_scale);
			body.apply_force(vec_gd_to_na(g) * body.mass(), s != current_gravity_scale);
		} else {
			body.set_gravity_scale(1.0, current_gravity_scale != 1.0);
		}
		body.linear_damping = if let Some((d, i)) = self.area_linear_damp {
			let i = i as f32;
			if self.area_replace {
				d / i
			} else if self.linear_damp < 0.0 {
				(space_linear_damp + d * i) / (i + 1.0)
			} else {
				(self.linear_damp + d * i) / (i + 1.0)
			}
		} else if self.linear_damp < 0.0 {
			space_linear_damp
		} else {
			self.linear_damp
		};
		body.angular_damping = if let Some((d, i)) = self.area_angular_damp {
			let i = i as f32;
			if self.area_replace {
				d / i
			} else if self.angular_damp < 0.0 {
				(space_angular_damp + d * i) / (i + 1.0)
			} else {
				(self.angular_damp + d * i) / (i + 1.0)
			}
		} else if self.linear_damp < 0.0 {
			space_linear_damp
		} else {
			self.linear_damp
		};
		self.area_gravity = None;
		self.area_linear_damp = None;
		self.area_angular_damp = None;
		self.area_replace = false;
		self.area_lock = false;
	}

	/// Adds body with which this body will not collide with
	///
	/// # Returns
	///
	/// `Err(AlreadyExcluded)` if the bodies already excluded each other, `Ok(())` otherwise
	pub fn add_exclusion(&mut self, other: &mut Self) -> Result<(), AlreadyExcluded> {
		let other_index = other.index();
		if !self.exclusions.contains(&other_index) {
			let self_index = self.index();
			assert!(
				!other.exclusions.contains(&self_index),
				"Other body already excludes self"
			);
			self.exclusions.push(other_index);
			other.exclusions.push(self_index);
			if let Instance::Attached(_, space) = &self.body {
				space
					.map_mut(|space| {
						space
							.add_body_exclusion(self_index, other_index)
							.expect("Exclusion already added");
					})
					.expect("Invalid space");
			}
			Ok(())
		} else {
			Err(AlreadyExcluded)
		}
	}

	/// Applies a force to the body at the given position. The force must be in global space.
	pub fn add_force_at_position(&mut self, force: Vector3, position: Vector3) {
		let force = vec_gd_to_na(force);
		let position = Point3::from(vec_gd_to_na(position));
		self.map_rigidbody_mut(|body| body.apply_force_at_point(force, position, true));
	}

	/// Applies an impulse to the body at the given position. The impulse must be in global space.
	/// The impulse is applied immediately.
	pub fn add_impulse_at_position(&mut self, impulse: Vector3, position: Vector3) {
		let impulse = vec_gd_to_na(impulse);
		let position = Point3::from(vec_gd_to_na(position));
		self.map_rigidbody_mut(|body| body.apply_impulse_at_point(impulse, position, true));
	}

	/// Returns the translation of this body
	pub fn translation(&self) -> Vector3 {
		self.map_rigidbody(|body| vec_na_to_gd(body.position().translation.vector))
	}

	/// Applies the transform and scales the colliders
	pub fn set_transform(&mut self, transform: &Transform) {
		let (iso, scl) = transform_to_isometry_and_scale(transform);
		self.scale = scl;
		self.map_rigidbody_mut(|body| body.set_position(iso, true));
		// FIXME inefficient as hell
		let shapes = self
			.create_shapes()
			.into_iter()
			.map(|v| v.map(|v| v.0))
			.collect::<Vec<_>>();
		self.inertia_stale |= match &mut self.body {
			Instance::Attached((body, _), space) => {
				let body = *body;
				space
					.map_mut(|space| {
						let body = space.get_body_mut(body).expect("Invalid body handle");
						body.set_position(iso, true);
						let iter = body
							.colliders()
							.iter()
							.copied()
							.zip(shapes.into_iter())
							.collect::<Vec<_>>();
						for (handle, shape) in iter {
							space
								.get_collider_mut(handle)
								.expect("Invalid collider handle")
								.set_shape(shape.unwrap());
						}
					})
					.expect("Invalid space handle");
				true
			}
			Instance::Loose(body) => {
				body.set_position(iso, true);
				false
			}
		}
	}

	/// Sets the linear velocity of this body
	pub fn set_linear_velocity(&mut self, velocity: Vector3) {
		self.map_rigidbody_mut(|body| body.set_linvel(vec_gd_to_na(velocity), true));
	}

	/// Sets the angular velocity of this body
	pub fn set_angular_velocity(&mut self, velocity: Vector3) {
		self.map_rigidbody_mut(|body| body.set_angvel(vec_gd_to_na(velocity), true));
	}

	/// Sets whether this body should sleep or not
	pub fn set_sleeping(&mut self, sleep: bool) {
		self.map_rigidbody_mut(|body| body.activation.sleeping = sleep);
	}

	/// Sets the energy threshold when this body will wake up or sleep.
	/// A negative value disables sleep.
	pub fn set_sleep_threshold(&mut self, threshold: f32) {
		self.map_rigidbody_mut(|body| body.activation.threshold = threshold);
	}

	/// Attach or remove a object id to this body.
	pub fn set_object_id(&mut self, id: Option<ObjectID>) {
		self.object_id = id;
	}

	/// Sets whether this body can be picked up by raycasts.
	pub fn set_ray_pickable(&mut self, pickable: bool) {
		self.ray_pickable = pickable;
		self.map_colliders(|collider| {
			let mut ud = ColliderUserdata::try_from(collider.user_data).expect("Invalid collider userdata");
			ud.set_ray_pickable(pickable);
			collider.user_data = ud.into();
		})
	}

	// TODO get rid of this ugly thing and return some state struct instead
	pub fn read_body<F>(&self, f: F)
	where
		F: FnOnce(&RigidBody, Option<SpaceIndex>),
	{
		match &self.body {
			Instance::Attached((b, _), sh) => {
				sh.map(|s| f(s.get_body(*b).expect("Invalid body handle"), Some(*sh)))
					.expect("Invalid space handle");
			}
			Instance::Loose(b) => f(b, None),
		}
	}

	/// Sets the space of this body
	pub fn set_space(&mut self, space: &mut Space) {
		if let Instance::Attached(_, _) = &self.body {
			self.remove_from_space();
		}
		let colliders = self.create_colliders(self.index());

		self.inertia_stale = true;

		let self_index = self.index();
		// TODO this is ugly
		let b = core::mem::replace(
			&mut self.body,
			Instance::Attached(
				(RigidBodyHandle::invalid(), Vec::new()),
				SpaceIndex::new(0, 0),
			),
		);

		self.body = if let Instance::Loose(b) = b {
			let mut collider_handles = Vec::with_capacity(colliders.len());
			let mp = *b.mass_properties();
			let handle = space.add_body(b);
			for collider in colliders {
				let handle = collider.map(|c| space.add_collider(c, handle));
				collider_handles.push(handle);
			}
			for &exclude in self.exclusions.iter() {
				let _ = space.add_body_exclusion(self_index, exclude);
			}
			space
				.get_body_mut(handle)
				.unwrap()
				.set_mass_properties(mp, false);
			let rb = space.get_body_mut(handle).unwrap();
			self.recalculate_inertia(rb, &mut ShapeIndex::write_all());
			Instance::Attached((handle, collider_handles), space.index())
		} else {
			unreachable!();
		}
	}

	/// Removes the body from it's space, if any
	pub fn remove_from_space(&mut self) {
		if let Instance::Attached((body, _), space) = &self.body {
			let body = space
				.map_mut(|space| space.remove_body(*body).expect("Invalid body handle"))
				.expect("Failed to modify space");
			self.body = Instance::Loose(body);
		}
	}

	/// Sets the transform of a shape and optionally scales it
	pub fn set_shape_transform(
		&mut self,
		shape: u32,
		transform: &Transform,
	) -> Result<(), InvalidShape> {
		if let Some(shp) = self.shapes.get_mut(shape as usize) {
			let (iso, scl) = transform_to_isometry_and_scale(transform);
			shp.transform = iso;
			shp.scale = scl;
			if let Instance::Attached((_, colliders), space) = &self.body {
				if let Some(collider) = colliders[shape as usize] {
					let shape_scale = iso.rotation * vec_gd_to_na(shp.scale);
					let shape_scale = vec_gd_to_na(self.scale).component_mul(&shape_scale);
					let shape_scale = vec_na_to_gd(shape_scale);
					let shape = shp
						.index
						.map(|shape| shape.scaled(shape_scale))
						.expect("Invalid shape index");
					space
						.map_mut(|space| {
							let c = space
								.get_collider_mut(collider)
								.expect("Invalid collider handle");
							c.set_shape(shape);
							c.set_position_wrt_parent(iso);
						})
						.expect("Invalid space index");
				}
			}
			Ok(())
		} else {
			Err(InvalidShape)
		}
	}

	/// Sets the transform of a shape and optionally scales it
	pub fn set_shape_enable(&mut self, shape: u32, enable: bool) -> Result<(), InvalidShape> {
		if let Some(shape) = self.shapes.get_mut(shape as usize) {
			shape.enabled = enable;
			Ok(())
		} else {
			Err(InvalidShape)
		}
	}

	/// Sets whether to clear any external forces such as gravity
	pub fn set_omit_force_integration(&mut self, enable: bool) {
		let g_scale = if enable { 0.0 } else { 1.0 };
		match &mut self.body {
			Instance::Attached((rb, _), space) => {
				space
					.map_mut(|space| {
						space
							.bodies_mut()
							.get_mut(*rb)
							.expect("Invalid body handle")
							.set_gravity_scale(g_scale, true);
					})
					.expect("Invalid space");
			}
			Instance::Loose(body) => {
				body.set_gravity_scale(g_scale, true);
			}
		}
	}

	/// Sets whether this body is static, kinematic or dynamic
	pub fn set_body_status(&mut self, status: BodyStatus) {
		match &mut self.body {
			Instance::Attached((rb, _), space) => {
				space
					.map_mut(|s| {
						s.bodies_mut()
							.get_mut(*rb)
							.expect("Invalid body handle")
							.set_body_status(status)
					})
					.expect("Invalid space");
			}
			Instance::Loose(body) => {
				body.set_body_status(status);
			}
		}
	}

	/// Sets the groups of this body
	pub fn set_groups(&mut self, groups: u32) {
		self.collision_groups = self.collision_groups.with_groups(groups);
		self.update_interaction_groups();
	}

	/// Sets the mask of this body
	pub fn set_mask(&mut self, mask: u32) {
		self.collision_groups = self.collision_groups.with_mask(mask);
		self.update_interaction_groups();
	}

	/// Updates the [`InteractionGroups`] of the colliders attached to this body
	fn update_interaction_groups(&mut self) {
		let cg = self.collision_groups;
		self.map_colliders(|collider| collider.set_collision_groups(cg));
	}

	/// Sets the mass of this body
	pub fn set_mass(&mut self, mass: f32) {
		self.map_rigidbody_mut(|body| {
			let mut p = *body.mass_properties();
			p.inv_mass = 1.0 / mass;
			body.set_mass_properties(p, true);
		});
		self.inertia_stale = true;
	}

	/// Sets the linear damp of this body
	pub fn set_linear_damp(&mut self, damp: f32) {
		self.map_rigidbody_mut(|body| body.linear_damping = damp);
	}

	/// Sets the angular damp of this body
	pub fn set_angular_damp(&mut self, damp: f32) {
		self.map_rigidbody_mut(|body| body.angular_damping = damp);
	}

	/// Sets the gravity scale of this body. This wakes up the body.
	pub fn set_gravity_scale(&mut self, scale: f32) {
		self.map_rigidbody_mut(|body| body.set_gravity_scale(scale, true));
	}

	/// Sets the restitution of this body and it's colliders, if any.
	pub fn set_restitution(&mut self, restitution: f32) {
		self.restitution = restitution;
		self.map_colliders(|collider| collider.restitution = restitution);
	}

	/// Sets the friction of this body and it's colliders, if any.
	pub fn set_friction(&mut self, friction: f32) {
		self.friction = friction;
		self.map_colliders(|collider| collider.friction = friction);
	}

	/// Removes the given shape from this body and any colliders
	pub fn remove_shape(&mut self, shape: u32) -> Result<(), InvalidShape> {
		if (shape as usize) < self.shapes.len() {
			self.shapes.remove(shape as usize);
			if let Instance::Attached((_, colliders), space) = &mut self.body {
				if let Some(collider) = colliders.remove(shape as usize) {
					space
						.map_mut(|space| {
							space
								.remove_collider(collider)
								.expect("Invalid collider handle");
						})
						.expect("Invalid space handle");
				}
			}
			Ok(())
		} else {
			Err(InvalidShape)
		}
	}

	/// Updates the state of this rigidbody if it's stale
	pub fn refresh_state(&mut self, body: &mut RigidBody, shapes: &mut Indices<Shape>) {
		self.contacts.clear();
		if self.inertia_stale {
			self.recalculate_inertia(body, shapes);
		}
	}

	/// Sets the maximum amount of contacts this body should keep track of
	pub fn set_max_contacts(&mut self, count: u32) {
		if self.contacts.len() > count as usize {
			self.contacts.resize_with(count as usize, || unreachable!());
		}
		if (count == 0) != (self.max_contacts == 0) {
			self.map_rigidbody_mut(|body| {
				let mut ud = RigidBodyUserdata::try_from(&*body).unwrap();
				ud.set_monitoring(count != 0);
				body.user_data = ud.into();
			});
		}
		self.max_contacts = count;
	}

	/// Adds a contact with another body unless the `max_contacts` limit has been reached,
	/// in which case it's discarded.
	pub fn add_contact(&mut self, contact: ContactEvent) {
		if self.contacts.len() < self.max_contacts as usize {
			self.contacts.push(contact);
		}
	}

	/// Returns the amount of active contacts
	pub fn contact_count(&self) -> u32 {
		self.contacts.len() as u32
	}

	/// Returns the given contact if it exists
	pub fn get_contact(&self, index: u32) -> Option<&ContactEvent> {
		self.contacts.get(index as usize)
	}

	/// Returns whether Continuous Collision Detected (CCD) is enabled
	pub fn is_ccd_enabled(&self) -> bool {
		self.map_rigidbody(|body| body.is_ccd_active())
	}

	/// Enables or disables Continuous Collision Detected (CCD)
	pub fn enable_ccd(&mut self, enable: bool) {
		self.map_rigidbody_mut(|body| body.enable_ccd(enable));
	}
}

impl ContactEvent {
	pub fn new(
		position: Vector3,
		other: BodyIndex,
		other_shape: u32,
		self_shape: u32,
		normal: Vector3,
	) -> Self {
		Self {
			position,
			other_index: other,
			other_shape,
			self_shape,
			normal,
		}
	}

	pub fn index(&self) -> BodyIndex {
		self.other_index
	}

	pub fn object_id(&self) -> Option<ObjectID> {
		self.other_index
			.map(|body| body.object_id())
			.expect("Invalid body")
	}

	pub fn other_shape(&self) -> u32 {
		self.other_shape
	}

	pub fn self_shape(&self) -> u32 {
		self.self_shape
	}

	pub fn position(&self) -> Vector3 {
		self.position
	}

	pub fn local_position(&self, body: &RigidBody) -> Vector3 {
		let p = Point3::from(vec_gd_to_na(self.position));
		let p = body.position().inverse_transform_point(&p);
		vec_na_to_gd(p.coords)
	}

	pub fn local_normal(&self, body: &RigidBody) -> Vector3 {
		let p = vec_gd_to_na(self.normal);
		let p = body.position().inverse_transform_vector(&p);
		vec_na_to_gd(p)
	}
}

impl RigidBodyUserdata {
	const TYPE_MASK: u128 = 0x8000_0000_0000_0000;
	const MONITORING_MASK: u128 = 0x0001_0000_0000_0000;
	#[allow(dead_code)]
	const INDEX_MASK: u128 = 0x0000_ffff_ffff_ffff;

	/// Creates a new userdata intended for rigidbodies. This data is marked to ensure it can
	/// be identified as belonging to a [`Body`].
	///
	/// The data fits entirely within the lower 64 bits of a u128, so the upper 64 bits can be
	/// used for other purposes
	///
	/// The exact format from right to left is:
	/// - 32 bits for index
	/// - 16 bits for generation
	/// - 1 bit for monitoring
	/// - 14 bits reserved
	/// - 1 bit body type indicator (always `0` for [`Body`])
	fn new(index: BodyIndex, monitoring: bool) -> Self {
		let mut s = Self(0);
		s.set_index(index);
		s.set_monitoring(monitoring);
		s
	}

	/// Stores an index
	fn set_index(&mut self, index: BodyIndex) {
		self.0 &= !Self::INDEX_MASK;
		self.0 |= index.index() as u128 | (index.generation() as u128) << 32;
	}

	/// Sets whether this body is monitoring for contacts
	fn set_monitoring(&mut self, monitoring: bool) {
		self.0 = if monitoring {
			self.0 | Self::MONITORING_MASK
		} else {
			self.0 & !Self::MONITORING_MASK
		};
	}

	/// Returns the index of the body
	pub fn index(&self) -> BodyIndex {
		let i = self.0 as u32;
		let g = (self.0 >> 32) as u16;
		BodyIndex::new(i, g)
	}

	/// Returns whether the body is monitoring for contacts
	pub fn monitoring(&self) -> bool {
		self.0 & Self::MONITORING_MASK > 0
	}
}

impl TryFrom<u128> for RigidBodyUserdata {
	type Error = UserdataMismatch;

	fn try_from(n: u128) -> Result<Self, Self::Error> {
		if n & Self::TYPE_MASK == 0 {
			Ok(Self(n))
		} else {
			Err(UserdataMismatch)
		}
	}
}

impl TryFrom<&RigidBody> for RigidBodyUserdata {
	type Error = UserdataMismatch;

	fn try_from(body: &RigidBody) -> Result<Self, Self::Error> {
		body.user_data.try_into()
	}
}

impl From<RigidBodyUserdata> for indices::Index {
	fn from(ud: RigidBodyUserdata) -> Self {
		ud.index().into()
	}
}

impl From<RigidBodyUserdata> for u128 {
	fn from(ud: RigidBodyUserdata) -> Self {
		ud.0
	}
}

impl ColliderUserdata {
	const TYPE_MASK: u128 = 0x0000_0000_8000_0000_0000_0000;
	const MONITORING_MASK: u128 = 0x0000_0000_0001_0000_0000_0000;
	const RAY_PICKABLE_MASK: u128 = 0x0000_0000_0002_0000_0000_0000;
	#[allow(dead_code)]
	const INDEX_MASK: u128 = 0x0000_0000_0000_ffff_ffff_ffff;
	const SHAPE_INDEX_MASK: u128 = 0xffff_ffff_0000_0000_0000_0000;

	/// Creates a new userdata intended for [`Collider`]s. This data is marked to ensure it can
	/// be identified as belonging to a [`Body`].
	///
	/// The data fits entirely within the lower 96 bits of a u128, so the upper 32 bits can be
	/// used for other purposes
	///
	/// The exact format from right to left is:
	/// - 32 bits for index
	/// - 16 bits for generation
	/// - 1 bit for monitoring
	/// - 1 bit for ray pickable
	/// - 13 bits reserved
	/// - 1 bit body type indicator (always `0` for [`Body`])
	/// - 32 bits for shape index
	fn new(index: BodyIndex, monitoring: bool, ray_pickable: bool, shape: u32) -> Self {
		let mut s = Self(0);
		s.set_index(index);
		s.set_monitoring(monitoring);
		s.set_ray_pickable(ray_pickable);
		s.set_shape(shape);
		s
	}

	/// Stores an index
	fn set_index(&mut self, index: BodyIndex) {
		self.0 = self.0 | index.index() as u128 | (index.generation() as u128) << 32;
	}

	/// Sets whether this body is monitoring for contacts
	fn set_monitoring(&mut self, monitoring: bool) {
		self.0 = if monitoring {
			self.0 | Self::MONITORING_MASK
		} else {
			self.0 & !Self::MONITORING_MASK
		};
	}

	/// Sets whether this body can be picked up by ray casts
	fn set_ray_pickable(&mut self, pickable: bool) {
		self.0 = if pickable {
			self.0 | Self::RAY_PICKABLE_MASK
		} else {
			self.0 & !Self::RAY_PICKABLE_MASK
		};
	}

	/// Stores the index of the [`Collider`] in it's corresponding [`Body`]
	fn set_shape(&mut self, index: u32) {
		self.0 &= !Self::SHAPE_INDEX_MASK;
		self.0 |= (index as u128) << 64;
	}

	/// Returns the index of the body
	pub fn index(&self) -> BodyIndex {
		let i = self.0 as u32;
		let g = (self.0 >> 32) as u16;
		BodyIndex::new(i, g)
	}

	/// Returns whether the body is monitoring for contacts
	pub fn monitoring(&self) -> bool {
		self.0 & Self::MONITORING_MASK > 0
	}

	/// Returns whether this body can be picked up by ray casts
	pub fn ray_pickable(&self) -> bool {
		self.0 & Self::RAY_PICKABLE_MASK > 0
	}

	/// Returns the index of the [`Collider`] in it's corresponding [`Body`]
	pub fn shape(&self) -> u32 {
		(self.0 >> 64) as u32
	}
}

impl TryFrom<u128> for ColliderUserdata {
	type Error = UserdataMismatch;

	fn try_from(n: u128) -> Result<Self, Self::Error> {
		if n & Self::TYPE_MASK == 0 {
			Ok(Self(n))
		} else {
			Err(UserdataMismatch)
		}
	}
}

impl TryFrom<&Collider> for ColliderUserdata {
	type Error = UserdataMismatch;

	fn try_from(body: &Collider) -> Result<Self, Self::Error> {
		body.user_data.try_into()
	}
}

impl From<ColliderUserdata> for crate::indices::Index {
	fn from(ud: ColliderUserdata) -> Self {
		ud.index().into()
	}
}

impl From<ColliderUserdata> for u128 {
	fn from(ud: ColliderUserdata) -> Self {
		ud.0
	}
}
