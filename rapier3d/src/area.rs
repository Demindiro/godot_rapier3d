use crate::body::Body;
use crate::indices::{self, Indices};
use crate::server::{AreaIndex, BodyIndex, Instance, MapIndex, ObjectID, ShapeIndex, SpaceIndex};
use crate::util::*;
use core::convert::{TryFrom, TryInto};
use core::mem;
use gdnative::core_types::*;
use rapier3d::dynamics::{RigidBody, RigidBodyBuilder, RigidBodyHandle, RigidBodySet};
use rapier3d::geometry::{Collider, ColliderHandle, InteractionGroups};
use rapier3d::na::{Isometry3, Point3};

pub struct Area {
	index: Option<AreaIndex>,
	shapes: Vec<Shape>,
	instance: Instance<(RigidBodyHandle, Vec<Option<ColliderHandle>>), RigidBody>,
	scale: Vector3,
	linear_damp: f32,
	angular_damp: f32,
	priority: i32,

	gravity_direction: Vector3,
	gravity_force: f32,
	gravity_distance_scale: f32,
	gravity_is_point: bool,

	object_id: Option<ObjectID>,
	space_override_mode: SpaceOverrideMode,
	monitorable: bool,
	ray_pickable: bool,
	interaction_groups: InteractionGroups,

	intersecting_bodies: Vec<BodyIndex>,
	body_events: Vec<(BodyIndex, bool)>,
	area_events: Vec<(AreaIndex, bool)>,
}

pub struct Shape {
	shape: ShapeIndex,
	transform: Isometry3<f32>,
	scale: Vector3,
	enabled: bool,
}

#[derive(Clone, Copy)]
pub enum SpaceOverrideMode {
	Disabled,
	Combine,
	CombineReplace,
	Replace,
	ReplaceCombine,
}

#[derive(Clone)]
pub enum Gravity {
	Direction(DirectionGravity),
	Point(PointGravity),
}

#[derive(Clone)]
pub struct DirectionGravity {
	gravity: Vector3,
}

#[derive(Clone)]
pub struct PointGravity {
	point: Vector3,
	gravity: f32,
	distance_scale: f32,
}

pub struct RigidbodyUserdata(u128);
pub struct ColliderUserdata(u128);

#[derive(Debug)]
pub struct UserdataMismatch;

impl Area {
	/// Creates a new Area
	pub fn new() -> Self {
		Self {
			index: None,
			shapes: Vec::new(),
			instance: Instance::Loose(RigidBodyBuilder::new_static().build()),
			scale: Vector3::one(),
			linear_damp: 0.0,
			angular_damp: 0.0,
			priority: 0,

			gravity_is_point: false,
			gravity_force: 0.0,
			gravity_direction: Vector3::new(0.0, -1.0, 0.0),
			gravity_distance_scale: 0.0,

			object_id: None,
			space_override_mode: SpaceOverrideMode::Disabled,
			monitorable: false,
			ray_pickable: false,
			interaction_groups: InteractionGroups::default(),

			intersecting_bodies: Vec::new(),
			body_events: Vec::new(),
			area_events: Vec::new(),
		}
	}

	/// Calls the given function with a reference to this body's [`RigidBody`].
	fn map_rigidbody_mut<F, R>(&mut self, f: F) -> R
	where
		F: FnOnce(&mut RigidBody) -> R,
	{
		match &mut self.instance {
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
		if let Instance::Attached((_, ch), space) = &mut self.instance {
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

	/// Sets the index of the area
	///
	/// # Panics
	///
	/// Panics if the index is already set
	pub fn set_index(&mut self, index: AreaIndex) {
		assert_eq!(self.index, None);
		self.index = Some(index);
		match &mut self.instance {
			Instance::Attached(_, _) => todo!(),
			Instance::Loose(b) => Self::set_rigidbody_userdata(index, b, self.monitorable),
		}
	}

	/// Returns the index of the area
	///
	/// # Panics
	///
	/// Panics if the index isn't set
	pub fn index(&self) -> AreaIndex {
		self.index.unwrap()
	}

	/// Gets a shape at a given index
	pub fn get_shape(&self, index: u32) -> Option<&Shape> {
		self.shapes.get(index as usize)
	}

	/// Gets a shape at a given index mutably
	pub fn get_shape_mut(&mut self, index: u32) -> Option<&mut Shape> {
		self.shapes.get_mut(index as usize)
	}

	/// Adds a new shape at the end of the shapes list
	pub fn add_shape(&mut self, shape: ShapeIndex, transform: &Transform, enabled: bool) {
		let (pos, scale) = transform_to_isometry_and_scale(transform);
		self.shapes.push(Shape {
			shape,
			transform: pos,
			scale,
			enabled,
		});
	}

	/// Removes a shape from a position in the list
	///
	/// # Returns
	///
	/// - [`Shape`] if the shape existed
	/// - [`None`] if the shape didn't exist
	pub fn remove_shape(&mut self, index: u32) -> Option<Shape> {
		if (index as usize) < self.shapes.len() {
			Some(self.shapes.remove(index as usize))
		} else {
			None
		}
	}

	/// Removes all shapes from this area
	pub fn remove_all_shapes(&mut self) {
		self.shapes.clear();
	}

	/// Returns the [`Transform`] of this shape
	pub fn get_shape_transform(&self, shape: u32) -> Option<Transform> {
		self.get_shape(shape)
			.map(|shape| isometry_to_transform(&shape.transform))
	}

	/// Sets the [`Transform`] of this shape
	///
	/// # Returns
	///
	/// [`true`] if the shape existed, [`false`] otherwise
	pub fn set_shape_transform(&mut self, shape: u32, transform: &Transform) -> bool {
		self.get_shape_mut(shape)
			.map(|shape| shape.transform = transform_to_isometry(*transform))
			.is_some()
	}

	/// Sets whether this shape is enabled or not
	///
	/// # Returns
	///
	/// [`true`] if the shape existed, [`false`] otherwise
	pub fn set_shape_enabled(&mut self, shape: u32, enabled: bool) -> bool {
		self.get_shape_mut(shape)
			.map(|shape| shape.enabled = enabled)
			.is_some()
	}

	/// Returns the ['ShapeIndex`] of the given shape.
	pub fn get_shape_index(&self, shape: u32) -> Option<ShapeIndex> {
		self.get_shape(shape).map(|shape| shape.shape)
	}

	/// Sets the ['ShapeIndex`] of the given shape.
	///
	/// # Returns
	///
	/// [`true`] if the shape existed, [`false`] otherwise
	pub fn set_shape_index(&mut self, shape: u32, index: ShapeIndex) -> bool {
		self.get_shape_mut(shape)
			.map(|shape| shape.shape = index)
			.is_some()
	}

	/// Returns the space this area is attached to, if any
	pub fn space(&self) -> Option<SpaceIndex> {
		self.instance.as_attached().map(|v| v.1)
	}

	/// Sets the space of this area. This removes the area from it's current space, if any.
	pub fn set_space(&mut self, space: Option<SpaceIndex>) {
		// Judging by the API areas are intended to be moved around, so use kinematic
		let instance = Instance::Loose(RigidBodyBuilder::new_kinematic().build());
		let instance = mem::replace(&mut self.instance, instance);
		let body = match instance {
			Instance::Attached((body, _), space) => space
				.map_mut(|space| space.remove_body(body).expect("Invalid body handle"))
				.expect("Invalid space"),
			Instance::Loose(body) => body,
		};

		self.instance = if let Some(space) = space {
			let a = space
				.map_mut(|space| {
					let body = space.add_body(body);
					let mut colliders = Vec::with_capacity(self.shapes.len());
					for shape in self.shapes.iter() {
						if shape.enabled {
							shape
								.shape
								.map(|s| {
									let mut c = s.build(shape.transform, shape.scale, true);
									self.set_collider_userdata(&mut c);
									let c = space.add_collider(c, body);
									colliders.push(Some(c));
								})
								.expect("Invalid shape");
						} else {
							colliders.push(None);
						}
					}
					space.set_area_priority(body, self.priority);
					(body, colliders)
				})
				.expect("Invalid space");
			Instance::Attached(a, space)
		} else {
			Instance::Loose(body)
		}
	}

	/// Returns the gravity parameters of this area in global space,
	fn gravity(&self, area: &RigidBody) -> Gravity {
		if self.gravity_is_point {
			let point = vec_gd_to_na(self.gravity_direction.component_mul(self.scale));
			let point = Point3::from(point);
			let point = area.position() * point;
			Gravity::Point(PointGravity::new(
				self.gravity_force,
				vec_na_to_gd(point.coords),
				self.gravity_distance_scale,
			))
		} else {
			Gravity::Direction(DirectionGravity::new(
				self.gravity_direction * self.gravity_force,
			))
		}
	}

	/// Sets the gravity direction of this area
	pub fn set_gravity_direction(&mut self, direction: Vector3) {
		self.gravity_direction = direction;
	}

	/// Sets the gravity force of this area
	pub fn set_gravity_force(&mut self, force: f32) {
		self.gravity_force = force;
	}

	/// Sets the gravity distance scale of this area. Only applies if the gravity is point gravity
	pub fn set_gravity_distance_scale(&mut self, scale: f32) {
		self.gravity_distance_scale = scale;
	}

	/// Sets whether the gravity is point or direction gravity
	pub fn set_point_gravity(&mut self, enable: bool) {
		self.gravity_is_point = enable;
	}

	/// Sets the order in which this area will be processed compared to other areas
	pub fn set_priority(&mut self, priority: i32) {
		self.priority = priority;
		if let Instance::Attached((b, _), s) = &self.instance {
			s.map_mut(|s| s.set_area_priority(*b, priority))
				.expect("Invalid space handle");
		}
	}

	/// Sets the linear damp override of this area. A negative value disables it.
	pub fn set_linear_damp(&mut self, damp: f32) {
		self.linear_damp = damp;
	}

	/// Sets the angular damp override of this area. A negative value disables it.
	pub fn set_angular_damp(&mut self, damp: f32) {
		self.angular_damp = damp;
	}

	/// Returns the object ID attached to this area
	pub fn object_id(&self) -> Option<ObjectID> {
		self.object_id
	}

	/// Attach an object ID to this area
	pub fn set_object_id(&mut self, id: Option<ObjectID>) {
		self.object_id = id;
	}

	/// Returns the space override mode of this area
	pub fn space_override_mode(&self) -> SpaceOverrideMode {
		self.space_override_mode
	}

	/// Sets the space override mode of this area
	pub fn set_space_override_mode(&mut self, mode: SpaceOverrideMode) {
		self.space_override_mode = mode;
	}

	/// Returns whether this area can be monitored by other areas
	pub fn monitorable(&self) -> bool {
		self.monitorable
	}

	/// Sets whether this area can be monitored by other areas
	pub fn set_monitorable(&mut self, enable: bool) {
		self.monitorable = enable;
		self.map_rigidbody_mut(|body| {
			let mut ud = RigidbodyUserdata::try_from(body.user_data).expect("Invalid user data");
			ud.set_monitorable(enable);
			body.user_data = ud.into();
		});
	}

	/// Returns the transform of this area
	pub fn transform(&self) -> Transform {
		// TODO scaling
		match &self.instance {
			Instance::Attached((body, _), space) => space
				.map(|space| {
					let body = space.bodies().get(*body).expect("Invalid body handle");
					isometry_to_transform(body.position())
				})
				.expect("Invalid space"),
			Instance::Loose(body) => isometry_to_transform(body.position()),
		}
	}

	/// Sets the transform of this area
	pub fn set_transform(&mut self, transform: &Transform) {
		let (position, scale) = transform_to_isometry_and_scale(transform);
		self.scale = scale;
		// TODO apply scaling
		match &mut self.instance {
			Instance::Attached((body, _), space) => {
				space
					.map_mut(|space| {
						space
							.bodies_mut()
							.get_mut(*body)
							.expect("Invalid body handle")
							.set_position(position, false);
					})
					.expect("Invalid space");
			}
			Instance::Loose(body) => {
				body.set_position(position, false);
			}
		}
	}

	/// Returns whether this area can be detected by raycasts
	pub fn ray_pickable(&self) -> bool {
		self.ray_pickable
	}

	/// Sets whether this area can be detected by raycasts
	pub fn set_ray_pickable(&mut self, enable: bool) {
		self.ray_pickable = enable;
		self.map_colliders(|collider| {
			let mut ud =
				ColliderUserdata::try_from(collider.user_data).expect("Invalid collider userdata");
			ud.set_ray_pickable(enable);
			collider.user_data = ud.into();
		});
	}

	/// Sets the collision layer of this area
	pub fn set_layer(&mut self, layer: u32) {
		self.interaction_groups = self.interaction_groups.with_groups(layer);
	}

	/// Sets the collision masj of this area
	pub fn set_mask(&mut self, mask: u32) {
		self.interaction_groups = self.interaction_groups.with_mask(mask);
	}

	/// Stores the area's index in the given [`Collider`]
	fn set_collider_userdata(&self, collider: &mut Collider) {
		collider.user_data =
			ColliderUserdata::new(self.index(), self.monitorable, self.ray_pickable, u32::MAX)
				.into();
	}

	/// Stores the area's index in the given [`RigidBody`]
	fn set_rigidbody_userdata(index: AreaIndex, body: &mut RigidBody, monitorable: bool) {
		body.user_data = RigidbodyUserdata::new(index, monitorable).into();
	}

	/// Retrieves the area's index from a collider
	///
	/// # Returns
	///
	/// The [`AreaIndex`] if found, otherwise [`None`]
	pub fn get_collider_userdata(collider: &Collider) -> Option<ColliderUserdata> {
		ColliderUserdata::try_from(collider.user_data).ok()
	}

	/// Retrieves the area's index from a [`RigidBody`]
	///
	/// # Returns
	///
	/// The [`AreaIndex`] if found, otherwise [`None`]
	pub fn get_rigidbody_userdata(body: &RigidBody) -> Option<RigidbodyUserdata> {
		RigidbodyUserdata::try_from(body.user_data).ok()
	}

	/// Clears all intersection events. This is necessary in case nothing is popping events
	pub fn clear_events(&mut self) {
		self.body_events.clear();
		self.area_events.clear();
	}

	/// Adds an intersection event with a body
	pub fn push_body_event(&mut self, body: BodyIndex, intersecting: bool) {
		self.body_events.push((body, intersecting));
		if intersecting {
			self.intersecting_bodies.push(body);
		} else {
			let i = self
				.intersecting_bodies
				.iter()
				.position(|v| *v == body)
				.expect("Body index not found");
			self.intersecting_bodies.remove(i);
		}
	}

	/// Retrieves and removes an intersection event with a body
	pub fn pop_body_event(&mut self) -> Option<(BodyIndex, bool)> {
		self.body_events.pop()
	}

	/// Adds an intersection event with an area
	pub fn push_area_event(&mut self, area: AreaIndex, intersecting: bool) {
		self.area_events.push((area, intersecting));
	}

	/// Retrieves and removes an intersection event with an area
	pub fn pop_area_event(&mut self) -> Option<(AreaIndex, bool)> {
		self.area_events.pop()
	}

	/// Applies the space override (if any) to the bodies
	pub fn apply_events(
		&mut self,
		area: &RigidBody,
		bodies: &mut Indices<Body>,
		rigid_bodies: &RigidBodySet,
	) {
		let (replace, lock) = match self.space_override_mode {
			SpaceOverrideMode::Disabled => return,
			SpaceOverrideMode::Combine => (false, false),
			SpaceOverrideMode::Replace => (true, true),
			SpaceOverrideMode::CombineReplace => (false, true),
			SpaceOverrideMode::ReplaceCombine => (true, false),
		};
		let gravity = self.gravity(area);
		for body in self.intersecting_bodies.iter() {
			let body = bodies.get_mut(body.into()).expect("Invalid body index");
			body.area_apply_overrides(
				rigid_bodies,
				replace,
				&gravity,
				self.linear_damp,
				self.angular_damp,
				lock,
			);
		}
	}
}

impl DirectionGravity {
	fn new(gravity: Vector3) -> Self {
		Self { gravity }
	}

	pub fn gravity(&self) -> Vector3 {
		self.gravity
	}
}

impl PointGravity {
	fn new(gravity: f32, point: Vector3, distance_scale: f32) -> Self {
		Self {
			point,
			gravity,
			distance_scale,
		}
	}

	pub fn gravity(&self) -> f32 {
		self.gravity
	}

	pub fn point(&self) -> Vector3 {
		self.point
	}

	pub fn distance_scale(&self) -> f32 {
		self.distance_scale
	}
}

impl RigidbodyUserdata {
	const TYPE_MASK: u128 = 0x8000_0000_0000_0000;
	const MONITORABLE_MASK: u128 = 0x0001_0000_0000_0000;
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
	/// - 1 bit for monitorable
	/// - 14 bits reserved
	/// - 1 bit body type indicator (always `1` for [`Area`])
	fn new(index: AreaIndex, monitorable: bool) -> Self {
		let mut s = Self(Self::TYPE_MASK);
		s.set_index(index);
		s.set_monitorable(monitorable);
		s
	}

	/// Stores an index
	fn set_index(&mut self, index: AreaIndex) {
		self.0 &= !Self::INDEX_MASK;
		self.0 |= index.index() as u128 | (index.generation() as u128) << 32;
	}

	/// Sets whether this area can be monitored by other areas
	fn set_monitorable(&mut self, monitorable: bool) {
		self.0 = if monitorable {
			self.0 | Self::MONITORABLE_MASK
		} else {
			self.0 & !Self::MONITORABLE_MASK
		};
	}

	/// Returns the index of the area
	pub fn index(&self) -> AreaIndex {
		let i = self.0 as u32;
		let g = (self.0 >> 32) as u16;
		AreaIndex::new(i, g)
	}

	/// Returns whether this area can be monitored ny other areas
	pub fn monitorable(&self) -> bool {
		self.0 & Self::MONITORABLE_MASK > 0
	}
}

impl TryFrom<u128> for RigidbodyUserdata {
	type Error = UserdataMismatch;

	fn try_from(n: u128) -> Result<Self, Self::Error> {
		if n & Self::TYPE_MASK > 0 {
			Ok(Self(n))
		} else {
			Err(UserdataMismatch)
		}
	}
}

impl TryFrom<&RigidBody> for RigidbodyUserdata {
	type Error = UserdataMismatch;

	fn try_from(body: &RigidBody) -> Result<Self, Self::Error> {
		body.user_data.try_into()
	}
}

impl From<RigidbodyUserdata> for indices::Index {
	fn from(ud: RigidbodyUserdata) -> Self {
		ud.index().into()
	}
}

impl From<RigidbodyUserdata> for u128 {
	fn from(ud: RigidbodyUserdata) -> Self {
		ud.0
	}
}

impl ColliderUserdata {
	const TYPE_MASK: u128 = 0x0000_0000_8000_0000_0000_0000;
	const MONITORABLE_MASK: u128 = 0x0000_0000_0001_0000_0000_0000;
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
	/// - 1 bit for monitorable
	/// - 1 bit for ray pickable
	/// - 13 bits reserved
	/// - 1 bit body type indicator (always `1` for [`Area`])
	/// - 32 bits for shape index
	fn new(index: AreaIndex, monitorable: bool, ray_pickable: bool, shape: u32) -> Self {
		let mut s = Self(Self::TYPE_MASK);
		s.set_index(index);
		s.set_monitorable(monitorable);
		s.set_ray_pickable(ray_pickable);
		s.set_shape(shape);
		s
	}

	/// Stores an index
	fn set_index(&mut self, index: AreaIndex) {
		self.0 = self.0 | index.index() as u128 | (index.generation() as u128) << 32;
	}

	/// Sets whether this area can be monitored by other areas
	fn set_monitorable(&mut self, monitorable: bool) {
		self.0 = if monitorable {
			self.0 | Self::MONITORABLE_MASK
		} else {
			self.0 & !Self::MONITORABLE_MASK
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
	pub fn index(&self) -> AreaIndex {
		let i = self.0 as u32;
		let g = (self.0 >> 32) as u16;
		AreaIndex::new(i, g)
	}

	/// Returns whether this area can be monitored by other areas
	pub fn monitorable(&self) -> bool {
		self.0 & Self::MONITORABLE_MASK > 0
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
		if n & Self::TYPE_MASK > 0 {
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

impl From<ColliderUserdata> for indices::Index {
	fn from(ud: ColliderUserdata) -> Self {
		ud.index().into()
	}
}

impl From<ColliderUserdata> for u128 {
	fn from(ud: ColliderUserdata) -> Self {
		ud.0
	}
}
