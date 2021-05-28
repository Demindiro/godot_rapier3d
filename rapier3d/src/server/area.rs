use super::*;
use crate::area::*;
use gdnative::core_types::*;
use gdnative::godot_error;

#[derive(Debug)]
enum ParamError {
	InvalidParam,
	InvalidData,
}

#[derive(Debug)]
enum Param {
	Gravity(f32),
	GravityVector(Vector3),
	GravityIsPoint(bool),
	GravityDistanceScale(f32),
	LinearDamp(f32),
	AngularDamp(f32),
	Priority(i32),
}

impl Param {
	fn new(n: i32, data: &Variant) -> Result<Self, ParamError> {
		match n {
			0 => data.try_to_f64().map(|g| Self::Gravity(g as f32)),
			1 => data.try_to_vector3().map(Self::GravityVector),
			2 => data.try_to_bool().map(Self::GravityIsPoint),
			3 => data
				.try_to_f64()
				.map(|s| Self::GravityDistanceScale(s as f32)),
			4 => None, // Superseded by GravityDistanceScale,
			5 => data.try_to_f64().map(|d| Self::LinearDamp(d as f32)),
			6 => data.try_to_f64().map(|d| Self::AngularDamp(d as f32)),
			7 => data
				.try_to_i64()
				.map(|p| Self::Priority(p as i32))
				.or_else(|| data.try_to_f64().map(|p| Self::Priority(p as i32))),
			_ => return Err(ParamError::InvalidParam),
		}
		.ok_or(ParamError::InvalidData)
	}
}

pub fn init(ffi: &mut ffi::FFI) {
	ffi!(ffi, area_add_shape, add_shape);
	ffi!(
		ffi,
		area_attach_object_instance_id,
		attach_object_instance_id
	);
	ffi!(ffi, area_clear_shapes, clear_shapes);
	ffi!(ffi, area_create, create);
	ffi!(ffi, area_get_area_event, get_area_event);
	ffi!(ffi, area_get_body_event, get_body_event);
	ffi!(ffi, area_get_object_instance_id, get_object_instance_id);
	ffi!(ffi, area_get_shape, get_shape);
	ffi!(ffi, area_get_shape_transform, get_shape_transform);
	ffi!(ffi, area_get_space, get_space);
	ffi!(ffi, area_get_space_override_mode, get_space_override_mode);
	ffi!(ffi, area_get_transform, get_transform);
	ffi!(ffi, area_is_ray_pickable, is_ray_pickable);
	ffi!(ffi, area_remove_shape, remove_shape);
	ffi!(ffi, area_set_collision_layer, set_collision_layer);
	ffi!(ffi, area_set_collision_mask, set_collision_mask);
	ffi!(ffi, area_set_monitorable, set_monitorable);
	ffi!(ffi, area_set_param, set_param);
	ffi!(ffi, area_set_ray_pickable, set_ray_pickable);
	ffi!(ffi, area_set_shape, set_shape);
	ffi!(ffi, area_set_shape_disabled, set_shape_disabled);
	ffi!(ffi, area_set_shape_transform, set_shape_transform);
	ffi!(ffi, area_set_space, set_space);
	ffi!(ffi, area_set_space_override_mode, set_space_override_mode);
	ffi!(ffi, area_set_transform, set_transform);
}

pub fn free(_area: Area) {
	godot_error!("TODO");
}

fn create() -> Option<Index> {
	let index = AreaIndex::add(Area::new());
	index.map_mut(|area| area.set_index(index)).unwrap();
	Some(Index::Area(index))
}

fn add_shape(area: Index, shape: Index, transform: &Transform, disable: bool) {
	if let Some(shape) = shape.as_shape() {
		map_or_err!(area, map_area_mut, |area, _| area
			.add_shape(shape, transform, !disable));
	} else {
		godot_error!("Index does not point to a shape");
	}
}

fn remove_shape(area: Index, shape: i32) {
	map_or_err!(area, map_area_mut, |area, _| {
		if area.remove_shape(shape as u32).is_none() {
			godot_error!("Invalid shape index");
		}
	});
}

fn get_shape(area: Index, shape: i32) -> Option<Index> {
	map_or_err!(area, map_area, |area, _| {
		if let Some(shape) = area.get_shape_index(shape as u32) {
			Some(Index::Shape(shape))
		} else {
			godot_error!("Invalid shape index");
			None
		}
	})
	.unwrap_or(None)
}

fn set_shape(area: Index, shape: i32, index: Index) {
	if let Index::Shape(index) = index {
		map_or_err!(area, map_area_mut, |area, _| {
			if !area.set_shape_index(shape as u32, index) {
				godot_error!("Invalid shape index");
			}
		});
	} else {
		godot_error!("Index does not point to a shape");
	}
}

fn clear_shapes(area: Index) {
	map_or_err!(area, map_area_mut, |area, _| area.remove_all_shapes());
}

// FIXME handle to_sys() stuff in the generated ffi wrapper
fn get_shape_transform(area: Index, shape: i32) -> gdnative::sys::godot_transform {
	let transform = map_or_err!(area, map_area, |area, _| {
		if let Some(trf) = area.get_shape_transform(shape as u32) {
			trf
		} else {
			godot_error!("Invalid shape index");
			Transform {
				basis: Basis::identity(),
				origin: Vector3::zero(),
			}
		}
	});
	let transform = transform.unwrap_or(Transform {
		basis: Basis::identity(),
		origin: Vector3::zero(),
	});
	// SAFETY: transform is guaranteed valid
	unsafe { *transform.sys() }
}

fn set_shape_transform(area: Index, shape: i32, transform: &Transform) {
	map_or_err!(area, map_area_mut, |area, _| {
		if !area.set_shape_transform(shape as u32, transform) {
			godot_error!("Invalid shape index");
		}
	});
}

fn set_shape_disabled(area: Index, shape: i32, disabled: bool) {
	map_or_err!(area, map_area_mut, |area, _| {
		if !area.set_shape_enabled(shape as u32, !disabled) {
			godot_error!("Invalid shape index");
		}
	});
}

fn get_space(area: Index) -> Option<Index> {
	map_or_err!(area, map_area, |area, _| { area.space().map(Index::Space) }).unwrap_or(None)
}

fn set_space(area: Index, space: Option<Index>) {
	let space = if let Some(Some(space)) = space.map(|i| i.as_space()) {
		Some(space)
	} else if space.is_some() {
		godot_error!("Index does not point to a space");
		return;
	} else {
		None
	};
	map_or_err!(area, map_area_mut, |area, _| {
		area.set_space(space);
	});
}

fn set_param(area: Index, param: i32, data: &Variant) {
	let param = match Param::new(param, data) {
		Ok(p) => p,
		Err(e) => {
			godot_error!("Failed to set area param: {:?}", e);
			return;
		}
	};

	let result = area.map_area_mut(|area, _| match param {
		Param::Priority(p) => area.set_priority(p),
		Param::LinearDamp(d) => area.set_linear_damp(d),
		Param::AngularDamp(d) => area.set_angular_damp(d),
		Param::GravityIsPoint(is) => area.set_point_gravity(is),
		Param::Gravity(g) => area.set_gravity_force(g),
		Param::GravityVector(g) => area.set_gravity_direction(g),
		Param::GravityDistanceScale(s) => area.set_gravity_distance_scale(s),
	});

	match result {
		Err(IndexError::WrongType) => {
			// Each Space has an imaginary Area
			let fix_g = |g| {
				if g == Vector3::zero() {
					Vector3::new(0.0, -1.0, 0.0)
				} else {
					g
				}
			};
			let result = area.map_space_mut(|space, _| match param {
				Param::LinearDamp(d) => space.set_default_linear_damp(d),
				Param::AngularDamp(d) => space.set_default_angular_damp(d),
				Param::Gravity(g) => space.set_gravity(fix_g(space.gravity()).normalize() * g),
				Param::GravityVector(g) => space.set_gravity(g * fix_g(space.gravity()).length()),
				_ => godot_error!("Parameter {:?} is not supported for spaces", param),
			});
			if let Err(e) = result {
				match e {
					IndexError::WrongType => {
						godot_error!("Index does not point to an area or space")
					}
					IndexError::NoElement => godot_error!("No space at index"),
				}
			}
		}
		Err(IndexError::NoElement) => godot_error!("No area at index {:?}", area),
		Ok(()) => (),
	}
}

fn attach_object_instance_id(area: Index, id: i32) {
	map_or_err!(area, map_area_mut, |area, _| area
		.set_object_id(ObjectID::new(id as u32)));
}

fn get_object_instance_id(area: Index) -> i32 {
	map_or_err!(area, map_area_mut, |area, _| area
		.object_id()
		.map(|id| id.get()))
	.flatten()
	.unwrap_or(0) as i32
}

fn set_monitorable(area: Index, monitorable: bool) {
	map_or_err!(area, map_area_mut, |area, _| area
		.set_monitorable(monitorable));
}

fn get_space_override_mode(area: Index) -> i32 {
	map_or_err!(area, map_area, |area, _| match area.space_override_mode() {
		SpaceOverrideMode::Disabled => 0,
		SpaceOverrideMode::Combine => 1,
		SpaceOverrideMode::CombineReplace => 2,
		SpaceOverrideMode::Replace => 3,
		SpaceOverrideMode::ReplaceCombine => 4,
	})
	.unwrap_or(0)
}

fn set_space_override_mode(area: Index, mode: i32) {
	let mode = match mode {
		0 => SpaceOverrideMode::Disabled,
		1 => SpaceOverrideMode::Combine,
		2 => SpaceOverrideMode::CombineReplace,
		3 => SpaceOverrideMode::Replace,
		4 => SpaceOverrideMode::ReplaceCombine,
		_ => {
			godot_error!("Invalid mode");
			return;
		}
	};
	map_or_err!(area, map_area_mut, |area, _| area
		.set_space_override_mode(mode));
}

// FIXME fix build script to generate proper return types
fn get_transform(area: Index) -> gdnative::sys::godot_transform {
	let t = map_or_err!(area, map_area, |area, _| area.transform()).unwrap_or(Transform {
		basis: Basis::identity(),
		origin: Vector3::zero(),
	});
	// SAFETY: t is guaranteed valid
	unsafe { *t.sys() }
}

fn set_transform(area: Index, transform: &Transform) {
	map_or_err!(area, map_area_mut, |area, _| area.set_transform(transform));
}

fn is_ray_pickable(area: Index) -> bool {
	map_or_err!(area, map_area, |area, _| area.ray_pickable()).unwrap_or(false)
}

fn set_ray_pickable(area: Index, enable: bool) {
	map_or_err!(area, map_area_mut, |area, _| area.set_ray_pickable(enable));
}

fn set_collision_mask(area: Index, mask: u32) {
	map_or_err!(area, map_area_mut, |area, _| area.set_mask(mask));
}

fn set_collision_layer(area: Index, layer: u32) {
	map_or_err!(area, map_area_mut, |area, _| area.set_layer(layer));
}

fn get_area_event(area: Index, event: &mut ffi::PhysicsAreaMonitorEvent) -> bool {
	map_or_err!(area, map_area_mut, |area, _| {
		if let Some((body, intersect)) = area.pop_area_event() {
			body.map(|body| event.set_object_id(body.object_id()))
				.expect("Invalid body index");
			event.set_index(Index::Area(body));
			event.set_added(intersect);
			true
		} else {
			false
		}
	})
	.unwrap_or(false)
}

fn get_body_event(area: Index, event: &mut ffi::PhysicsAreaMonitorEvent) -> bool {
	map_or_err!(area, map_area_mut, |area, _| {
		if let Some((body, intersect)) = area.pop_body_event() {
			body.map(|body| event.set_object_id(body.object_id()))
				.expect("Invalid body index");
			event.set_index(Index::Body(body));
			event.set_added(intersect);
			true
		} else {
			false
		}
	})
	.unwrap_or(false)
}
