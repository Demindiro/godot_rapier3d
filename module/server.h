#ifndef PLUGGABLE_PHYSICS_SERVER_H
#define PLUGGABLE_PHYSICS_SERVER_H

#include "typedef.h"

#include "api.gen.h"
#include "index.h"
#include "core/rid.h"
#include "core/hash_map.h"
#include "core/resource.h"
#include "core/vector.h"
#include "servers/physics_server.h"
#include "body_state.h"
#include "space_state.h"
// TODO ditto
#include "gdnative.h"
//#include "gdnative/gdnative.h"


#define EXEC_FFI_FN(_server_, _fn_, ...) do { \
	ERR_FAIL_COND_MSG((_server_)->fn_table._fn_ == nullptr, "Not implemented"); \
	(*(_server_)->fn_table._fn_)(__VA_ARGS__); \
} while (0)


#define EXEC_V_FFI_FN(_v_, _server_, _fn_, ...) do { \
	ERR_FAIL_COND_V_MSG((_server_)->fn_table._fn_ == nullptr, _v_, "Not implemented"); \
	(*(_server_)->fn_table._fn_)(__VA_ARGS__); \
} while (0)


class PluggablePhysicsRID_Data : public RID_Data {
	friend class PluggablePhysicsServer;
	index_t index;
};


class PluggablePhysicsServer : public PhysicsServer {
	GDCLASS(PluggablePhysicsServer, PhysicsServer);

	struct Callback {
		ObjectID object_id;
		StringName method;
		Variant userdata;

		// HashMap needs this for whatever reason
		Callback() {
			this->object_id = 0;
			this->method = "";
			this->userdata = Variant();
		}

		Callback(Object *p_object, StringName p_method, Variant p_userdata) {
			this->object_id = p_object != nullptr ? p_object->get_instance_id() : 0;
			this->method = p_method;
			this->userdata = p_userdata;
		}
	};

	struct AreaCallback {
		ObjectID object_id;
		StringName method;

		// HashMap needs this for whatever reason
		AreaCallback() {
			this->object_id = 0;
			this->method = "";
		}

		AreaCallback(Object *p_object, StringName p_method) {
			this->object_id = p_object != nullptr ? p_object->get_instance_id() : 0;
			this->method = p_method;
		}
	};

	PluggablePhysicsDirectBodyState *body_state_singleton;
	PluggablePhysicsDirectSpaceState *space_state_singleton;

	struct fn_table fn_table;
	Ref<GDNative> library;

	mutable RID_Owner<PluggablePhysicsRID_Data> rids;
	HashMap<index_t, RID> reverse_rids;
	HashMap<index_t, Callback> body_force_integration_callbacks;
	HashMap<index_t, AreaCallback> area_body_monitor_callbacks;
	HashMap<index_t, AreaCallback> area_area_monitor_callbacks;

	friend class PluggablePhysicsDirectBodyState;
	friend class PluggablePhysicsDirectSpaceState;

	_FORCE_INLINE_ RID make_rid(index_t index) {
		PluggablePhysicsRID_Data *data = memnew(PluggablePhysicsRID_Data);
		data->index = index;
		RID rid = this->rids.make_rid(data);
		this->reverse_rids.set(index, rid);
		return rid;
	}

	_FORCE_INLINE_ index_t get_index(RID rid) const {
		if (rid.is_valid()) {
			PluggablePhysicsRID_Data *data = this->rids.get(rid);
			return data != nullptr ? data->index : 0;
		}
		return 0;
	}

	_FORCE_INLINE_ RID get_rid(index_t index) const {
		return index != 0 ? this->reverse_rids.get(index) : RID();
	}

protected:
	static void _bind_methods();

public:
	PluggablePhysicsServer();
	~PluggablePhysicsServer();

	virtual RID area_create();
	virtual void area_set_space(RID p_area, RID p_space);
	virtual RID area_get_space(RID p_area) const;
	virtual void area_set_space_override_mode(RID p_area, AreaSpaceOverrideMode p_mode);
	virtual AreaSpaceOverrideMode area_get_space_override_mode(RID p_area) const;
	virtual void area_add_shape(RID p_area, RID p_shape, const Transform &p_transform = Transform(), bool p_disabled = false);
	virtual void area_set_shape(RID p_area, int p_shape_idx, RID p_shape);
	virtual void area_set_shape_transform(RID p_area, int p_shape_idx, const Transform &p_transform);
	virtual int area_get_shape_count(RID p_area) const;
	virtual RID area_get_shape(RID p_area, int p_shape_idx) const;
	virtual Transform area_get_shape_transform(RID p_area, int p_shape_idx) const;
	virtual void area_remove_shape(RID p_area, int p_shape_idx);
	virtual void area_clear_shapes(RID p_area);
	virtual void area_set_shape_disabled(RID p_area, int p_shape_idx, bool p_disabled);
	virtual void area_attach_object_instance_id(RID p_area, ObjectID p_id);
	virtual ObjectID area_get_object_instance_id(RID p_area) const;
	virtual void area_set_param(RID p_area, AreaParameter p_param, const Variant &p_value);
	virtual Variant area_get_param(RID p_area, AreaParameter p_param) const;
	virtual void area_set_transform(RID p_area, const Transform &p_transform);
	virtual Transform area_get_transform(RID p_area) const;
	virtual void area_set_collision_mask(RID p_area, uint32_t p_mask);
	virtual void area_set_collision_layer(RID p_area, uint32_t p_layer);
	virtual void area_set_monitorable(RID p_area, bool p_monitorable);
	virtual void area_set_monitor_callback(RID p_area, Object *p_receiver, const StringName &p_method);
	virtual void area_set_area_monitor_callback(RID p_area, Object *p_receiver, const StringName &p_method);
	virtual void area_set_ray_pickable(RID p_area, bool p_enable);
	virtual bool area_is_ray_pickable(RID p_area) const;

	virtual RID body_create(BodyMode p_mode = BODY_MODE_RIGID, bool p_init_sleeping = false);
	virtual void body_set_space(RID p_body, RID p_space);
	virtual RID body_get_space(RID p_body) const;
	virtual void body_set_mode(RID p_body, BodyMode p_mode);
	virtual BodyMode body_get_mode(RID p_body) const;
	virtual void body_add_shape(RID p_body, RID p_shape, const Transform &p_transform = Transform(), bool p_disabled = false);
	virtual void body_set_shape(RID p_body, int p_shape_idx, RID p_shape);
	virtual void body_set_shape_transform(RID p_body, int p_shape_idx, const Transform &p_transform);
	virtual int body_get_shape_count(RID p_body) const;
	virtual RID body_get_shape(RID p_body, int p_shape_idx) const;
	virtual Transform body_get_shape_transform(RID p_body, int p_shape_idx) const;
	virtual void body_set_shape_disabled(RID p_body, int p_shape_idx, bool p_disabled);
	virtual void body_remove_shape(RID p_body, int p_shape_idx);
	virtual void body_clear_shapes(RID p_body);
	virtual void body_attach_object_instance_id(RID p_body, uint32_t p_id);
	virtual uint32_t body_get_object_instance_id(RID p_body) const;
	virtual void body_set_enable_continuous_collision_detection(RID p_body, bool p_enable);
	virtual bool body_is_continuous_collision_detection_enabled(RID p_body) const;
	virtual void body_set_collision_layer(RID p_body, uint32_t p_layer);
	virtual uint32_t body_get_collision_layer(RID p_body) const;
	virtual void body_set_collision_mask(RID p_body, uint32_t p_mask);
	virtual uint32_t body_get_collision_mask(RID p_body) const;
	virtual void body_set_user_flags(RID p_body, uint32_t p_flags);
	virtual uint32_t body_get_user_flags(RID p_body) const;
	virtual void body_set_param(RID p_body, BodyParameter p_param, float p_value);
	virtual float body_get_param(RID p_body, BodyParameter p_param) const;
	virtual void body_set_kinematic_safe_margin(RID p_body, real_t p_margin);
	virtual real_t body_get_kinematic_safe_margin(RID p_body) const;
	virtual void body_set_state(RID p_body, BodyState p_state, const Variant &p_variant);
	virtual Variant body_get_state(RID p_body, BodyState p_state) const;
	virtual void body_set_applied_force(RID p_body, const Vector3 &p_force);
	virtual Vector3 body_get_applied_force(RID p_body) const;
	virtual void body_set_applied_torque(RID p_body, const Vector3 &p_torque);
	virtual Vector3 body_get_applied_torque(RID p_body) const;
	virtual void body_add_central_force(RID p_body, const Vector3 &p_force);
	virtual void body_add_force(RID p_body, const Vector3 &p_force, const Vector3 &p_pos);
	virtual void body_add_torque(RID p_body, const Vector3 &p_torque);
	virtual void body_apply_central_impulse(RID p_body, const Vector3 &p_impulse);
	virtual void body_apply_impulse(RID p_body, const Vector3 &p_pos, const Vector3 &p_impulse);
	virtual void body_apply_torque_impulse(RID p_body, const Vector3 &p_impulse);
	virtual void body_set_axis_velocity(RID p_body, const Vector3 &p_axis_velocity);
	virtual void body_set_axis_lock(RID p_body, BodyAxis p_axis, bool p_lock);
	virtual bool body_is_axis_locked(RID p_body, BodyAxis p_axis) const;
	virtual void body_add_collision_exception(RID p_body, RID p_body_b);
	virtual void body_remove_collision_exception(RID p_body, RID p_body_b);
	virtual void body_get_collision_exceptions(RID p_body, List<RID> *p_exceptions);
	virtual void body_set_max_contacts_reported(RID p_body, int p_contacts);
	virtual int body_get_max_contacts_reported(RID p_body) const;
	virtual void body_set_contacts_reported_depth_threshold(RID p_body, float p_threshold);
	virtual float body_get_contacts_reported_depth_threshold(RID p_body) const;
	virtual void body_set_omit_force_integration(RID p_body, bool p_omit);
	virtual bool body_is_omitting_force_integration(RID p_body) const;
	virtual void body_set_force_integration_callback(RID p_body, Object *p_receiver, const StringName &p_method, const Variant &p_udata = Variant());
	virtual void body_set_ray_pickable(RID p_body, bool p_enable);
	virtual bool body_is_ray_pickable(RID p_body) const;
	virtual PhysicsDirectBodyState *body_get_direct_state(RID p_body);

	virtual bool body_test_motion(RID p_body, const Transform &p_from, const Vector3 &p_motion, bool p_infinite_inertia, MotionResult *r_result = NULL, bool p_exclude_raycast_shapes = true);
	virtual int body_test_ray_separation(RID p_body, const Transform &p_transform, bool p_infinite_inertia, Vector3 &r_recover_motion, SeparationResult *r_results, int p_result_max, float p_margin = 0.001);

	virtual RID soft_body_create(bool p_init_sleeping = false);
	virtual void soft_body_update_visual_server(RID p_body, class SoftBodyVisualServerHandler *p_visual_server_handler);
	virtual void soft_body_set_space(RID p_body, RID p_space);
	virtual RID soft_body_get_space(RID p_body) const;
	virtual void soft_body_set_mesh(RID p_body, const REF &p_mesh);
	virtual void soft_body_set_collision_layer(RID p_body, uint32_t p_layer);
	virtual uint32_t soft_body_get_collision_layer(RID p_body) const;
	virtual void soft_body_set_collision_mask(RID p_body, uint32_t p_mask);
	virtual uint32_t soft_body_get_collision_mask(RID p_body) const;
	virtual void soft_body_add_collision_exception(RID p_body, RID p_body_b);
	virtual void soft_body_remove_collision_exception(RID p_body, RID p_body_b);
	virtual void soft_body_get_collision_exceptions(RID p_body, List<RID> *p_exceptions);
	virtual void soft_body_set_state(RID p_body, BodyState p_state, const Variant &p_variant);
	virtual Variant soft_body_get_state(RID p_body, BodyState p_state) const;
	virtual void soft_body_set_transform(RID p_body, const Transform &p_transform);
	virtual Vector3 soft_body_get_vertex_position(RID p_body, int vertex_index) const;
	virtual void soft_body_set_ray_pickable(RID p_body, bool p_enable);
	virtual bool soft_body_is_ray_pickable(RID p_body) const;
	virtual void soft_body_set_simulation_precision(RID p_body, int p_simulation_precision);
	virtual int soft_body_get_simulation_precision(RID p_body);
	virtual void soft_body_set_total_mass(RID p_body, real_t p_total_mass);
	virtual real_t soft_body_get_total_mass(RID p_body);
	virtual void soft_body_set_linear_stiffness(RID p_body, real_t p_stiffness);
	virtual real_t soft_body_get_linear_stiffness(RID p_body);
	virtual void soft_body_set_areaAngular_stiffness(RID p_body, real_t p_stiffness);
	virtual real_t soft_body_get_areaAngular_stiffness(RID p_body);
	virtual void soft_body_set_volume_stiffness(RID p_body, real_t p_stiffness);
	virtual real_t soft_body_get_volume_stiffness(RID p_body);
	virtual void soft_body_set_pressure_coefficient(RID p_body, real_t p_pressure_coefficient);
	virtual real_t soft_body_get_pressure_coefficient(RID p_body);
	virtual void soft_body_set_pose_matching_coefficient(RID p_body, real_t p_pose_matching_coefficient);
	virtual real_t soft_body_get_pose_matching_coefficient(RID p_body);
	virtual void soft_body_set_damping_coefficient(RID p_body, real_t p_damping_coefficient);
	virtual real_t soft_body_get_damping_coefficient(RID p_body);
	virtual void soft_body_set_drag_coefficient(RID p_body, real_t p_drag_coefficient);
	virtual real_t soft_body_get_drag_coefficient(RID p_body);
	virtual void soft_body_move_point(RID p_body, int p_point_index, const Vector3 &p_global_position);
	virtual Vector3 soft_body_get_point_global_position(RID p_body, int p_point_index);
	virtual Vector3 soft_body_get_point_offset(RID p_body, int p_point_index) const;
	virtual void soft_body_remove_all_pinned_points(RID p_body);
	virtual void soft_body_pin_point(RID p_body, int p_point_index, bool p_pin);
	virtual bool soft_body_is_point_pinned(RID p_body, int p_point_index);

	virtual JointType joint_get_type(RID p_joint) const;
	virtual void joint_set_solver_priority(RID p_joint, int p_priority);
	virtual int joint_get_solver_priority(RID p_joint) const;
	virtual void joint_disable_collisions_between_bodies(RID p_joint, const bool p_disable);
	virtual bool joint_is_disabled_collisions_between_bodies(RID p_joint) const;
	virtual RID joint_create_pin(RID p_body_A, const Vector3 &p_local_A, RID p_body_B, const Vector3 &p_local_B);
	virtual void pin_joint_set_param(RID p_joint, PinJointParam p_param, float p_value);
	virtual float pin_joint_get_param(RID p_joint, PinJointParam p_param) const;
	virtual void pin_joint_set_local_a(RID p_joint, const Vector3 &p_A);
	virtual Vector3 pin_joint_get_local_a(RID p_joint) const;
	virtual void pin_joint_set_local_b(RID p_joint, const Vector3 &p_B);
	virtual Vector3 pin_joint_get_local_b(RID p_joint) const;
	virtual RID joint_create_hinge(RID p_body_A, const Transform &p_hinge_A, RID p_body_B, const Transform &p_hinge_B);
	virtual RID joint_create_hinge_simple(RID p_body_A, const Vector3 &p_pivot_A, const Vector3 &p_axis_A, RID p_body_B, const Vector3 &p_pivot_B, const Vector3 &p_axis_B);
	virtual void hinge_joint_set_param(RID p_joint, HingeJointParam p_param, float p_value);
	virtual float hinge_joint_get_param(RID p_joint, HingeJointParam p_param) const;
	virtual void hinge_joint_set_flag(RID p_joint, HingeJointFlag p_flag, bool p_value);
	virtual bool hinge_joint_get_flag(RID p_joint, HingeJointFlag p_flag) const;
	virtual RID joint_create_slider(RID p_body_A, const Transform &p_local_frame_A, RID p_body_B, const Transform &p_local_frame_B);
	virtual void slider_joint_set_param(RID p_joint, SliderJointParam p_param, float p_value);
	virtual float slider_joint_get_param(RID p_joint, SliderJointParam p_param) const;
	virtual RID joint_create_cone_twist(RID p_body_A, const Transform &p_local_frame_A, RID p_body_B, const Transform &p_local_frame_B);
	virtual void cone_twist_joint_set_param(RID p_joint, ConeTwistJointParam p_param, float p_value);
	virtual float cone_twist_joint_get_param(RID p_joint, ConeTwistJointParam p_param) const;
	virtual RID joint_create_generic_6dof(RID p_body_A, const Transform &p_local_frame_A, RID p_body_B, const Transform &p_local_frame_B);
	virtual void generic_6dof_joint_set_param(RID p_joint, Vector3::Axis p_axis, G6DOFJointAxisParam p_param, float p_value);
	virtual float generic_6dof_joint_get_param(RID p_joint, Vector3::Axis p_axis, G6DOFJointAxisParam p_param);
	virtual void generic_6dof_joint_set_flag(RID p_joint, Vector3::Axis p_axis, G6DOFJointAxisFlag p_flag, bool p_enable);
	virtual bool generic_6dof_joint_get_flag(RID p_joint, Vector3::Axis p_axis, G6DOFJointAxisFlag p_flag);
	virtual void generic_6dof_joint_set_precision(RID p_joint, int precision);
	virtual int generic_6dof_joint_get_precision(RID p_joint);

	virtual RID shape_create(ShapeType p_shape);
	virtual void shape_set_data(RID p_shape, const Variant &p_data);
	virtual ShapeType shape_get_type(RID p_shape) const;
	virtual Variant shape_get_data(RID p_shape) const;
	virtual void shape_set_margin(RID p_shape, real_t p_margin);
	virtual real_t shape_get_margin(RID p_shape) const;
	virtual void shape_set_custom_solver_bias(RID p_shape, real_t p_bias);
	virtual real_t shape_get_custom_solver_bias(RID p_shape) const;

	virtual RID space_create();
	virtual void space_set_active(RID p_space, bool p_active);
	virtual bool space_is_active(RID p_space) const;
	virtual void space_set_param(RID p_space, SpaceParameter p_param, real_t p_value);
	virtual real_t space_get_param(RID p_space, SpaceParameter p_param) const;
	virtual PhysicsDirectSpaceState *space_get_direct_state(RID p_space);
	virtual void space_set_debug_contacts(RID p_space, int p_max_contacts);
	virtual Vector<Vector3> space_get_contacts(RID p_space) const;
	virtual int space_get_contact_count(RID p_space) const;

	virtual void free(RID p_rid);

	virtual void set_active(bool p_active);

	virtual void init();
	virtual void step(float p_deltaTime);
	virtual void sync();
	virtual void flush_queries();
	virtual void finish();

	virtual bool is_flushing_queries() const { return false; }

	virtual int get_process_info(ProcessInfo p_info);
};

#endif
