use gdnative::prelude::*;
use rapier3d::dynamics::{RigidBodyBuilder, RigidBodyHandle};

#[derive(NativeClass)]
#[inherit(Spatial)]
pub struct StaticBody {
    body: Option<RigidBodyHandle>,
}

#[methods]
impl StaticBody {
    fn new(owner: TRef<Spatial>) -> Self {
        Self { body: None }
    }

    #[export]
    fn _enter_tree(&mut self, owner: TRef<Spatial>) {
        assert!(self.body.is_none(), "Body handle is not None");
        let world = owner.get_world().expect("Failed to get World");
        let body = RigidBodyBuilder::new_static().build();
        self.body = Some(crate::add_rigid_body(world, body));
    }
}

impl super::Body for StaticBody {
    fn handle(&self) -> Option<RigidBodyHandle> {
        self.body
    }
}
