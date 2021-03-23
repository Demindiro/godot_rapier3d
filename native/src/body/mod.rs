mod kinematic;
mod rigid;
mod r#static;

pub use kinematic::*;
pub use r#static::*;
pub use rigid::*;

use rapier3d::dynamics::RigidBodyHandle;

pub trait Body {
    fn handle(&self) -> Option<RigidBodyHandle>;
}
