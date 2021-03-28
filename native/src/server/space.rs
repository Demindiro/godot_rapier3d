use super::{ffi, Index};

pub fn init(ffi: &mut ffi::FFI) {
	ffi.space_create(create);
	ffi.space_set_active(set_active);
}

fn create() -> Option<Index> {
	Some(Index::add_space(crate::create_space()))
}

fn set_active(index: Index, active: bool) {
	println!(
		"Then God actived the World, even though it is a no-op {:?} {}",
		index, active
	);
}
