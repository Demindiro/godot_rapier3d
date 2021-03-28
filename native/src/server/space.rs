use super::{ffi, Index};

pub fn init(ffi: &mut ffi::FFI) {
	ffi.space_create(create);
	ffi.space_set_active(set_active);
}

unsafe extern "C" fn create() -> *const Index {
	Index::add_space(crate::create_space()).raw()
}

unsafe extern "C" fn set_active(index: *const Index, active: bool) {
	println!(
		"Then God actived the World, even though it is a no-op {:?} {}",
		index, active
	);
}
