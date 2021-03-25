use super::{ffi, Index};

pub fn init(ffi: &mut ffi::FFI) {
	ffi.space_create(create);
	ffi.space_set_active(set_active);
}

unsafe extern "C" fn create() -> ffi::Index {
	println!("Then God created the World");
	super::add_index(Index::Space(crate::create_space()))
}

unsafe extern "C" fn set_active(index: ffi::Index, active: bool) {
	println!(
		"Then God actived the World, even though it is a no-op {:?} {}",
		index, active
	);
}
