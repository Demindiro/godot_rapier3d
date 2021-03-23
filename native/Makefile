TARGET_LINUX?=x86_64-unknown-linux-gnu
TARGET_OSX?=x86_64-apple-darwin
TARGET_WINDOWS?=x86_64-pc-windows-gnu
OUTPUT_DIR?=../addons/rapier3d/lib


default: release

release: linux

linux:
	cargo build --quiet --target $(TARGET_LINUX) --release
	cp target/$(TARGET_LINUX)/release/libgodot_rapier3d.so $(OUTPUT_DIR)/libgodot_rapier3d.so

debug:
	cargo build
	cp target/debug/libgodot_rapier3d.so $(OUTPUT_DIR)/libgodot_rapier3d.so

clean:
	cargo clean
