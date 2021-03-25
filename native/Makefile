TARGET_LINUX?=x86_64-unknown-linux-gnu
TARGET_OSX?=x86_64-apple-darwin
TARGET_WINDOWS?=x86_64-pc-windows-gnu
OUTPUT_DIR?=../addons/rapier3d/lib
GODOT?=godot


default: release

release: linux

linux:
	GODOT_PATH=$(GODOT) cargo build --quiet --target $(TARGET_LINUX) --release
	cp target/$(TARGET_LINUX)/release/libgodot_rapier3d.so $(OUTPUT_DIR)/libgodot_rapier3d.so

debug:
	GODOT_PATH=$(GODOT) cargo build
	cp target/debug/libgodot_rapier3d.so $(OUTPUT_DIR)/libgodot_rapier3d.so

clean:
	cargo clean
