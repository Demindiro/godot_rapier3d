TARGET_LINUX?=x86_64-unknown-linux-gnu
TARGET_OSX?=x86_64-apple-darwin
TARGET_WINDOWS?=x86_64-pc-windows-gnu
OUTPUT_DIR?=addons/rapier3d/lib
GODOT?=godot


default: release

release: linux

linux: rapier3d/api.json
	GODOT_PATH=$(GODOT) cargo build --quiet --target $(TARGET_LINUX) --release
	cp target/$(TARGET_LINUX)/release/librapier3d.so $(OUTPUT_DIR)/librapier3d.so
	strip $(OUTPUT_DIR)/librapier3d.so

debug: rapier3d/api.json
	GODOT_PATH=$(GODOT) cargo build
	cp target/debug/librapier3d.so $(OUTPUT_DIR)/librapier3d.so

export-linux: linux
	$(GODOT) --export "Linux/X11" bin/rapier_test.x86_64

clean:
	cargo clean
	rm rapier3d/api.json || true
	rm module/api.json || true
	rm module/api.gen.h || true


rapier3d/api.json: module/api.json
	cp module/api.json rapier3d/api.json

module/api.json: module/generate.py
	cd module && ./generate.py
