@echo off
cargo build --target x86_64-pc-windows-msvc --release
xcopy target/x86_64-pc-windows-msvc/release/rapier3d.dll addons/rapier3d/lib/rapier3d.dll
