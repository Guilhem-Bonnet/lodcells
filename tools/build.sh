#!/bin/sh
# Compile la GDExtension lodcells et l'installe dans l'addon Godot.
set -eu
here=$(cd "$(dirname "$0")/.." && pwd)
cd "$here"
cargo build -p lodcells-godot --release
mkdir -p "$here/godot/addons/lodcells/bin"
cp target/release/liblodcells_godot.so "$here/godot/addons/lodcells/bin/"
echo "installé : godot/addons/lodcells/bin/liblodcells_godot.so"
