#!/usr/bin/env sh
set -e

profile="rpi_profile"
build_folder="build-arm"

hootl=""
local_build=false
flir=OFF
linker_flags="-static"
for arg in "$@"; do
  case "$arg" in
    --local) local_build=true ;;
    --flir) flir=ON; linker_flags="" ;;
    *) echo "Usage: $0 [--local] [--flir]" >&2; exit 1 ;;
  esac
done
if [ "$flir" = ON ] && [ "$local_build" = false ]; then
  echo "--flir requires --local; the ARM profile has no Jetson sysroot." >&2
  exit 1
fi
if [ "$local_build" = true ]; then
  profile="default"
  build_folder="build-native"
  hootl="-DHOOTL=ON"
fi

rm -rf .venv
# rm -rf "$build_folder"
rm -rf cmake

python3 -m venv .venv
. .venv/bin/activate
pip install -r requirements.txt

# let cmake infer this
unset CC
unset CXX
unset CMAKE_TOOLCHAIN_FILE

conan profile detect --force

# Export the checked-in source URL fixes before resolving dependencies.
conan export conan/recipes/libiconv --version=1.17
conan export conan/recipes/gettext --version=0.22.5

conan install . \
  --build=missing \
  --profile:build=default \
  --profile:host="$profile" \
  -of=cmake

mkdir -p "$build_folder"
cd "$build_folder"

. ../cmake/conanbuild.sh

cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_TOOLCHAIN_FILE=../cmake/conan_toolchain.cmake \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  -DCMAKE_EXE_LINKER_FLAGS="$linker_flags" \
  -DDRIVEBRAIN_FLIR="$flir" \
  $hootl

ln -sf "$build_folder"/compile_commands.json ../compile_commands.json

make -j

# run unit tests
if [ "$local_build" = true ]; then
  ctest --rerun-failed --output-on-failure
fi

cd ..
