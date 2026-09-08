#!/usr/bin/env sh
set -e

profile="rpi_profile"
build_folder="build-arm"

hootl=""
if [ "$1" = "--local" ]; then
    profile="default"
    build_folder="build-native"
    hootl="-DHOOTL=ON"
fi

ht_proto_override=""

if [ -n "${HT_PROTO_SOURCE_DIR:-}" ]; then
    ht_proto_override="-DFETCHCONTENT_SOURCE_DIR_HT_PROTO=${HT_PROTO_SOURCE_DIR}"
    echo "Using local HT_proto source: ${HT_PROTO_SOURCE_DIR}"
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
    -DCMAKE_EXE_LINKER_FLAGS="-static" \
    $hootl \
    $ht_proto_override

make -j

# run unit tests
if [ "$1" = "--local" ]; then
    ctest --rerun-failed --output-on-failure
fi

cd ..
ln -sf "$build_folder"/compile_commands.json compile_commands.json
