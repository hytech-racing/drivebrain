#!/usr/bin/env sh
set -e

for a in "$@"; do
  case "$a" in
    --test) shouldTest=1 ;;
    --clean) shouldClean=1 ;;
    --local-cache-only) localCacheOnly=1 ;;
  esac
done

profile="rpi_profile"
build_folder="build-arm"

hootl=""
if [ "$shouldTest" = 1 ]; then
  profile="default"
  build_folder="build-native"
  hootl="-DHOOTL=ON"
fi

if [ "$shouldClean" = 1 ]; then
  rm -rf .venv
  rm -rf cmake
fi

python3 -m venv .venv
. .venv/bin/activate
pip install -r requirements.txt

# let cmake infer this
unset CC
unset CXX
unset CMAKE_TOOLCHAIN_FILE

conan profile detect --force

if [ "$localCacheOnly" = 1 ]; then
    conan install . \
    --build=missing \
    --profile:build=default \
    --profile:host="$profile" \
    -of=cmake \
    --no-remote
else
  conan install . \
    --build=missing \
    --profile:build=default \
    --profile:host="$profile" \
    -of=cmake
fi


mkdir -p "$build_folder"
cd "$build_folder"

. ../cmake/conanbuild.sh

cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_TOOLCHAIN_FILE=../cmake/conan_toolchain.cmake \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  -DCMAKE_EXE_LINKER_FLAGS="-static" \
  $hootl \
  --log-level=NOTICE

make -j

# run unit tests
if [ "$shouldTest" = 1 ]; then
  ctest --rerun-failed --output-on-failure
fi

cd ..
ln -sf "$build_folder"/compile_commands.json compile_commands.json
