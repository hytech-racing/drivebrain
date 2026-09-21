#!/usr/bin/env python3
"""
Bundles the cross-compiled drivebrain binary's runtime shared-library closure
(plus the dynamic linker itself) into a lib/ directory next to it, and
repoints the binary's ELF interpreter/RPATH at that bundle via patchelf.

phoenix6 (drivebrain_comms/KrakenComms) ships only as a shared library, no
static .a, so -static linking is no longer possible now that drivebrain links
against it -- this makes the resulting dynamic binary self-contained instead:
it no longer depends on the deploy target providing phoenix6, the conan
packages it was built against, or even a standard /lib/ld-linux-aarch64.so.1
(notably, NixOS -- the actual deploy target -- provides none of those).
"""
import os
import re
import shutil
import subprocess
import sys

READELF = "aarch64-linux-gnu-readelf"
DEPLOY_LIB_DIR = "/opt/drivebrain/lib"


def is_aarch64_elf(path):
    try:
        out = subprocess.run([READELF, "-h", path], capture_output=True, text=True, check=True).stdout
    except subprocess.CalledProcessError:
        return False
    return "AArch64" in out


def build_lib_index(root_dirs):
    index = {}
    for root_dir in root_dirs:
        if not os.path.isdir(root_dir):
            continue
        for dirpath, _, filenames in os.walk(root_dir):
            for fname in filenames:
                if ".so" not in fname:
                    continue
                full = os.path.join(dirpath, fname)
                if os.path.isfile(full):
                    index.setdefault(fname, []).append(full)
    return index


def locate(name, index):
    for candidate in index.get(name, []):
        if is_aarch64_elf(candidate):
            return candidate
    return None


def needed_entries(elf_path):
    out = subprocess.run([READELF, "-d", elf_path], capture_output=True, text=True, check=True).stdout
    return re.findall(r"\(NEEDED\)\s+Shared library: \[(.*?)\]", out)


def interpreter_name(elf_path):
    out = subprocess.run([READELF, "-l", elf_path], capture_output=True, text=True, check=True).stdout
    match = re.search(r"Requesting program interpreter: (\S+)\]", out)
    return os.path.basename(match.group(1)) if match else None


def find_search_dirs():
    dirs = ["/usr/lib/phoenix6", "/lib/aarch64-linux-gnu", "/usr/lib/aarch64-linux-gnu"]
    conan_home = os.environ.get("CONAN_HOME", os.path.expanduser("~/.conan2"))
    packages_dir = os.path.join(conan_home, "p")
    if os.path.isdir(packages_dir):
        dirs.append(packages_dir)
    return dirs


def main():
    if len(sys.argv) != 3:
        print(f"usage: {sys.argv[0]} <binary> <output-lib-dir>", file=sys.stderr)
        return 1

    binary_path, lib_dir = sys.argv[1], sys.argv[2]
    os.makedirs(lib_dir, exist_ok=True)
    index = build_lib_index(find_search_dirs())

    interp_name = interpreter_name(binary_path)
    if not interp_name:
        print(f"error: {binary_path} has no ELF interpreter (already static?)", file=sys.stderr)
        return 1

    to_visit = [interp_name] + needed_entries(binary_path)
    visited = set()
    missing = []
    while to_visit:
        name = to_visit.pop()
        if name in visited:
            continue
        visited.add(name)

        src = locate(name, index)
        if src is None:
            missing.append(name)
            continue

        shutil.copy2(src, os.path.join(lib_dir, name))
        to_visit.extend(needed_entries(src))

    if missing:
        print(f"warning: could not locate {missing} in any known search path; "
              f"the deployed binary will need these to already exist on the target", file=sys.stderr)

    subprocess.run([
        "patchelf",
        "--set-interpreter", f"{DEPLOY_LIB_DIR}/{interp_name}",
        "--set-rpath", "$ORIGIN/lib",
        binary_path,
    ], check=True)

    print(f"Bundled {len(visited) - len(missing)} runtime libraries into {lib_dir}/")
    print(f"Interpreter set to {DEPLOY_LIB_DIR}/{interp_name}, RPATH set to $ORIGIN/lib")
    return 0


if __name__ == "__main__":
    sys.exit(main())
