"""
Auto-download and cache HT_proto from the latest release
(i got this idea from pytorch's api for downloading training datasets)

HT_CAN is pinned by release tag instead, since its tags are plain ints and the
CAN definitions have to match whatever drivebrain was built against.
"""

import importlib
import json
import os
import sys
import tarfile
import tempfile
import types
import urllib.request
from pathlib import Path

GITHUB_API_URL = "https://api.github.com/repos/hytech-racing/HT_proto/releases/latest"
ASSET_NAME = "python_hytech_msgs_proto_lib.tar.gz"
CACHE_DIR = Path.home() / ".cache" / "hytech_proto"

CAN_API_URL = "https://api.github.com/repos/hytech-racing/HT_CAN/releases/tags/{tag}"
CAN_ASSET_NAME = "python_proto_lib.tar.gz"
CAN_CACHE_DIR = Path.home() / ".cache" / "hytech_can"

def _get_release_url(api_url: str, asset_name: str) -> tuple[str, str]:
    """Return (tag, download_url) for an asset in the release at api_url"""
    req = urllib.request.Request(api_url, headers={"Accept": "application/vnd.github+json"})
    with urllib.request.urlopen(req) as resp:
        data = json.loads(resp.read())
    tag = data["tag_name"]
    for asset in data["assets"]:
        if asset["name"] == asset_name:
            return tag, asset["browser_download_url"]
    raise FileNotFoundError(f"{asset_name} not found in release {tag}")

def _get_latest_release_url() -> tuple[str, str]:
    """Return (tag, download_url) from the latest release"""
    return _get_release_url(GITHUB_API_URL, ASSET_NAME)

def _download_and_extract(url: str, dest: Path, asset_name: str = ASSET_NAME) -> None:
    dest.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(suffix=".tar.gz", delete=False) as tmp:
        tmp_path = tmp.name
        print(f"Downloading {asset_name}...")
        urllib.request.urlretrieve(url, tmp_path)
    try:
        with tarfile.open(tmp_path, "r:gz") as tar:
            tar.extractall(dest, filter="data")
        print(f"Extracted to {dest}")
    finally:
        os.unlink(tmp_path)

def _write_version(dest: Path, tag: str) -> None:
    (dest / ".version").write_text(tag)

def _read_version(dest: Path) -> str | None:
    vf = dest / ".version"
    return vf.read_text().strip() if vf.exists() else None

def ensure_proto() -> Path:
    """Download the proto lib if missing or a newer release exists

    Always checks GitHub for the latest tag and updates if out of date
    """
    cached_version = _read_version(CACHE_DIR)
    tag, url = _get_latest_release_url()

    if cached_version == tag:
        return CACHE_DIR

    print(f"HT_proto: {'updating ' + cached_version + ' -> ' + tag if cached_version else 'downloading ' + tag}")
    _download_and_extract(url, CACHE_DIR)
    _write_version(CACHE_DIR, tag)
    return CACHE_DIR

def ensure_can(tag: int) -> Path:
    """Download the CAN proto lib for a release tag if it isn't already cached

    Each tag caches to its own directory, so switching between them costs nothing
    after the first download.
    """
    dest = CAN_CACHE_DIR / str(tag)
    if _read_version(dest) == str(tag):
        return dest

    release_tag, url = _get_release_url(CAN_API_URL.format(tag=tag), CAN_ASSET_NAME)
    print(f"HT_CAN: downloading {release_tag}")
    _download_and_extract(url, dest, CAN_ASSET_NAME)
    _write_version(dest, release_tag)
    return dest


def load_proto() -> types.ModuleType:
    """Load hytech_msgs as a module

    proto = load_proto()
    msg = proto.hytech_msgs_pb2.SomeMessage(...)
    """
    proto_dir = ensure_proto()
    lib_dir = proto_dir / "python_hytech_msgs_proto_lib"

    if str(lib_dir) not in sys.path:
        sys.path.insert(0, str(lib_dir))

    mod = types.ModuleType("hytech_proto")
    mod.hytech_msgs_pb2 = importlib.import_module("hytech_msgs_pb2")
    mod.base_msgs_pb2 = importlib.import_module("base_msgs_pb2")
    mod.dv_msgs_pb2 = importlib.import_module("dv_msgs_pb2")
    return mod

def load_can(tag: int) -> types.ModuleType:
    """Load the CAN messages from a given HT_CAN release as a module

    can = load_can(268)
    msg = can.hytech_pb2.drivebrain_steering_input(...)
    """
    can_dir = ensure_can(tag)
    lib_dir = can_dir / "python_can_lib"

    if str(lib_dir) not in sys.path:
        sys.path.insert(0, str(lib_dir))

    mod = types.ModuleType("hytech_can")
    mod.hytech_pb2 = importlib.import_module("hytech_pb2")
    return mod
