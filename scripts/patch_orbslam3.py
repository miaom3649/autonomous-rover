#!/usr/bin/env python3
"""Apply rover-project patches to ORB_SLAM3 and rebuild the shared library.

Run on the Raspberry Pi after a fresh ORB_SLAM3 clone/build:
    python3 scripts/patch_orbslam3.py

Patches applied:
  1. System.cc — don't prepend "./" to absolute atlas save paths
     Without this, System.SaveAtlasToFile: "/home/.../room" gets written to
     ".//home/.../room.osa" (relative to cwd), which fails with a boost
     archive output stream error.

  2. Map.cc — copy mspMapPoints before iterating in PreSave
     EraseObservation() -> SetBadFlag() -> EraseMapPoint() removes the
     MapPoint from mspMapPoints mid-iteration, invalidating the STL iterator
     and causing SIGSEGV in _Rb_tree_increment.
"""
import re
import subprocess
import sys
from pathlib import Path

ORB_DIR = Path.home() / "ORB_SLAM3"


def apply_literal(path: Path, old: str, new: str, count: int = 1) -> None:
    text = path.read_text()
    if old not in text:
        print(f"  SKIP {path.name}: pattern not found (already patched?)")
        return
    path.write_text(text.replace(old, new, count))
    print(f"  OK   {path.name}")


def apply_regex(path: Path, pattern: str, replacement: str, count: int = 1) -> None:
    text = path.read_text()
    patched, n = re.subn(pattern, replacement, text, count=count)
    if n == 0:
        print(f"  SKIP {path.name}: pattern not found (already patched?)")
        return
    path.write_text(patched)
    print(f"  OK   {path.name}: {n} substitution(s)")


print("=== Patching ORB_SLAM3 ===\n")

print("[1/2] System.cc — absolute atlas save path fix")
apply_literal(
    ORB_DIR / "src/System.cc",
    old='string pathSaveFileName = "./";\n',
    new=(
        'string pathSaveFileName = '
        '(!mStrSaveAtlasToFile.empty() && mStrSaveAtlasToFile[0] == \'/\') ? "" : "./";\n'
    ),
)

print("[2/2] Map.cc — PreSave iterator invalidation fix (first loop only)")
apply_regex(
    ORB_DIR / "src/Map.cc",
    pattern=r"([ \t]+)for\(MapPoint\* pMPi : mspMapPoints\)",
    replacement=(
        r"\1// Copy before iterating: EraseObservation may call SetBadFlag"
        r" which removes from mspMapPoints\n"
        r"\1std::set<MapPoint*> mspMapPointsCopy(mspMapPoints);\n"
        r"\1for(MapPoint* pMPi : mspMapPointsCopy)"
    ),
    count=1,
)

print("\n=== Rebuilding ORB_SLAM3 ===\n")
cpu = __import__("os").cpu_count() or 4
result = subprocess.run(["make", f"-j{cpu}"], cwd=ORB_DIR / "build")
if result.returncode != 0:
    print("Build failed.", file=sys.stderr)
    sys.exit(result.returncode)

print("\nDone. Restart orb_slam3_node to pick up the updated .so.")
