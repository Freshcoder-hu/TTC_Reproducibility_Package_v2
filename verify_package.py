#!/usr/bin/env python3
"""Validate the bundled research artifact checksums (Python standard library only)."""
import sys, json, hashlib
from pathlib import Path

base = Path(__file__).resolve().parent
manifest = json.loads((base / "PACKAGE_SHA256_MANIFEST.json").read_text(encoding="utf-8"))
errors = []
checked = 0
for item in manifest["files"]:
    path = base / item["path"]
    if not path.is_file():
        errors.append("MISSING " + item["path"])
        continue
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    if h.hexdigest() != item["sha256"] or path.stat().st_size != item["bytes"]:
        errors.append("MISMATCH " + item["path"])
    checked += 1
print(f"Verified {checked} files; errors = {len(errors)}")
for message in errors[:25]:
    print(message)
sys.exit(1 if errors else 0)
