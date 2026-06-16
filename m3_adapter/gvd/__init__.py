"""GVD subsystem: voxel field -> places graph -> rooms, with a staged pipeline.
See docs/superpowers/specs/2026-06-16-gvd-subpackage-architecture.md.
"""
import sys as _sys
from pathlib import Path as _Path

# Ensure the project root (which holds vxw_format.py) is importable wherever the
# package is used from — tests, the CLI shim, or a standalone import.
_root = str(_Path(__file__).resolve().parents[2])
if _root not in _sys.path:
    _sys.path.insert(0, _root)
