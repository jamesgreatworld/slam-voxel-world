"""features.py — modular, pluggable object feature/descriptor framework for
data association. Each feature is an ObjectFeature plugin (name, weight,
extract, distance). Object descriptors compose over the active feature set, so
new features (colour histogram, CNN crop embedding, ORB) plug in without
changing the matcher. (Object tracking — DeepSORT-style appearance, extensible.)

# Example extension (not implemented yet):
# class ColorFeature(ObjectFeature):
#     name = "color"; weight = 1.5
#     def extract(self, cells, voxel_size, ctx):
#         rgb = ctx["rgb"]            # per-voxel RGB grid, provided by caller
#         vals = rgb[cells[:,0], cells[:,1], cells[:,2]]
#         hist, _ = np.histogramdd(vals, bins=(4,4,4), range=[(0,256)]*3)
#         h = hist.ravel(); return tuple((h / max(h.sum(),1)).tolist())
#     def distance(self, a, b):       # Bhattacharyya
#         a=np.asarray(a); b=np.asarray(b); return float(1 - np.sqrt(a*b).sum())
# Then: DEFAULT_FEATURES.append(ColorFeature())  — matcher auto-uses it.
"""
from __future__ import annotations
from abc import ABC, abstractmethod
import numpy as np


class ObjectFeature(ABC):
    name: str = "base"
    weight: float = 1.0

    @abstractmethod
    def extract(self, cells: np.ndarray, voxel_size: float, ctx: dict) -> tuple:
        """Compute this feature's vector for one object.
        cells: (M,3) dense voxel coords of the object's cluster.
        ctx: free-form context (e.g. {'occ':..., 'sem':..., 'rgb':..., 'frames':...})
             so features needing more than geometry (colour, image crops) can reach it."""
        ...

    @abstractmethod
    def distance(self, a: tuple, b: tuple) -> float:
        """Distance between two feature vectors of this type (>=0)."""
        ...


class ShapeFeature(ObjectFeature):
    """Rotation-invariant shape: principal-axis std-devs (metres), descending."""
    name = "shape"
    weight = 2.0

    def extract(self, cells, voxel_size, ctx):
        c = cells.astype(np.float64)
        if len(c) < 3:
            return (0.0, 0.0, 0.0)
        c = c - c.mean(axis=0)
        cov = (c.T @ c) / len(c)
        ev = np.clip(np.linalg.eigvalsh(cov), 0, None)
        return tuple(float(np.sqrt(e) * voxel_size) for e in ev[::-1])

    def distance(self, a, b):
        a = np.asarray(a, dtype=float); b = np.asarray(b, dtype=float)
        n = max(len(a), len(b))
        a = np.pad(a, (0, n - len(a))); b = np.pad(b, (0, n - len(b)))
        return float(np.linalg.norm(a - b))


# The active feature set. Append new features here (or pass a custom list).
DEFAULT_FEATURES: list = [ShapeFeature()]


def extract_features(cells, voxel_size, ctx=None, features=None) -> dict:
    feats = features if features is not None else DEFAULT_FEATURES
    ctx = ctx or {}
    return {f.name: f.extract(cells, voxel_size, ctx) for f in feats}


def feature_cost(feats_a: dict, feats_b: dict, features=None) -> float:
    """Sum of weight * distance over features present in BOTH descriptors."""
    feats = features if features is not None else DEFAULT_FEATURES
    total = 0.0
    for f in feats:
        a = feats_a.get(f.name); b = feats_b.get(f.name)
        if a is not None and b is not None and len(a) and len(b):
            total += f.weight * f.distance(a, b)
    return total
