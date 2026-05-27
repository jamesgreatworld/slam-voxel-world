"""generate_textures — programmatic 16×16 PNG textures for the MC item pack.

We don't ship any Mojang assets. Instead we synthesise plausible
vanilla-style block textures with numpy: planks have wood-grain stripes,
wool has high-frequency noise, iron has metallic noise, stained glass has
a gradient + frame, etc. Output: mc_item_pack/textures/block/*.png.

The colours match _TEXTURE_COLORS in mc_item_loader.py so the textured and
untextured renderings stay visually consistent.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from PIL import Image

_OUT_DIR = Path(__file__).resolve().parent / "textures" / "block"


def _rng(seed: int):
    return np.random.default_rng(seed)


def _save(name: str, img: np.ndarray) -> None:
    _OUT_DIR.mkdir(parents=True, exist_ok=True)
    Image.fromarray(img).save(_OUT_DIR / f"{name}.png")


def _clip(arr: np.ndarray) -> np.ndarray:
    return np.clip(arr, 0, 255).astype(np.uint8)


# ----- texture recipes ----------------------------------------------------

def planks(name: str, rgb: tuple[int, int, int], dark_amount: int = 30,
           seed: int = 0) -> None:
    base = np.array(rgb, dtype=np.int16)
    img = np.tile(base, (16, 16, 1))
    # 2 horizontal seams (plank divisions)
    img[3:5, :] -= dark_amount
    img[11:13, :] -= dark_amount
    # vertical plank end at column 7 on upper plank, column 8 on lower
    img[:8, 7, :] -= dark_amount
    img[8:, 8, :] -= dark_amount
    # fine grain noise
    img = img + _rng(seed).integers(-12, 12, size=img.shape, dtype=np.int16)
    _save(name, _clip(img))


def log_top(name: str, rgb: tuple[int, int, int], seed: int = 0) -> None:
    base = np.array(rgb, dtype=np.int16)
    img = np.tile(base, (16, 16, 1))
    y, x = np.ogrid[-8:8, -8:8]
    rad = np.sqrt(y * y + x * x)
    for r in [2.5, 4.5, 6.5]:
        mask = np.abs(rad - r) < 0.7
        img[mask] -= 22
    img = img + _rng(seed).integers(-6, 6, size=img.shape, dtype=np.int16)
    _save(name, _clip(img))


def wool(name: str, rgb: tuple[int, int, int], seed: int = 0,
         noise_amp: int = 14) -> None:
    base = np.array(rgb, dtype=np.int16)
    img = np.tile(base, (16, 16, 1))
    img = img + _rng(seed).integers(-noise_amp, noise_amp, size=img.shape, dtype=np.int16)
    _save(name, _clip(img))


def metal(name: str, rgb: tuple[int, int, int], seed: int = 0) -> None:
    base = np.array(rgb, dtype=np.int16)
    img = np.tile(base, (16, 16, 1))
    # vertical brushed-metal streaks
    streak = _rng(seed).integers(-10, 10, size=(1, 16, 1), dtype=np.int16)
    img = img + streak
    # speckles
    img = img + _rng(seed + 1).integers(-20, 20, size=img.shape, dtype=np.int16)
    _save(name, _clip(img))


def concrete(name: str, rgb: tuple[int, int, int], seed: int = 0) -> None:
    base = np.array(rgb, dtype=np.int16)
    img = np.tile(base, (16, 16, 1))
    img = img + _rng(seed).integers(-7, 7, size=img.shape, dtype=np.int16)
    _save(name, _clip(img))


def stained_glass(name: str, rgb: tuple[int, int, int], seed: int = 0) -> None:
    base = np.array(rgb, dtype=np.int16)
    img = np.tile(base, (16, 16, 1))
    grad = np.linspace(30, -15, 16, dtype=np.int16).reshape(-1, 1, 1)
    img = img + grad
    # darker frame (1px on each side)
    img[0, :] = base // 3
    img[-1, :] = base // 3
    img[:, 0] = base // 3
    img[:, -1] = base // 3
    _save(name, _clip(img))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out-dir", type=Path, default=_OUT_DIR)
    args = ap.parse_args()
    globals()["_OUT_DIR"] = args.out_dir
    planks("oak_planks",       (159, 132, 77),  dark_amount=30, seed=1)
    planks("dark_oak_planks",  ( 80,  53, 27),  dark_amount=20, seed=2)
    log_top("oak_log_top",     (165, 130, 65),                  seed=3)
    wool("red_wool",           (165,  36, 27),                  seed=4)
    wool("yellow_wool",        (240, 180, 47),                  seed=5)
    wool("white_wool",         (236, 236, 236), noise_amp=10,   seed=6)
    wool("light_blue_wool",    ( 96, 168, 213),                 seed=7)
    metal("iron_block",        (220, 220, 220),                 seed=8)
    concrete("black_concrete", ( 15,  16,  19),                 seed=9)
    concrete("gray_concrete",  ( 62,  68,  73),                 seed=10)
    stained_glass("light_blue_stained_glass", (101, 159, 217),  seed=11)
    print("wrote 11 textures to", args.out_dir)


if __name__ == "__main__":
    main()
