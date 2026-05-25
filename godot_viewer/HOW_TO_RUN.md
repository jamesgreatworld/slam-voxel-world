# How to run the vxw_viewer

## 1. Get Godot (one time, ~80 MB)

Download **Godot Engine 4.x Standard** (NOT the .NET version) from
<https://godotengine.org/download/windows>.

You get a single file like `Godot_v4.x.x-stable_win64.exe`. Put it wherever
you like, e.g. `F:\Godot\Godot.exe`. No install, no admin, no account.

## 2. Open this project

1. Double-click `Godot.exe`.
2. In the project manager, click **Import** (top right).
3. Browse to `F:\slam-voxel-world\godot_viewer\` and select `project.godot`.
4. Click **Import & Edit**.

The editor will open. The Output panel at the bottom may show a few warnings
the first time — ignore.

## 3. Run

Press **F5** (or click the ▶ Play button top-right).

If Godot asks for a main scene, pick `main.tscn`.

You should see:
- A flat-ish grey "fog" of cubes — that's your scanned point cloud as voxels
- HUD top-left: `vxw_viewer | 70811 voxels | voxel 0.10m | ...`
- WASD + mouse to fly around. Space = up, Ctrl = down. Shift = boost.
  Tab = release mouse. Esc = quit.

If you see "No voxels loaded" — check the Output panel for errors.
The default path is `../out/baseline.vxw` relative to this folder, so make
sure you've run `pixi run python m3_adapter/pcd_to_vxw.py ...` first.

## 4. Load a different world

Run from the command line:

```powershell
F:\Godot\Godot.exe --path F:\slam-voxel-world\godot_viewer -- --world=F:/some/other.vxw
```

Note the second `--` separates Godot's args from your script's args.
