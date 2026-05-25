# run_demo.ps1 — one-command end-to-end pipeline.
#
# Stages:
#   1. Convert a PCL .pcd point cloud into a .vxw voxel world (pixi run python ...)
#   2. Launch Godot 4 to visualize the .vxw
#
# Usage:
#   .\run_demo.ps1                                # default baseline.pcd, 10cm voxels
#   .\run_demo.ps1 -Pcd 'E:\...\global.pcd'      # custom .pcd
#   .\run_demo.ps1 -VoxelSize 0.20                # coarser voxels
#   .\run_demo.ps1 -NoViewer                      # skip Godot, just convert
#   .\run_demo.ps1 -Headless                      # run Godot headless (smoke test)

[CmdletBinding()]
param(
    [string]$Pcd       = 'E:\aros_slam_ws\lightning_lm_foxy\data\test_compare_baseline\global.pcd',
    [double]$VoxelSize = 0.10,
    [string]$OutName,                       # defaults from .pcd parent dir name
    [switch]$NoViewer,
    [switch]$Headless,
    [string]$GodotExe  = 'F:\Godot\Godot_v4.6.3-stable_win64.exe'
)

$ErrorActionPreference = 'Stop'
$ProjectRoot = $PSScriptRoot
Set-Location $ProjectRoot

if (-not (Test-Path $Pcd)) {
    Write-Error "PCD not found: $Pcd"
    exit 1
}

if (-not $OutName) {
    $OutName = (Split-Path -Leaf (Split-Path -Parent $Pcd))  # parent dir name
}
$OutVxw = Join-Path $ProjectRoot "out\$OutName.vxw"
New-Item -ItemType Directory -Force (Split-Path $OutVxw) | Out-Null

Write-Output "=== [1/2] Converting PCD -> VXW ==="
Write-Output "  input  : $Pcd"
Write-Output "  output : $OutVxw"
Write-Output "  voxel  : $VoxelSize m"
Write-Output ""

pixi run python m3_adapter/pcd_to_vxw.py $Pcd $OutVxw `
    --voxel-size $VoxelSize `
    --swap-yz `
    --compression gzip
if ($LASTEXITCODE -ne 0) { Write-Error "pcd_to_vxw.py failed"; exit 1 }

if ($NoViewer) {
    Write-Output ""
    Write-Output "Done. (viewer skipped)"
    exit 0
}

if (-not (Test-Path $GodotExe)) {
    Write-Warning "Godot not found at $GodotExe — skipping viewer launch."
    exit 0
}

Write-Output ""
Write-Output "=== [2/2] Launching Godot viewer ==="
Write-Output "  godot   : $GodotExe"
Write-Output "  project : $ProjectRoot\godot_viewer"
$worldArg = "--world=$($OutVxw.Replace('\','/'))"
Write-Output "  world   : $worldArg"
Write-Output ""

$godotArgs = @('--path', "$ProjectRoot\godot_viewer", '--', $worldArg)
if ($Headless) {
    $godotArgs = @('--path', "$ProjectRoot\godot_viewer", '--headless', '--quit-after', '60', '--', $worldArg)
}
& $GodotExe @godotArgs
