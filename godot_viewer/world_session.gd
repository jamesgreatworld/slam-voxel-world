# world_session.gd — owns the loaded world + its path. Initial load, in-place
# swap, reload, backup, world snapshot and litematic import all live here.
# After every in-place swap it emits world_loaded so other controllers rebind.
# (This signal is the seam hot-reload [architecture.md §12 P1] will reuse.)
extends Node

signal world_loaded(world, path: String)

const VxwLoader = preload("res://vxw_loader.gd")

var world  # VxwLoader.VxwWorld
var world_path: String = ""

var _renderer: Node3D
var _logger


func init_session(renderer: Node3D, logger) -> void:
    _renderer = renderer
    _logger = logger


func resolve(p: String) -> String:
    if p.is_absolute_path():
        return p
    var base := ProjectSettings.globalize_path("res://")
    return base.path_join(p)


# Initial boot load. Returns false when the world is empty/missing; the caller
# decides what to show. Does NOT emit world_loaded — boot wiring in main._ready
# runs in explicit order because the other controllers don't exist yet.
func load_initial(world_path_raw: String) -> bool:
    world_path = resolve(world_path_raw)
    world = VxwLoader.load_world(world_path)
    if world.voxel_count() == 0:
        _logger.error("world", {"reason": "empty load", "path": world_path_raw})
        return false
    _log_loaded()
    _renderer.build(world)
    return true


# Menu "Load world" entry — validate, then swap in place.
func request_load(new_path: String) -> void:
    _logger.info("world_load_requested", {"new_path": new_path, "old_path": world_path})
    if not DirAccess.dir_exists_absolute(new_path):
        _logger.error("world_load_requested", {"reason": "dir missing", "path": new_path})
        return
    if not FileAccess.file_exists(new_path + "/manifest.json"):
        _logger.error("world_load_requested", {"reason": "manifest.json missing in selected dir", "path": new_path})
        return
    load_in_place(new_path)


func reload() -> void:
    _logger.info("world_reload", {"path": world_path})
    load_in_place(world_path)


# In-place world swap: tear down renderer children + rebuild. Listener rebind
# (voxel editor / entities / HUD / rig) happens via the world_loaded signal.
func load_in_place(path: String) -> void:
    var new_world = VxwLoader.load_world(path)
    if new_world.voxel_count() == 0:
        _logger.error("world_load_in_place", {"reason": "empty world", "path": path})
        return
    world = new_world
    world_path = path
    for child in _renderer.get_children():
        child.queue_free()
    _renderer.build(world)
    _log_loaded()
    world_loaded.emit(world, world_path)


func save_backup() -> void:
    var ts := Time.get_datetime_string_from_system().replace(":", "-").replace("T", "_")
    var src := world_path
    var dir := src.get_base_dir()
    var base := src.get_file()
    if base.ends_with(".vxw"):
        base = base.substr(0, base.length() - 4)
    var dst := "%s/%s_backup_%s.vxw" % [dir, base, ts]
    var ok := _copy_dir_recursive(src, dst)
    _logger.info("world_backup", {"src": src, "dst": dst, "ok": ok})


func save_snapshot() -> void:
    # Snapshot the current world dir into out/snapshots/<base>_<ts>/
    var src := world_path
    if src == "" or not DirAccess.dir_exists_absolute(src):
        if _logger != null:
            _logger.error("world_snapshot", {"reason": "src missing", "src": src})
        return
    var ts := Time.get_datetime_string_from_system().replace(":", "-").replace("T", "_")
    var base := src.get_file()
    if base.ends_with(".vxw"):
        base = base.substr(0, base.length() - 4)
    var proj_root := ProjectSettings.globalize_path("res://..")
    var dst_root := proj_root + "/out/snapshots"
    DirAccess.make_dir_recursive_absolute(dst_root)
    var dst := "%s/%s_%s" % [dst_root, base, ts]
    var ok := _copy_dir_recursive(src, dst)
    if _logger != null:
        _logger.info("world_snapshot", {"src": src, "dst": dst, "ok": ok})


func import_litematic(path: String) -> void:
    _logger.info("import_litematic", {"src": path})
    var basename: String = path.get_file().get_basename()
    var out_vxw: String = ProjectSettings.globalize_path("res://../out") + "/" + basename + ".vxw"
    var proj_root: String = ProjectSettings.globalize_path("res://..")
    var pixi_cmd := "pixi"
    var args := [
        "run", "python",
        proj_root + "/m3_adapter/litematic_to_vxw.py",
        path, out_vxw,
        "--voxel-size", "1.0",
        "--compression", "gzip",
    ]
    var output: Array = []
    var exit_code: int = OS.execute(pixi_cmd, args, output, true, true)
    if exit_code != 0:
        _logger.error("import_litematic", {"exit_code": exit_code, "output": output})
        return
    _logger.info("import_litematic_ok", {"out": out_vxw})
    load_in_place(out_vxw)


func _log_loaded() -> void:
    _logger.info("world_loaded", {
        "voxels": world.voxel_count(),
        "voxel_size_m": world.voxel_size_meters,
        "chunk_extent": world.chunk_extent,
        "path": world_path,
    })


func _copy_dir_recursive(src: String, dst: String) -> bool:
    if not DirAccess.dir_exists_absolute(src):
        push_error("[world_session] copy_dir source missing: " + src)
        return false
    DirAccess.make_dir_recursive_absolute(dst)
    var d := DirAccess.open(src)
    if d == null:
        return false
    d.list_dir_begin()
    var name := d.get_next()
    while name != "":
        if name == "." or name == "..":
            name = d.get_next()
            continue
        var sp := src + "/" + name
        var dp := dst + "/" + name
        if d.current_is_dir():
            _copy_dir_recursive(sp, dp)
        else:
            DirAccess.copy_absolute(sp, dp)
        name = d.get_next()
    return true
