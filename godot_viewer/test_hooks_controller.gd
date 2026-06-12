# test_hooks_controller.gd — executes the --test-* CLI hooks after the world
# and all controllers are ready. Keeps the test-only paths out of main.gd.
# Hook semantics are unchanged from the pre-split main.gd; see each block.
extends Node

var _cfg          # boot_config
var _ws           # world_session
var _es           # edit_session
var _ents         # entity_subsystem
var _renderer: Node3D
var _env          # environment_controller
var _logger


func init_hooks(cfg, ws, es, ents, renderer: Node3D, env_ctl, logger) -> void:
    _cfg = cfg
    _ws = ws
    _es = es
    _ents = ents
    _renderer = renderer
    _env = env_ctl
    _logger = logger


func _read_entity_records() -> Array:
    var ent_path: String = _ws.world_path + "/entities.json"
    if not FileAccess.file_exists(ent_path):
        return []
    var txt := FileAccess.get_file_as_string(ent_path)
    if txt.is_empty():
        return []
    var d = JSON.parse_string(txt)
    if d == null or not d.has("entities"):
        return []
    return d.entities


func run() -> void:
    if _cfg.test_delete_first or _cfg.test_rotate_first_deg != 0.0 or _cfg.test_grab_first_set:
        var rec_list := _read_entity_records()
        if rec_list.size() > 0:
            var first_id := String(rec_list[0].get("id", ""))
            if _cfg.test_grab_first_set:
                _ents.selector.grab_to(first_id, _cfg.test_grab_first_to)
            if _cfg.test_rotate_first_deg != 0.0:
                _ents.selector.rotate_by_id(first_id, deg_to_rad(_cfg.test_rotate_first_deg))
            if _cfg.test_delete_first:
                _ents.selector.delete_by_id(first_id)

    for _i in _cfg.test_undo_times:
        _es.undo_last_edit()

    if _cfg.test_duplicate_first:
        var recs := _read_entity_records()
        if recs.size() > 0:
            _ents.selector._selected_id = String(recs[0].get("id", ""))
            _ents.edit.duplicate_selected()

    if _cfg.test_snapshot_world:
        _ws.save_snapshot()

    if _cfg.test_toggle_behavior_on_first:
        # Find the first entity whose preset declares "switchable" and toggle
        # it. Logs the resulting state so the caller can assert
        # entities.json[<idx>].custom_meta.state == "on".
        var recs3 := _read_entity_records()
        var presets3: Dictionary = _ents.ent_renderer.get_item_presets()
        var picked_id := ""
        var picked_idx := -1
        for i in recs3.size():
            var rec: Dictionary = recs3[i]
            var item_id: String = String(rec.get("custom_meta", {}).get("mc_item", ""))
            if item_id == "" or not presets3.has(item_id):
                continue
            var behs: Array = presets3[item_id].get("behaviors", [])
            if behs.has("switchable"):
                picked_id = String(rec.get("id", ""))
                picked_idx = i
                break
        if picked_id != "":
            _ents.selector._selected_id = picked_id
            var ok3: bool = bool(_ents.edit.use_selected())
            _logger.info("test_toggle_behavior_on_first",
                        {"id": picked_id, "index": picked_idx, "applied": ok3})
        else:
            _logger.info("test_toggle_behavior_on_first",
                        {"status": "no switchable entity"})

    if _cfg.test_hide_voxel_set:
        # Phase-2 dirty-rebuild verification hook. Hide a single voxel and
        # log whether the renderer accepted the call. The snapshot frame
        # (20 frames later by default) gives _process plenty of time to
        # rebuild the affected chunk, so visual evidence is the snapshot
        # itself; log evidence is the print on the next dirty drain.
        var hv_ok := bool(_renderer.hide_voxel(_cfg.test_hide_voxel_vi))
        _logger.info("test_hide_voxel", {
            "vi": [_cfg.test_hide_voxel_vi.x, _cfg.test_hide_voxel_vi.y, _cfg.test_hide_voxel_vi.z],
            "ok": hv_ok,
            "still_has": _renderer.has_voxel(_cfg.test_hide_voxel_vi),
        })

    if _cfg.test_toggle_day_night:
        # CLI hook: flip to Night so the next snapshot frame captures the
        # darker visuals. Logs the new DirectionalLight.light_energy so
        # callers can grep for the value (0.55 = night, 1.6 = day).
        _env.toggle_day_night()
