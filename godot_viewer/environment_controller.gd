# environment_controller.gd — Day/Night visual state.
# Tweaks DirectionalLight3D + WorldEnvironment.environment (+ procedural sky
# material) between two hardcoded palettes. The Day values mirror what
# main.tscn bakes in so a fresh launch is visually identical. Lamp / computer
# emission_energy_multiplier on entity meshes keeps them readable in Night.
extends Node

const _DAY_LIGHT_ENERGY: float = 1.05
const _DAY_LIGHT_COLOR: Color = Color(1, 1, 1)
const _DAY_AMBIENT_COLOR: Color = Color(0.4, 0.45, 0.55, 1)
const _DAY_AMBIENT_ENERGY: float = 0.55
const _DAY_SKY_TOP: Color = Color(0.4, 0.5, 0.65, 1)
const _DAY_SKY_HORIZON: Color = Color(0.65, 0.7, 0.75, 1)
const _DAY_GROUND_BOTTOM: Color = Color(0.1, 0.1, 0.1, 1)
const _DAY_GROUND_HORIZON: Color = Color(0.45, 0.4, 0.35, 1)

const _NIGHT_LIGHT_ENERGY: float = 0.55
const _NIGHT_LIGHT_COLOR: Color = Color(0.82, 0.88, 1.0)
const _NIGHT_AMBIENT_COLOR: Color = Color(0.20, 0.24, 0.34, 1)
const _NIGHT_AMBIENT_ENERGY: float = 0.12
const _NIGHT_SKY_TOP: Color = Color(0.05, 0.07, 0.16, 1)
const _NIGHT_SKY_HORIZON: Color = Color(0.12, 0.16, 0.26, 1)
const _NIGHT_GROUND_BOTTOM: Color = Color(0.02, 0.02, 0.05, 1)
const _NIGHT_GROUND_HORIZON: Color = Color(0.08, 0.10, 0.16, 1)
# Sharper shadows in moonlight — real moonlight casts harder edges than the
# soft default. Day mode keeps the default blur.
const _DAY_SHADOW_BLUR: float = 1.0
const _NIGHT_SHADOW_BLUR: float = 0.5

var _light: DirectionalLight3D
var _world_env: WorldEnvironment
var _pause_menu: CanvasLayer
var _logger
# Day/Night state. Default is Day to match the values baked into main.tscn.
var _night_mode: bool = false


func init_controller(light: DirectionalLight3D, world_env: WorldEnvironment,
        pause_menu: CanvasLayer, logger) -> void:
    _light = light
    _world_env = world_env
    _pause_menu = pause_menu
    _logger = logger


func is_night() -> bool:
    return _night_mode


func toggle_day_night() -> void:
    _night_mode = not _night_mode
    if _night_mode:
        _apply_night_mode()
    else:
        _apply_day_mode()
    if _pause_menu != null:
        _pause_menu.set_day_night_label(_night_mode)
    if _logger != null:
        var le := 0.0
        if _light != null:
            le = _light.light_energy
        _logger.info("day_night_toggle", {
            "night_mode": _night_mode,
            "directional_light_energy": le,
        })


func _get_sky_material() -> ProceduralSkyMaterial:
    if _world_env == null or _world_env.environment == null:
        return null
    var sky: Sky = _world_env.environment.sky
    if sky == null:
        return null
    var mat = sky.sky_material
    if mat is ProceduralSkyMaterial:
        return mat
    return null


func _apply_day_mode() -> void:
    if _light != null:
        _light.light_energy = _DAY_LIGHT_ENERGY
        _light.light_color = _DAY_LIGHT_COLOR
        _light.shadow_blur = _DAY_SHADOW_BLUR
    if _world_env != null and _world_env.environment != null:
        _world_env.environment.ambient_light_color = _DAY_AMBIENT_COLOR
        _world_env.environment.ambient_light_energy = _DAY_AMBIENT_ENERGY
    var sky_mat: ProceduralSkyMaterial = _get_sky_material()
    if sky_mat != null:
        sky_mat.sky_top_color = _DAY_SKY_TOP
        sky_mat.sky_horizon_color = _DAY_SKY_HORIZON
        sky_mat.ground_bottom_color = _DAY_GROUND_BOTTOM
        sky_mat.ground_horizon_color = _DAY_GROUND_HORIZON


func _apply_night_mode() -> void:
    if _light != null:
        _light.light_energy = _NIGHT_LIGHT_ENERGY
        _light.light_color = _NIGHT_LIGHT_COLOR
        _light.shadow_blur = _NIGHT_SHADOW_BLUR
    if _world_env != null and _world_env.environment != null:
        _world_env.environment.ambient_light_color = _NIGHT_AMBIENT_COLOR
        _world_env.environment.ambient_light_energy = _NIGHT_AMBIENT_ENERGY
    var sky_mat: ProceduralSkyMaterial = _get_sky_material()
    if sky_mat != null:
        sky_mat.sky_top_color = _NIGHT_SKY_TOP
        sky_mat.sky_horizon_color = _NIGHT_SKY_HORIZON
        sky_mat.ground_bottom_color = _NIGHT_GROUND_BOTTOM
        sky_mat.ground_horizon_color = _NIGHT_GROUND_HORIZON
