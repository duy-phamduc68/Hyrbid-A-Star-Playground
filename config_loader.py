from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import math

import yaml


@dataclass(frozen=True)
class Config:
    path_model: str

    json_file: Path
    width: int
    height: int
    grid_size: int
    yaw_reso_deg: int

    car_l: float
    car_w: float
    wheel_base: float

    v_max: float
    steer_max_deg: float
    dt: float

    enable_search_animation: bool
    delay_search: float
    delay_snake: float
    enable_final_path_animation: bool
    final_path_speed: float
    search_render_every: int
    snake_render_every: int

    smoothing_enabled: bool
    smooth_min_points: int
    smooth_iters: int
    smooth_data_weight: float
    smooth_smoothness_weight: float
    smooth_obstacle_weight: float
    smooth_curvature_weight: float
    smooth_safety_buffer: float
    smooth_max_step: float

    h_cost_weight: float
    min_g_improvement: float
    sniper_trigger_radius: float

    steering_multipliers: tuple[float, ...]

    kinematic_substeps: int
    analytic_step_px: float
    analytic_min_chunks: int

    angle_wrap_deg: float
    tiny_epsilon: float

    collision_dilation_radius: float
    overlay_dilation_radius: float

    penalty_reverse: float
    penalty_gear_shift: float
    penalty_steer: float

    prox_penalty_weight: float
    prox_wall_buffer: float
    prox_car_buffer: float

    tol_dist: float
    tol_yaw: float

    color_start: str
    color_goal: str
    color_ghost: str
    color_trail_forward: str
    color_trail_reverse: str
    color_snake_forward: str
    color_snake_reverse: str
    color_snake_line_forward: str
    color_bg: str
    base_obstacle_color: str
    rasterized_cont_obstacle_color: str
    rasterized_cont_obstacle_alpha: float
    dilation_color: str
    cont_obstacle_color: str
    h2d_cmap: str
    h2d_interpolation: str
    car_text_color: str

    figure_size: tuple[float, float]
    figure_dpi: int
    title_color: str

    start_goal_alpha: float
    trail_alpha: float
    trail_linewidth: float
    trail_zorder: int
    ghost_alpha: float
    ghost_arrow_alpha: float
    snake_body_alpha: float
    snake_linewidth: float
    bad_cell_alpha: float

    arrow_head_width: float
    arrow_head_length: float
    car_linewidth: float
    cell_linewidth: float

    zorder_heatmap: int
    zorder_base_obstacle: int
    zorder_dilation: int
    zorder_cont_obstacle: int
    zorder_snake_line: int
    zorder_car_rect: int
    zorder_car_arrow: int
    zorder_car_text: int

    h2d_alpha: float
    dilation_alpha: float
    cont_obstacle_alpha: float

    @property
    def cols(self) -> int:
        return self.width // self.grid_size

    @property
    def rows(self) -> int:
        return self.height // self.grid_size

    @property
    def rear_overhang(self) -> float:
        return (self.car_l - self.wheel_base) / 2.0

    @property
    def inscribed_rad(self) -> float:
        return self.car_w / 2.0

    @property
    def circumscribed_rad(self) -> float:
        return math.hypot(self.car_l / 2.0, self.car_w / 2.0)

    @property
    def min_turn_rad(self) -> float:
        return self.wheel_base / math.tan(math.radians(self.steer_max_deg))



def load_config(config_path: Path) -> Config:
    with config_path.open("r", encoding="utf-8") as f:
        raw = yaml.safe_load(f)

    cfg_dir = config_path.parent

    planner = raw["planner"]
    env = raw["environment"]
    vehicle = raw["vehicle"]
    kinematics = raw["kinematics"]
    animation = raw["animation"]
    smoothing = raw["smoothing"]
    search = raw["search"]
    steering = raw["steering"]
    sampling = raw["sampling"]
    numeric = raw["numeric"]
    dilation = raw["dilation"]
    penalties = raw["penalties"]
    proximity = raw["proximity"]
    goal_tol = raw["goal_tolerance"]
    render = raw["render"]

    json_file = (cfg_dir / env["json_file"]).resolve()

    return Config(
        path_model=str(planner["path_model"]).upper(),
        json_file=json_file,
        width=int(env["width"]),
        height=int(env["height"]),
        grid_size=int(env["grid_size"]),
        yaw_reso_deg=int(env["yaw_resolution_deg"]),
        car_l=float(vehicle["length"]),
        car_w=float(vehicle["width"]),
        wheel_base=float(vehicle["wheel_base"]),
        v_max=float(kinematics["v_max"]),
        steer_max_deg=float(kinematics["steer_max_deg"]),
        dt=float(kinematics["dt"]),
        enable_search_animation=bool(animation["enable_search_animation"]),
        delay_search=float(animation["delay_search"]),
        delay_snake=float(animation["delay_snake"]),
        enable_final_path_animation=bool(animation["enable_final_path_animation"]),
        final_path_speed=float(animation["final_path_speed"]),
        search_render_every=int(animation["search_render_every"]),
        snake_render_every=int(animation["snake_render_every"]),
        smoothing_enabled=bool(smoothing["enabled"]),
        smooth_min_points=int(smoothing["min_points"]),
        smooth_iters=int(smoothing["iterations"]),
        smooth_data_weight=float(smoothing["data_weight"]),
        smooth_smoothness_weight=float(smoothing["smoothness_weight"]),
        smooth_obstacle_weight=float(smoothing["obstacle_weight"]),
        smooth_curvature_weight=float(smoothing["curvature_weight"]),
        smooth_safety_buffer=float(smoothing["safety_buffer"]),
        smooth_max_step=float(smoothing["max_step"]),
        h_cost_weight=float(search["h_cost_weight"]),
        min_g_improvement=float(search["min_g_improvement"]),
        sniper_trigger_radius=float(search["sniper_trigger_radius"]),
        steering_multipliers=tuple(float(x) for x in steering["multipliers"]),
        kinematic_substeps=int(sampling["kinematic_substeps"]),
        analytic_step_px=float(sampling["analytic_step_px"]),
        analytic_min_chunks=int(sampling["min_chunks"]),
        angle_wrap_deg=float(numeric["angle_wrap_deg"]),
        tiny_epsilon=float(numeric["tiny_epsilon"]),
        collision_dilation_radius=float(dilation["collision_radius"]),
        overlay_dilation_radius=float(dilation["overlay_radius"]),
        penalty_reverse=float(penalties["reverse"]),
        penalty_gear_shift=float(penalties["gear_shift"]),
        penalty_steer=float(penalties["steer"]),
        prox_penalty_weight=float(proximity["penalty_weight"]),
        prox_wall_buffer=float(proximity["wall_buffer"]),
        prox_car_buffer=float(proximity["car_buffer"]),
        tol_dist=float(goal_tol["distance"]),
        tol_yaw=float(goal_tol["yaw_deg"]),
        color_start=str(render["color_start"]),
        color_goal=str(render["color_goal"]),
        color_ghost=str(render["color_ghost"]),
        color_trail_forward=str(render["color_trail_forward"]),
        color_trail_reverse=str(render["color_trail_reverse"]),
        color_snake_forward=str(render["color_snake_forward"]),
        color_snake_reverse=str(render["color_snake_reverse"]),
        color_snake_line_forward=str(render["color_snake_line_forward"]),
        color_bg=str(render["color_background"]),
        base_obstacle_color=str(render["base_obstacle_color"]),
        rasterized_cont_obstacle_color=str(render["rasterized_cont_obstacle_color"]),
        rasterized_cont_obstacle_alpha=float(render["rasterized_cont_obstacle_alpha"]),
        dilation_color=str(render["dilation_color"]),
        cont_obstacle_color=str(render["cont_obstacle_color"]),
        h2d_cmap=str(render["h2d_cmap"]),
        h2d_interpolation=str(render["h2d_interpolation"]),
        car_text_color=str(render["car_text_color"]),
        figure_size=(
            float(render["figure_size"][0]),
            float(render["figure_size"][1]),
        ),
        figure_dpi=int(render["figure_dpi"]),
        title_color=str(render["title_color"]),
        start_goal_alpha=float(render["start_goal_alpha"]),
        trail_alpha=float(render["trail_alpha"]),
        trail_linewidth=float(render["trail_linewidth"]),
        trail_zorder=int(render["trail_zorder"]),
        ghost_alpha=float(render["ghost_alpha"]),
        ghost_arrow_alpha=float(render["ghost_arrow_alpha"]),
        snake_body_alpha=float(render["snake_body_alpha"]),
        snake_linewidth=float(render["snake_linewidth"]),
        bad_cell_alpha=float(render["bad_cell_alpha"]),
        arrow_head_width=float(render["arrow_head_width"]),
        arrow_head_length=float(render["arrow_head_length"]),
        car_linewidth=float(render["car_linewidth"]),
        cell_linewidth=float(render["cell_linewidth"]),
        zorder_heatmap=int(render["zorder_heatmap"]),
        zorder_base_obstacle=int(render["zorder_base_obstacle"]),
        zorder_dilation=int(render["zorder_dilation"]),
        zorder_cont_obstacle=int(render["zorder_cont_obstacle"]),
        zorder_snake_line=int(render["zorder_snake_line"]),
        zorder_car_rect=int(render["zorder_car_rect"]),
        zorder_car_arrow=int(render["zorder_car_arrow"]),
        zorder_car_text=int(render["zorder_car_text"]),
        h2d_alpha=float(render["h2d_alpha"]),
        dilation_alpha=float(render["dilation_alpha"]),
        cont_obstacle_alpha=float(render["cont_obstacle_alpha"]),
    )
