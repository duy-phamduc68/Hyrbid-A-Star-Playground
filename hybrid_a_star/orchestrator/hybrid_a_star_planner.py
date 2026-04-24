from __future__ import annotations

import heapq
import math

import matplotlib.pyplot as plt
import numpy as np

from config_loader import Config
from rendering import draw_car_center, draw_static_scene, refresh_canvas, set_figure_fullscreen

from ..algorithms.analytic_paths import RS_AVAILABLE, generate_analytic_samples, get_analytic_cost
from ..core.environment import compute_dilated_overlay_cells, compute_h2d, precompute_environment
from ..core.geometry import center_to_rear_axle, collides_state, get_car_corners, obstacle_proximity_penalty, rear_axle_to_center
from ..core.kinematics import kinematic_expansion
from ..core.node import Node
from ..core.smoothing import smooth_path_gradient_descent


class HybridAStarPlanner:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.path_model = cfg.path_model
        self._skip_requested = False

    def run_all(self, scenarios: dict) -> None:
        scenario_names = list(scenarios.keys())
        print(f"Path model: {self.path_model}")
        if self.path_model == "RS" and not RS_AVAILABLE:
            print("RS selected but library is unavailable. Set planner.path_model: DUBINS in config.yml to proceed.")

        print("Available scenarios:")
        for i, name in enumerate(scenario_names):
            print(f"[{i}] {name}")

        scenario_plan = list(enumerate(scenario_names))

        for idx, scenario_name in scenario_plan:
            print(f"\n=== Scenario [{idx}] {scenario_name} ===")
            keep_running = self.run_scenario(scenario_name, scenarios[scenario_name])
            if not keep_running:
                break

    def run_scenario(self, scenario_name: str, scenario: dict) -> bool:
        print(f"Loading '{scenario_name}'. Precomputing environment...")
        self._skip_requested = False
        base_obs, wall_obs, rasterized_cont_obs, cont_obs_polys, dist_grid, wall_dist_grid = precompute_environment(self.cfg, scenario)

        start_cx, start_cy, start_yaw = scenario["start"]["x"], scenario["start"]["y"], scenario["start"]["theta"]
        goal_cx, goal_cy, goal_yaw = scenario["goal"]["x"], scenario["goal"]["y"], scenario["goal"]["theta"]

        s_rx, s_ry = center_to_rear_axle(self.cfg, start_cx, start_cy, start_yaw)
        g_rx, g_ry = center_to_rear_axle(self.cfg, goal_cx, goal_cy, goal_yaw)

        h2d_grid = compute_h2d(self.cfg, g_rx, g_ry, dist_grid)
        dilated_obs = compute_dilated_overlay_cells(base_obs, dist_grid, self.cfg.overlay_dilation_radius)

        plt.ion()
        fig, ax = plt.subplots(figsize=self.cfg.figure_size, dpi=self.cfg.figure_dpi)
        stop_requested = False

        def on_close(_event):
            nonlocal stop_requested
            stop_requested = True

        def on_key(event):
            # Space skips/terminates the current scenario run.
            if event.key in (" ", "space"):
                self._skip_requested = True
                print("Space pressed: skipping current scenario...")

        fig.canvas.mpl_connect("close_event", on_close)
        fig.canvas.mpl_connect("key_press_event", on_key)
        set_figure_fullscreen(fig)

        draw_static_scene(ax, self.cfg, scenario_name, h2d_grid, wall_obs, rasterized_cont_obs, dilated_obs, cont_obs_polys)

        _, start_arrow, _ = draw_car_center(
            ax,
            self.cfg,
            start_cx,
            start_cy,
            start_yaw,
            self.cfg.color_start,
            "A",
            self.cfg.start_goal_alpha,
        )
        draw_car_center(
            ax,
            self.cfg,
            goal_cx,
            goal_cy,
            goal_yaw,
            self.cfg.color_goal,
            "B",
            self.cfg.start_goal_alpha,
        )
        if not refresh_canvas(fig, self.cfg.delay_search):
            return False

        open_set = []
        visited_bins = {}
        counter = 0
        allowed_dirs = [1] if self.path_model == "DUBINS" else [1, -1]
        start_node = Node(s_rx, s_ry, start_yaw, 0.0, 0.0, [s_rx], [s_ry], [start_yaw], [1], 1, None)
        heapq.heappush(open_set, (0.0, counter, start_node))

        final_node = None
        dynamic_car_parts = []
        sniper_star = None
        expansion_count = 0

        print("Searching...")
        search_animation_enabled = self.cfg.enable_search_animation

        while open_set and not stop_requested and not self._skip_requested:
            if not plt.fignum_exists(fig.number):
                stop_requested = True
                break

            _, _, current = heapq.heappop(open_set)
            curr_key = (
                int(current.x // self.cfg.grid_size),
                int(current.y // self.cfg.grid_size),
                int(current.yaw // self.cfg.yaw_reso_deg),
            )
            best_known_g = visited_bins.get(curr_key)
            if best_known_g is not None and current.g > (best_known_g + self.cfg.tiny_epsilon):
                continue

            expansion_count += 1
            render_this_step = search_animation_enabled and (expansion_count % max(1, self.cfg.search_render_every) == 0)

            if current.parent and render_this_step:
                trail_color = self.cfg.color_trail_forward if current.direction == 1 else self.cfg.color_trail_reverse
                ax.plot(
                    current.path_x,
                    current.path_y,
                    color=trail_color,
                    linewidth=self.cfg.trail_linewidth,
                    alpha=self.cfg.trail_alpha,
                    zorder=self.cfg.trail_zorder,
                )

            if render_this_step:
                for part in dynamic_car_parts:
                    part.remove()
                dynamic_car_parts.clear()

                c_cx, c_cy = rear_axle_to_center(self.cfg, current.x, current.y, current.yaw)
                rect, arrow, _ = draw_car_center(
                    ax,
                    self.cfg,
                    c_cx,
                    c_cy,
                    current.yaw,
                    self.cfg.color_ghost,
                    "",
                    alpha=self.cfg.ghost_alpha,
                    draw_arrow=True,
                    arrow_alpha=self.cfg.ghost_arrow_alpha,
                )
                dynamic_car_parts.append(rect)
                if arrow:
                    dynamic_car_parts.append(arrow)

                if not refresh_canvas(fig, self.cfg.delay_search):
                    stop_requested = True
                    break

            if stop_requested or self._skip_requested:
                break

            if math.hypot(current.x - g_rx, current.y - g_ry) < self.cfg.sniper_trigger_radius:
                sniper_node = self._attempt_sniper_shot(current, g_rx, g_ry, goal_yaw, dist_grid, cont_obs_polys, base_obs)
                if sniper_node:
                    print("Sniper shot successful. Bypassing discrete search.")
                    star_cx, star_cy = rear_axle_to_center(self.cfg, current.x, current.y, current.yaw)
                    sniper_star = ax.plot(
                        [star_cx],
                        [star_cy],
                        marker="*",
                        color="yellow",
                        markersize=16,
                        markeredgecolor="black",
                        markeredgewidth=0.8,
                        linestyle="None",
                        zorder=max(self.cfg.zorder_car_text, self.cfg.zorder_snake_line) + 2,
                    )[0]
                    final_node = sniper_node
                    break

            if math.hypot(current.x - g_rx, current.y - g_ry) < self.cfg.tol_dist:
                yaw_err = abs((current.yaw - goal_yaw + 180) % 360 - 180)
                if yaw_err < self.cfg.tol_yaw:
                    print("Goal reached.")
                    final_node = current
                    break

            for direction in allowed_dirs:
                for steer_mul in self.cfg.steering_multipliers:
                    steer = float(steer_mul) * self.cfg.steer_max_deg
                    v = self.cfg.v_max * direction
                    px, py, pyaw = kinematic_expansion(self.cfg, current, v, steer)
                    nx, ny, nyaw = px[-1], py[-1], pyaw[-1]

                    if self._samples_collide(px, py, pyaw, dist_grid, base_obs, cont_obs_polys):
                        continue

                    bin_key = (
                        int(nx // self.cfg.grid_size),
                        int(ny // self.cfg.grid_size),
                        int(nyaw // self.cfg.yaw_reso_deg),
                    )

                    step_g = abs(v * self.cfg.dt)
                    if direction == -1:
                        step_g *= self.cfg.penalty_reverse
                    if direction != current.direction:
                        step_g += self.cfg.penalty_gear_shift
                    step_g += abs(steer) * self.cfg.penalty_steer
                    new_g = current.g + step_g

                    if bin_key in visited_bins and (visited_bins[bin_key] - new_g) < self.cfg.min_g_improvement:
                        continue

                    h2d = h2d_grid[int(ny // self.cfg.grid_size), int(nx // self.cfg.grid_size)]
                    if h2d == np.inf:
                        continue

                    h_cost = max(
                        h2d * self.cfg.grid_size,
                        get_analytic_cost(self.cfg, self.path_model, nx, ny, nyaw, g_rx, g_ry, goal_yaw),
                    )
                    h_cost += obstacle_proximity_penalty(self.cfg, nx, ny, nyaw, wall_dist_grid, cont_obs_polys)

                    f_cost = new_g + (h_cost * self.cfg.h_cost_weight)
                    visited_bins[bin_key] = new_g
                    counter += 1
                    heapq.heappush(
                        open_set,
                        (
                            f_cost,
                            counter,
                            Node(nx, ny, nyaw, new_g, f_cost, px, py, pyaw, [direction] * len(px), direction, current),
                        ),
                    )

        if self._skip_requested:
            print("Scenario skipped by user.")
            plt.ioff()
            try:
                plt.close(fig)
            except Exception:
                pass
            return True

        if stop_requested:
            print("Terminated early by user.")
            plt.ioff()
            try:
                plt.close(fig)
            except Exception:
                pass
            return False

        if final_node:
            self._animate_final_path(
                fig,
                ax,
                final_node,
                dynamic_car_parts,
                start_arrow,
                base_obs,
                cont_obs_polys,
                dist_grid,
            )
        else:
            if dynamic_car_parts:
                self._recolor_ghost_to_black(dynamic_car_parts)
                refresh_canvas(fig, self.cfg.delay_search)
            print("Failed to find path.")
            plt.ioff()
            plt.show()

        return True

    def _samples_collide(self, px, py, pyaw, dist_grid, base_obs, cont_obs_polys) -> bool:
        for s_rx, s_ry, s_yaw in zip(px, py, pyaw):
            # covert to center
            cx, cy = rear_axle_to_center(self.cfg, s_rx, s_ry, s_yaw)
        
            c = int(cx // self.cfg.grid_size)
            r = int(cy // self.cfg.grid_size)
        
            # bound check
            corners = get_car_corners(self.cfg, s_rx, s_ry, s_yaw)
            for x, y in corners:
                if x < 0 or x > self.cfg.width or y < 0 or y > self.cfg.height:
                    return True
        
            # grid bounds
            if not (0 <= c < self.cfg.cols and 0 <= r < self.cfg.rows):
                return True
        
            d = dist_grid[r, c]
        
            # guaranteed collision
            if d <= self.cfg.inscribed_rad:
                return True
        
            # guaranteed safe -> skip SAT
            if d >= self.cfg.circumscribed_rad:
                continue
            
            # uncertain -> SAT check (includes obstacle polygons + grid cells)
            if collides_state(self.cfg, s_rx, s_ry, s_yaw, base_obs, cont_obs_polys):
                return True
        return False

    def _recolor_ghost_to_black(self, dynamic_car_parts) -> None:
        for part in dynamic_car_parts:
            if hasattr(part, "set_edgecolor"):
                part.set_edgecolor("black")
            if hasattr(part, "set_facecolor"):
                part.set_facecolor("black")
            if hasattr(part, "set_color"):
                part.set_color("black")

    def _attempt_sniper_shot(self, current: Node, g_rx: float, g_ry: float, goal_yaw: float, dist_grid, cont_obs_polys, base_obs):
        result = generate_analytic_samples(
            self.cfg,
            self.path_model,
            current.x,
            current.y,
            current.yaw,
            g_rx,
            g_ry,
            goal_yaw,
        )
        if result is None:
            return None

        samples, total_path_len = result
        if not samples:
            return None

        px, py, pyaw, dirs = [], [], [], []
        for sx, sy, syaw, direction in samples:
            px.append(sx)
            py.append(sy)
            pyaw.append(syaw)
            dirs.append(direction)

        if self._samples_collide(px, py, pyaw, dist_grid, base_obs, cont_obs_polys):
            return None

        f_cost = current.g + total_path_len
        direction = dirs[-1] if dirs else 1
        return Node(px[-1], py[-1], pyaw[-1], f_cost, f_cost, px, py, pyaw, dirs, direction, current)

    def _animate_final_path(self, fig, ax, final_node, dynamic_car_parts, start_arrow, base_obs, cont_obs_polys, dist_grid):
        print("Animating final path snake...")

        if self._skip_requested:
            print("Scenario skipped by user.")
            plt.ioff()
            try:
                plt.close(fig)
            except Exception:
                pass
            return

        for part in dynamic_car_parts:
            part.remove()

        if not refresh_canvas(fig, self.cfg.delay_search):
            print("Terminated early by user.")
            plt.ioff()
            return

        path_nodes = []
        curr = final_node
        while curr:
            path_nodes.append(curr)
            curr = curr.parent
        path_nodes.reverse()

        raw_cx, raw_cy, raw_yaw, raw_dir = [], [], [], []
        for node in path_nodes:
            if not node.parent:
                continue
            node_dirs = node.path_dir if node.path_dir else ([node.direction] * len(node.path_x))
            for rx, ry, ryaw, sample_dir in zip(node.path_x, node.path_y, node.path_yaw, node_dirs):
                cx, cy = rear_axle_to_center(self.cfg, rx, ry, ryaw)
                raw_cx.append(cx)
                raw_cy.append(cy)
                raw_yaw.append(ryaw)
                raw_dir.append(sample_dir)

        if self.cfg.smoothing_enabled and len(raw_cx) >= self.cfg.smooth_min_points:
            smooth_cx, smooth_cy = smooth_path_gradient_descent(
                self.cfg,
                raw_cx,
                raw_cy,
                raw_yaw,
                raw_dir,
                base_obs,
                cont_obs_polys,
                dist_grid,
            )
            print("Applied gradient-descent post-smoothing.")
        else:
            smooth_cx, smooth_cy = raw_cx, raw_cy

        fwd_x, fwd_y = [], []
        rev_x, rev_y = [], []

        # final_path_speed > 1.0 speeds up playback, < 1.0 slows it down.
        final_path_speed = max(self.cfg.final_path_speed, self.cfg.tiny_epsilon)
        effective_snake_render_every = max(1, int(round(self.cfg.snake_render_every * final_path_speed)))
        effective_delay_snake = self.cfg.delay_snake / final_path_speed

        snake_idx = 0
        snake_line_fwd, = ax.plot(
            [],
            [],
            color=self.cfg.color_snake_line_forward,
            linewidth=self.cfg.snake_linewidth,
            zorder=self.cfg.zorder_snake_line,
        )
        snake_line_rev, = ax.plot(
            [],
            [],
            color=self.cfg.color_snake_reverse,
            linewidth=self.cfg.snake_linewidth,
            zorder=self.cfg.zorder_snake_line,
        )

        if not self.cfg.enable_final_path_animation:
            for i in range(len(smooth_cx)):
                cx, cy = smooth_cx[i], smooth_cy[i]
                ryaw = raw_yaw[i]
                direction = raw_dir[i]
                if direction == 1:
                    fwd_x.append(cx)
                    fwd_y.append(cy)
                    rev_x.append(np.nan)
                    rev_y.append(np.nan)
                else:
                    rev_x.append(cx)
                    rev_y.append(cy)
                    fwd_x.append(np.nan)
                    fwd_y.append(np.nan)

                # Keep ghost boxes visible in skip mode, just rendered in one pass.
                if i % max(1, self.cfg.snake_render_every) == 0:
                    draw_car_center(
                        ax,
                        self.cfg,
                        cx,
                        cy,
                        ryaw,
                        self.cfg.color_snake_forward,
                        "",
                        alpha=self.cfg.snake_body_alpha,
                        draw_arrow=False,
                    )

            snake_line_fwd.set_data(fwd_x, fwd_y)
            snake_line_rev.set_data(rev_x, rev_y)
            if not refresh_canvas(fig, self.cfg.delay_search):
                print("Terminated early by user.")
                plt.ioff()
                return

            plt.ioff()
            plt.show()
            return

        stop_requested = False
        for i in range(len(smooth_cx)):
            if stop_requested or self._skip_requested or not plt.fignum_exists(fig.number):
                stop_requested = True
                break

            snake_idx += 1
            cx, cy = smooth_cx[i], smooth_cy[i]
            ryaw = raw_yaw[i]
            direction = raw_dir[i]

            if direction == 1:
                fwd_x.append(cx)
                fwd_y.append(cy)
                rev_x.append(np.nan)
                rev_y.append(np.nan)
            else:
                rev_x.append(cx)
                rev_y.append(cy)
                fwd_x.append(np.nan)
                fwd_y.append(np.nan)

            if snake_idx % effective_snake_render_every == 0:
                draw_car_center(
                    ax,
                    self.cfg,
                    cx,
                    cy,
                    ryaw,
                    self.cfg.color_snake_forward,
                    "",
                    alpha=self.cfg.snake_body_alpha,
                    draw_arrow=False,
                )
                snake_line_fwd.set_data(fwd_x, fwd_y)
                snake_line_rev.set_data(rev_x, rev_y)
                if not refresh_canvas(fig, effective_delay_snake):
                    stop_requested = True
                    break

        if self._skip_requested:
            print("Scenario skipped by user.")
            plt.ioff()
            try:
                plt.close(fig)
            except Exception:
                pass
            return

        if stop_requested:
            print("Terminated early by user.")
            plt.ioff()
            try:
                plt.close(fig)
            except Exception:
                pass
            return

        plt.ioff()
        plt.show()
