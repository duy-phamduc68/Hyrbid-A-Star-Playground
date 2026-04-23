from __future__ import annotations

import math

import matplotlib.patches as patches
import matplotlib.pyplot as plt
import matplotlib.transforms as transforms
import numpy as np

from config_loader import Config


def set_figure_fullscreen(fig) -> None:
    manager = plt.get_current_fig_manager()
    try:
        if hasattr(manager, "full_screen_toggle"):
            manager.full_screen_toggle()
            return
    except Exception:
        pass

    try:
        win = getattr(manager, "window", None)
        if win is not None:
            if hasattr(win, "showMaximized"):
                win.showMaximized()
                return
            if hasattr(win, "state"):
                win.state("zoomed")
                return
            if hasattr(win, "Maximize"):
                win.Maximize(True)
                return
    except Exception:
        pass


def refresh_canvas(fig, delay: float) -> bool:
    if not plt.fignum_exists(fig.number):
        return False
    fig.canvas.draw_idle()
    fig.canvas.flush_events()
    if delay > 0.0:
        plt.pause(delay)
    return plt.fignum_exists(fig.number)


def draw_car_center(ax, cfg: Config, cx: float, cy: float, theta: float, color: str, label: str, alpha: float = 1.0, draw_arrow: bool = True, arrow_alpha: float | None = None):
    if arrow_alpha is None:
        arrow_alpha = alpha

    bottom_left = (cx - cfg.car_l / 2, cy - cfg.car_w / 2)
    rect = patches.Rectangle(
        bottom_left,
        cfg.car_l,
        cfg.car_w,
        linewidth=cfg.car_linewidth,
        edgecolor=color,
        facecolor=color,
        alpha=alpha,
        zorder=cfg.zorder_car_rect,
    )
    tr = transforms.Affine2D().rotate_deg_around(cx, cy, theta)
    rect.set_transform(tr + ax.transData)
    ax.add_patch(rect)

    arrow = None
    if draw_arrow:
        rad = math.radians(theta)
        arrow_len = cfg.car_l / 2
        arrow = ax.arrow(
            cx,
            cy,
            arrow_len * math.cos(rad),
            arrow_len * math.sin(rad),
            head_width=cfg.arrow_head_width,
            head_length=cfg.arrow_head_length,
            fc=color,
            ec=color,
            linewidth=cfg.car_linewidth,
            alpha=arrow_alpha,
            zorder=cfg.zorder_car_arrow,
        )

    text = None
    if label:
        text = ax.text(
            cx,
            cy,
            label,
            color=cfg.car_text_color,
            weight="bold",
            ha="center",
            va="center",
            alpha=alpha,
            zorder=cfg.zorder_car_text,
        )

    return rect, arrow, text


def draw_static_scene(
    ax,
    cfg: Config,
    scenario_name: str,
    h2d_grid,
    wall_obs: set[tuple[int, int]],
    rasterized_cont_obs: set[tuple[int, int]],
    dilated_obs: set[tuple[int, int]],
    cont_obs_polys: list[list[tuple[float, float]]],
):
    ax.set_facecolor(cfg.color_bg)
    ax.set_title(f"Hybrid A* v1 | Scenario: {scenario_name}", color=cfg.title_color)
    ax.set_xlim(0, cfg.width)
    ax.set_ylim(0, cfg.height)
    ax.invert_yaxis()
    ax.set_aspect("equal", adjustable="box")

    masked_h2d = np.ma.masked_invalid(h2d_grid)
    cmap = plt.get_cmap(cfg.h2d_cmap)
    cmap.set_bad(color=cfg.color_bg, alpha=cfg.bad_cell_alpha)
    ax.imshow(
        masked_h2d,
        extent=[0, cfg.width, cfg.height, 0],
        cmap=cmap,
        alpha=cfg.h2d_alpha,
        interpolation=cfg.h2d_interpolation,
        zorder=cfg.zorder_heatmap,
    )

    for c, r in wall_obs:
        ax.add_patch(
            patches.Rectangle(
                (c * cfg.grid_size, r * cfg.grid_size),
                cfg.grid_size,
                cfg.grid_size,
                linewidth=cfg.cell_linewidth,
                facecolor=cfg.base_obstacle_color,
                zorder=cfg.zorder_base_obstacle,
            )
        )

    for c, r in rasterized_cont_obs:
        ax.add_patch(
            patches.Rectangle(
                (c * cfg.grid_size, r * cfg.grid_size),
                cfg.grid_size,
                cfg.grid_size,
                linewidth=cfg.cell_linewidth,
                facecolor=cfg.rasterized_cont_obstacle_color,
                alpha=cfg.rasterized_cont_obstacle_alpha,
                zorder=cfg.zorder_base_obstacle,
            )
        )

    for c, r in dilated_obs:
        ax.add_patch(
            patches.Rectangle(
                (c * cfg.grid_size, r * cfg.grid_size),
                cfg.grid_size,
                cfg.grid_size,
                linewidth=cfg.cell_linewidth,
                facecolor=cfg.dilation_color,
                alpha=cfg.dilation_alpha,
                zorder=cfg.zorder_dilation,
            )
        )

    for poly in cont_obs_polys:
        ax.add_patch(
            patches.Polygon(
                poly,
                closed=True,
                facecolor=cfg.cont_obstacle_color,
                alpha=cfg.cont_obstacle_alpha,
                zorder=cfg.zorder_cont_obstacle,
            )
        )
