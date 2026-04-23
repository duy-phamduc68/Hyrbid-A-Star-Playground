from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import pygame  # type: ignore

from config_loader import load_config
from hybrid_a_star import HybridAStarPlanner


CARD_W = 300
CARD_H = 340
CARD_MARGIN = 30

BG_COLOR = (20, 20, 20)
CARD_COLOR = (50, 50, 50)
CARD_BORDER = (230, 230, 230)
PREVIEW_BG = (30, 30, 30)
TEXT_COLOR = (245, 245, 245)
MUTED_TEXT = (170, 170, 170)
GRID_COLOR = (100, 100, 100)
START_COLOR = (0, 255, 255)
GOAL_COLOR = (255, 165, 0)
OBS_COLOR = (255, 50, 50)


def parse_args():
    parser = argparse.ArgumentParser(description="Hybrid A* v1 (modular)")
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).resolve().parent / "config.yml",
        help="Path to YAML config file",
    )
    return parser.parse_args()


class ScenarioPicker:
    def __init__(self, scenarios: dict, grid_size: int, width: int, height: int, path_model: str):
        self.scenarios = scenarios
        self.grid_size = grid_size
        self.width = width
        self.height = height
        self.path_model = path_model

        pygame.init()
        display_info = pygame.display.Info()
        self.window_w = display_info.current_w
        self.window_h = display_info.current_h
        self.window = pygame.display.set_mode((self.window_w, self.window_h), pygame.FULLSCREEN)
        pygame.display.set_caption("Scenario Picker")

        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont(None, 28)
        self.ui_font = pygame.font.SysFont(None, 22)
        self.scroll_y = 0

    def _draw_mini_preview(self, surface, data, rect) -> None:
        sx = rect.width / self.width
        sy = rect.height / self.height

        for col, row in data.get("grid_obs", []):
            x = rect.x + (col * self.grid_size) * sx
            y = rect.y + (row * self.grid_size) * sy
            w = max(1, self.grid_size * sx)
            h = max(1, self.grid_size * sy)
            pygame.draw.rect(surface, GRID_COLOR, (x, y, w, h))

        for obs in data.get("cont_obs", []):
            x = rect.x + obs["x"] * sx
            y = rect.y + obs["y"] * sy
            pygame.draw.circle(surface, OBS_COLOR, (int(x), int(y)), 3)

        start = data.get("start")
        if start:
            x = rect.x + start["x"] * sx
            y = rect.y + start["y"] * sy
            pygame.draw.circle(surface, START_COLOR, (int(x), int(y)), 4)

        goal = data.get("goal")
        if goal:
            x = rect.x + goal["x"] * sx
            y = rect.y + goal["y"] * sy
            pygame.draw.circle(surface, GOAL_COLOR, (int(x), int(y)), 4)

    def pick(self) -> str | None:
        start_x = 40
        start_y = 100
        card_rects = []

        while True:
            mx, my = pygame.mouse.get_pos()
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return None
                if event.type == pygame.KEYDOWN and event.key in (pygame.K_ESCAPE, pygame.K_q):
                    return None
                if event.type == pygame.MOUSEWHEEL:
                    self.scroll_y += event.y * 40
                if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                    for name, rect in reversed(card_rects):
                        if rect.collidepoint(mx, my):
                            return name

            self.window.fill(BG_COLOR)

            title = self.font.render("Pick Scenario To Run", True, TEXT_COLOR)
            subtitle = self.ui_font.render(
                f"Path model: {self.path_model} | Click a card to launch matplotlib animation | ESC/Q to quit",
                True,
                MUTED_TEXT,
            )
            hint = self.ui_font.render(
                "In matplotlib: press Space to skip/terminate early and return here.",
                True,
                MUTED_TEXT,
            )
            self.window.blit(title, (40, 30))
            self.window.blit(subtitle, (40, 60))
            self.window.blit(hint, (40, 82))

            cols = max(1, (self.window_w - 2 * start_x) // (CARD_W + CARD_MARGIN))
            keys = list(self.scenarios.keys())
            rows = max(1, (len(keys) + cols - 1) // cols)
            content_h = rows * CARD_H + max(0, rows - 1) * CARD_MARGIN
            viewport_h = self.window_h - start_y - 30
            min_scroll = min(0, viewport_h - content_h)
            self.scroll_y = max(min_scroll, min(0, self.scroll_y))
            viewport = pygame.Rect(0, start_y, self.window_w, viewport_h)

            card_rects = []
            for idx, name in enumerate(keys):
                row = idx // cols
                col = idx % cols
                cx = start_x + col * (CARD_W + CARD_MARGIN)
                cy = start_y + self.scroll_y + row * (CARD_H + CARD_MARGIN)
                card = pygame.Rect(cx, cy, CARD_W, CARD_H)
                if not card.colliderect(viewport):
                    continue

                highlight = card.collidepoint(mx, my)
                pygame.draw.rect(self.window, CARD_COLOR, card)
                pygame.draw.rect(self.window, CARD_BORDER, card, 2 if highlight else 1)

                preview = pygame.Rect(cx + 10, cy + 10, CARD_W - 20, CARD_H - 60)
                pygame.draw.rect(self.window, PREVIEW_BG, preview)
                self._draw_mini_preview(self.window, self.scenarios[name], preview)

                name_surf = self.font.render(name, True, TEXT_COLOR)
                self.window.blit(name_surf, (cx + 10, cy + CARD_H - 40))
                card_rects.append((name, card))

            pygame.display.flip()
            self.clock.tick(60)

    def close(self) -> None:
        pygame.quit()


def main() -> None:
    args = parse_args()
    cfg = load_config(args.config.resolve())

    with cfg.json_file.open("r", encoding="utf-8") as f:
        scenarios = json.load(f)

    if not scenarios:
        print("No scenarios found in scenario JSON file.")
        return

    planner = HybridAStarPlanner(cfg)

    while True:
        picker = ScenarioPicker(scenarios, cfg.grid_size, cfg.width, cfg.height, cfg.path_model)
        selected_name = None
        try:
            selected_name = picker.pick()
        finally:
            picker.close()

        if selected_name is None:
            break

        print(f"\n=== Scenario {selected_name} ===")
        planner.run_scenario(selected_name, scenarios[selected_name])

        # Reset matplotlib state before returning to pygame menu.
        plt.ioff()
        try:
            plt.close("all")
        except Exception:
            pass


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Terminated early by user.")
        plt.ioff()
        try:
            plt.close("all")
        except Exception:
            pass
