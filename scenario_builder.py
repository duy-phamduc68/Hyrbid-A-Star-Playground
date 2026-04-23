import pygame # type: ignore
import json
import os
import math

# --- Configuration ---
PX_PER_METER = 10
GRID_SIZE = 5
GRID_WIDTH, GRID_HEIGHT = 500, 500  # The strict physical bounds of the scenario
CAR_L, CAR_W = 50, 20  # Length (X-axis), Width (Y-axis)
JSON_FILE = "scenario.json"

# Colors
BG_COLOR = (30, 30, 30)
GRID_BG = (15, 15, 15)
GRID_COLOR = (100, 100, 100)
AQUA = (0, 255, 255)
ORANGE = (255, 165, 0)
RED = (255, 50, 50)
WHITE = (255, 255, 255)
YELLOW = (255, 230, 0)
GREEN = (80, 200, 120)
POPUP_BG = (20, 20, 20)
POPUP_BORDER = (220, 220, 220)
ERROR_COLOR = (255, 110, 110)

class ScenarioBuilder:
    def __init__(self):
        pygame.init()
        display_info = pygame.display.Info()
        self.window_w = display_info.current_w
        self.window_h = display_info.current_h
        
        # Setup native resolution window
        self.window = pygame.display.set_mode((self.window_w, self.window_h), pygame.FULLSCREEN)
        pygame.display.set_caption("Hybrid A* Scenario Builder")
        
        # Offsets to center the 500x500 grid on a fullscreen monitor
        self.ui_height = 150
        self.grid_offset_x = (self.window_w - GRID_WIDTH) // 2
        self.grid_offset_y = max(10, (self.window_h - self.ui_height - GRID_HEIGHT) // 2)

        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont(None, 24)
        self.ui_font = pygame.font.SysFont(None, 20)

        # App State
        self.app_state = "HOME" # "HOME" or "EDITOR"
        self.scenarios = self.load_scenarios_data()
        self.home_click_zones = []
        self.home_scroll = 0

        # Current Scenario Data
        self.current_scenario_name = ""
        self.start_pose = None  # {'x':, 'y':, 'theta':}
        self.goal_pose = None
        self.grid_obstacles = set() # set of (col, row)
        self.cont_obstacles = []    # list of {'x':, 'y':, 'theta':}

        # Editor State
        self.mode = "PLACE_START"
        self.current_theta = 0.0 # Degrees
        self.grid_draw_mode = "FREE"  # FREE or LINE
        self.grid_drag_action = None   # ADD or REMOVE
        self.line_start_cell = None    # (col, row) for LINE mode

        # Popup and status state
        self.prompt_active = False
        self.prompt_action = None  # save
        self.prompt_text = ""
        self.prompt_title = ""
        self.prompt_hint = ""
        self.status_message = "Ready"
        self.status_color = WHITE

    def mouse_in_grid(self, lx, ly):
        """Check if logical coordinates are within the 500x500 bounds"""
        return 0 <= lx < GRID_WIDTH and 0 <= ly < GRID_HEIGHT

    def set_status(self, message, color=WHITE):
        self.status_message = message
        self.status_color = color

    def load_scenarios_data(self):
        if not os.path.exists(JSON_FILE):
            return {}
        with open(JSON_FILE, 'r') as f:
            try:
                data = json.load(f)
                return data if isinstance(data, dict) else {}
            except json.JSONDecodeError:
                return {}

    def save_scenarios_data(self, data):
        with open(JSON_FILE, 'w') as f:
            json.dump(data, f, indent=4)
        self.scenarios = data

    def swap_scenarios(self, name, direction):
        keys = list(self.scenarios.keys())
        idx = keys.index(name)
        target_idx = idx + direction
        
        if 0 <= target_idx < len(keys):
            keys[idx], keys[target_idx] = keys[target_idx], keys[idx]
            new_dict = {k: self.scenarios[k] for k in keys}
            self.save_scenarios_data(new_dict)

    def load_scenario_to_editor(self, name):
        self.current_scenario_name = name
        if name and name in self.scenarios:
            scenario = self.scenarios[name]
            self.start_pose = scenario.get("start")
            self.goal_pose = scenario.get("goal")
            self.grid_obstacles = set(tuple(x) for x in scenario.get("grid_obs", []))
            self.cont_obstacles = scenario.get("cont_obs", [])
            self.set_status(f"Loaded '{name}'", GREEN)
        else:
            self.current_scenario_name = ""
            self.start_pose = None
            self.goal_pose = None
            self.grid_obstacles.clear()
            self.cont_obstacles.clear()
            self.set_status("Started New Scenario", GREEN)
        
        self.line_start_cell = None
        self.app_state = "EDITOR"

    def open_name_prompt(self, action):
        self.prompt_active = True
        self.prompt_action = action
        self.prompt_text = self.current_scenario_name if self.current_scenario_name else ""

        if action == "save":
            self.prompt_title = "Save Scenario: type name and press Enter"
            self.refresh_prompt_hint()

    def refresh_prompt_hint(self):
        if self.prompt_action != "save":
            return
        typed = self.prompt_text.strip()
        if not typed:
            self.prompt_hint = "Type a scenario name."
        elif typed in self.scenarios:
            self.prompt_hint = "Name already exists. Saving will overwrite it."
        else:
            self.prompt_hint = "New name. Press Enter to save."

    def close_name_prompt(self):
        self.prompt_active = False
        self.prompt_action = None
        self.prompt_text = ""
        self.prompt_title = ""
        self.prompt_hint = ""

    def submit_name_prompt(self):
        name = self.prompt_text.strip()
        if not name:
            self.set_status("Scenario name is required.", ERROR_COLOR)
            return

        if self.prompt_action == "save":
            existed = name in self.scenarios
            self.scenarios[name] = {
                "start": self.start_pose,
                "goal": self.goal_pose,
                "grid_obs": list(self.grid_obstacles),
                "cont_obs": self.cont_obstacles
            }
            self.save_scenarios_data(self.scenarios)
            self.current_scenario_name = name
            if existed:
                self.set_status(f"Updated existing scenario '{name}'", GREEN)
            else:
                self.set_status(f"Saved '{name}'", GREEN)
            self.close_name_prompt()

    def draw_car(self, x, y, theta, color, label):
        """Draws a car using Logical Coordinates (x,y) mapped to Screen Coordinates"""
        car_surf = pygame.Surface((CAR_L, CAR_W), pygame.SRCALPHA)
        car_surf.fill(color)
        
        text = self.font.render(label, True, (0, 0, 0))
        txt_rect = text.get_rect(center=(CAR_L//2, CAR_W//2))
        car_surf.blit(text, txt_rect)
        
        pygame.draw.polygon(car_surf, (0, 0, 0), [
            (CAR_L - 10, CAR_W//2 - 5), 
            (CAR_L - 10, CAR_W//2 + 5), 
            (CAR_L - 2, CAR_W//2)
        ])

        rotated_surf = pygame.transform.rotate(car_surf, -theta)
        
        # Translate logical 500x500 coordinates to actual screen position
        screen_x = x + self.grid_offset_x
        screen_y = y + self.grid_offset_y
        
        rect = rotated_surf.get_rect(center=(screen_x, screen_y))
        self.window.blit(rotated_surf, rect)

    def vehicle_corners(self, pose):
        cx, cy = pose['x'], pose['y']
        theta = math.radians(pose['theta'])
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)

        half_l = CAR_L / 2
        half_w = CAR_W / 2
        local = [
            (-half_l, -half_w), (half_l, -half_w),
            (half_l, half_w), (-half_l, half_w),
        ]

        corners = []
        for lx, ly in local:
            wx = cx + (lx * cos_t) - (ly * sin_t)
            wy = cy + (lx * sin_t) + (ly * cos_t)
            corners.append((wx, wy))
        return corners

    def polygon_axes(self, polygon):
        axes = []
        for i in range(len(polygon)):
            x1, y1 = polygon[i]
            x2, y2 = polygon[(i + 1) % len(polygon)]
            edge_x = x2 - x1
            edge_y = y2 - y1
            axis_x = -edge_y
            axis_y = edge_x
            length = math.hypot(axis_x, axis_y)
            if length > 0:
                axes.append((axis_x / length, axis_y / length))
        return axes

    def project_polygon(self, polygon, axis):
        ax, ay = axis
        projections = [(px * ax) + (py * ay) for px, py in polygon]
        return min(projections), max(projections)

    def sat_intersects(self, poly_a, poly_b):
        axes = self.polygon_axes(poly_a) + self.polygon_axes(poly_b)
        for axis in axes:
            min_a, max_a = self.project_polygon(poly_a, axis)
            min_b, max_b = self.project_polygon(poly_b, axis)
            if max_a < min_b or max_b < min_a:
                return False
        return True

    def corners_in_bounds(self, corners):
        for x, y in corners:
            if x < 0 or x > GRID_WIDTH or y < 0 or y > GRID_HEIGHT:
                return False
        return True

    def wall_cells_near_polygon(self, polygon):
        xs = [p[0] for p in polygon]
        ys = [p[1] for p in polygon]
        min_col = max(0, int(min(xs) // GRID_SIZE) - 1)
        max_col = min((GRID_WIDTH // GRID_SIZE) - 1, int(max(xs) // GRID_SIZE) + 1)
        min_row = max(0, int(min(ys) // GRID_SIZE) - 1)
        max_row = min((GRID_HEIGHT // GRID_SIZE) - 1, int(max(ys) // GRID_SIZE) + 1)

        near = []
        for col in range(min_col, max_col + 1):
            for row in range(min_row, max_row + 1):
                if (col, row) in self.grid_obstacles:
                    near.append((col, row))
        return near

    def wall_cell_polygon(self, col, row):
        x0 = col * GRID_SIZE
        y0 = row * GRID_SIZE
        x1 = x0 + GRID_SIZE
        y1 = y0 + GRID_SIZE
        return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]

    def can_place_vehicle(self, candidate_pose, role, candidate_index=None):
        candidate_poly = self.vehicle_corners(candidate_pose)

        if not self.corners_in_bounds(candidate_poly):
            return False, "Vehicle is out of bounds for this heading."

        for col, row in self.wall_cells_near_polygon(candidate_poly):
            wall_poly = self.wall_cell_polygon(col, row)
            if self.sat_intersects(candidate_poly, wall_poly):
                return False, "Vehicle collides with a grid wall."

        if role == "start":
            for obs in self.cont_obstacles:
                if self.sat_intersects(candidate_poly, self.vehicle_corners(obs)):
                    return False, "Start collides with obstacle vehicle."
        elif role == "goal":
            for obs in self.cont_obstacles:
                if self.sat_intersects(candidate_poly, self.vehicle_corners(obs)):
                    return False, "Goal collides with obstacle vehicle."
        elif role == "obstacle":
            if self.start_pose and self.sat_intersects(candidate_poly, self.vehicle_corners(self.start_pose)):
                return False, "Obstacle collides with start vehicle."
            if self.goal_pose and self.sat_intersects(candidate_poly, self.vehicle_corners(self.goal_pose)):
                return False, "Obstacle collides with goal vehicle."
            for i, obs in enumerate(self.cont_obstacles):
                if candidate_index is not None and i == candidate_index: continue
                if self.sat_intersects(candidate_poly, self.vehicle_corners(obs)):
                    return False, "Obstacle collides with another obstacle."

        return True, ""

    def obstacle_index_at_point(self, x, y):
        for i in range(len(self.cont_obstacles) - 1, -1, -1):
            obs = self.cont_obstacles[i]
            dx = x - obs['x']
            dy = y - obs['y']
            theta = math.radians(obs['theta'])
            cos_t = math.cos(theta)
            sin_t = math.sin(theta)
            local_x = dx * cos_t + dy * sin_t
            local_y = -dx * sin_t + dy * cos_t
            if abs(local_x) <= (CAR_L / 2) and abs(local_y) <= (CAR_W / 2):
                return i
        return None

    def can_add_wall_cell(self, cell):
        col, row = cell
        wall_poly = self.wall_cell_polygon(col, row)
        if self.start_pose and self.sat_intersects(wall_poly, self.vehicle_corners(self.start_pose)):
            return False, "Wall collides with start vehicle."
        if self.goal_pose and self.sat_intersects(wall_poly, self.vehicle_corners(self.goal_pose)):
            return False, "Wall collides with goal vehicle."
        for obs in self.cont_obstacles:
            if self.sat_intersects(wall_poly, self.vehicle_corners(obs)):
                return False, "Wall collides with obstacle vehicle."
        return True, ""

    def draw_mini_preview(self, surface, data, rect):
        scale_x = rect.width / GRID_WIDTH
        scale_y = rect.height / GRID_HEIGHT
        
        # Draw Grid Obstacles
        for col, row in data.get("grid_obs", []):
            x = rect.x + (col * GRID_SIZE) * scale_x
            y = rect.y + (row * GRID_SIZE) * scale_y
            w = max(1, GRID_SIZE * scale_x)
            h = max(1, GRID_SIZE * scale_y)
            pygame.draw.rect(surface, GRID_COLOR, (x, y, w, h))
            
        # Draw Cont Obstacles
        for obs in data.get("cont_obs", []):
            x = rect.x + obs['x'] * scale_x
            y = rect.y + obs['y'] * scale_y
            pygame.draw.circle(surface, RED, (int(x), int(y)), 3)
            
        # Draw Start / Goal
        if data.get("start"):
            x = rect.x + data["start"]['x'] * scale_x
            y = rect.y + data["start"]['y'] * scale_y
            pygame.draw.circle(surface, AQUA, (int(x), int(y)), 4)
            
        if data.get("goal"):
            x = rect.x + data["goal"]['x'] * scale_x
            y = rect.y + data["goal"]['y'] * scale_y
            pygame.draw.circle(surface, ORANGE, (int(x), int(y)), 4)

    def run(self):
        running = True
        while running:
            mx, my = pygame.mouse.get_pos()
            
            # Map screen mouse coordinates to 500x500 logical coordinates
            lx = mx - self.grid_offset_x
            ly = my - self.grid_offset_y
            
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

                # -------------------------
                # HOME SCREEN EVENTS
                # -------------------------
                if self.app_state == "HOME":
                    if event.type == pygame.KEYDOWN and event.key in (pygame.K_ESCAPE, pygame.K_q):
                        running = False
                    if event.type == pygame.MOUSEWHEEL:
                        self.home_scroll += event.y * 40
                    if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                        for zone in reversed(self.home_click_zones):
                            if zone["rect"].collidepoint(mx, my):
                                if zone["type"] == "create":
                                    self.load_scenario_to_editor(None)
                                elif zone["type"] == "load":
                                    self.load_scenario_to_editor(zone["name"])
                                elif zone["type"] == "move_left":
                                    self.swap_scenarios(zone["name"], -1)
                                elif zone["type"] == "move_right":
                                    self.swap_scenarios(zone["name"], 1)
                                elif zone["type"] == "delete":
                                    del self.scenarios[zone["name"]]
                                    self.save_scenarios_data(self.scenarios)
                                break

                # -------------------------
                # EDITOR EVENTS
                # -------------------------
                elif self.app_state == "EDITOR":
                    mouse_in_grid = self.mouse_in_grid(lx, ly)
                    hovered_cell = (lx // GRID_SIZE, ly // GRID_SIZE)

                    if self.prompt_active:
                        if event.type == pygame.KEYDOWN:
                            if event.key == pygame.K_ESCAPE:
                                self.close_name_prompt()
                                self.set_status("Canceled.")
                            elif event.key == pygame.K_RETURN:
                                self.submit_name_prompt()
                            elif event.key == pygame.K_BACKSPACE:
                                self.prompt_text = self.prompt_text[:-1]
                            elif event.unicode and event.unicode.isprintable() and len(self.prompt_text) < 48:
                                self.prompt_text += event.unicode
                            self.refresh_prompt_hint()
                        continue
                    
                    if event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_ESCAPE:
                            self.app_state = "HOME" # Return to Home Screen
                        elif event.key == pygame.K_1:
                            self.mode = "PLACE_START"
                            self.line_start_cell = None
                        elif event.key == pygame.K_2:
                            self.mode = "PLACE_GOAL"
                            self.line_start_cell = None
                        elif event.key == pygame.K_3:
                            self.mode = "DRAW_GRID"
                        elif event.key == pygame.K_4:
                            self.mode = "PLACE_CONT"
                            self.line_start_cell = None
                        elif event.key == pygame.K_f and self.mode == "DRAW_GRID":
                            self.grid_draw_mode = "FREE"
                            self.line_start_cell = None
                        elif event.key == pygame.K_v and self.mode == "DRAW_GRID":
                            self.grid_draw_mode = "LINE"
                            self.grid_drag_action = None
                        elif event.key == pygame.K_s:
                            self.open_name_prompt("save")
                        elif event.key == pygame.K_c: # Clear all
                            self.start_pose = self.goal_pose = None
                            self.grid_obstacles.clear()
                            self.cont_obstacles.clear()
                            self.line_start_cell = None
                            self.set_status("Cleared current scene.", GREEN)

                    elif event.type == pygame.MOUSEWHEEL:
                        self.current_theta = (self.current_theta + event.y * 15) % 360

                    elif event.type == pygame.MOUSEBUTTONDOWN:
                        if event.button == 1: # Left click
                            if not mouse_in_grid: continue
                            if self.mode == "PLACE_START":
                                pose = {'x': lx, 'y': ly, 'theta': self.current_theta}
                                ok, msg = self.can_place_vehicle(pose, "start")
                                if ok:
                                    self.start_pose = pose
                                    self.set_status("Placed start.", GREEN)
                                else:
                                    self.set_status(msg, ERROR_COLOR)
                            elif self.mode == "PLACE_GOAL":
                                pose = {'x': lx, 'y': ly, 'theta': self.current_theta}
                                ok, msg = self.can_place_vehicle(pose, "goal")
                                if ok:
                                    self.goal_pose = pose
                                    self.set_status("Placed goal.", GREEN)
                                else:
                                    self.set_status(msg, ERROR_COLOR)
                            elif self.mode == "PLACE_CONT":
                                hit_index = self.obstacle_index_at_point(lx, ly)
                                if hit_index is not None:
                                    self.cont_obstacles.pop(hit_index)
                                    self.set_status("Removed obstacle vehicle.", GREEN)
                                else:
                                    pose = {'x': lx, 'y': ly, 'theta': self.current_theta}
                                    ok, msg = self.can_place_vehicle(pose, "obstacle")
                                    if ok:
                                        self.cont_obstacles.append(pose)
                                        self.set_status("Placed obstacle vehicle.", GREEN)
                                    else:
                                        self.set_status(msg, ERROR_COLOR)
                            elif self.mode == "DRAW_GRID":
                                cell = hovered_cell
                                if self.grid_draw_mode == "FREE":
                                    if cell in self.grid_obstacles:
                                        self.grid_obstacles.discard(cell)
                                        self.grid_drag_action = "REMOVE"
                                    else:
                                        can_add, msg = self.can_add_wall_cell(cell)
                                        if can_add:
                                            self.grid_obstacles.add(cell)
                                            self.grid_drag_action = "ADD"
                                        else:
                                            self.grid_drag_action = None
                                            self.set_status(msg, ERROR_COLOR)
                                elif self.grid_draw_mode == "LINE":
                                    if self.line_start_cell is None:
                                        self.line_start_cell = cell
                                    elif self.line_start_cell == cell:
                                        self.line_start_cell = None
                                    else:
                                        start_col, start_row = self.line_start_cell
                                        end_col, end_row = cell
                                        if start_col == end_col:
                                            r0, r1 = sorted((start_row, end_row))
                                            line_cells = [(start_col, r) for r in range(r0, r1 + 1)]
                                            blocked = None
                                            for line_cell in line_cells:
                                                if line_cell in self.grid_obstacles: continue
                                                can_add, msg = self.can_add_wall_cell(line_cell)
                                                if not can_add:
                                                    blocked = msg
                                                    break
                                            if blocked:
                                                self.set_status(blocked, ERROR_COLOR)
                                                continue
                                            for r in range(r0, r1 + 1):
                                                self.grid_obstacles.add((start_col, r))
                                            self.line_start_cell = None
                                        elif start_row == end_row:
                                            c0, c1 = sorted((start_col, end_col))
                                            line_cells = [(c, start_row) for c in range(c0, c1 + 1)]
                                            blocked = None
                                            for line_cell in line_cells:
                                                if line_cell in self.grid_obstacles: continue
                                                can_add, msg = self.can_add_wall_cell(line_cell)
                                                if not can_add:
                                                    blocked = msg
                                                    break
                                            if blocked:
                                                self.set_status(blocked, ERROR_COLOR)
                                                continue
                                            for c in range(c0, c1 + 1):
                                                self.grid_obstacles.add((c, start_row))
                                            self.line_start_cell = None
                                        else:
                                            self.set_status("Line mode supports only vertical/horizontal lines.", ERROR_COLOR)
                        elif event.button == 3: # Right click
                            if self.mode == "DRAW_GRID" and mouse_in_grid:
                                cell = hovered_cell
                                if self.grid_draw_mode == "FREE":
                                    self.grid_obstacles.discard(cell)
                                    self.grid_drag_action = "REMOVE"
                                elif self.grid_draw_mode == "LINE":
                                    self.line_start_cell = None

                    elif event.type == pygame.MOUSEBUTTONUP:
                        if event.button in (1, 3):
                            self.grid_drag_action = None

            # --- Rendering Loop ---
            if self.app_state == "HOME":
                self.window.fill((20, 20, 20))
                title = self.font.render("Scenario Home Screen (Click a card to load/edit) - ESC / Q to Quit", True, WHITE)
                self.window.blit(title, (40, 40))

                card_w, card_h = 300, 340
                margin = 30
                start_x, start_y = 40, 100
                cols = max(1, (self.window_w - 2 * start_x) // (card_w + margin))

                total_cards = len(self.scenarios) + 1
                rows = max(1, (total_cards + cols - 1) // cols)
                content_h = rows * card_h + (rows - 1) * margin
                viewport_h = self.window_h - start_y - 30
                min_scroll = min(0, viewport_h - content_h)
                self.home_scroll = max(min_scroll, min(0, self.home_scroll))
                viewport_rect = pygame.Rect(0, start_y, self.window_w, viewport_h)

                self.home_click_zones = []

                # 1. Create New Card
                cx, cy = start_x, start_y + self.home_scroll
                rect = pygame.Rect(cx, cy, card_w, card_h)
                if rect.colliderect(viewport_rect):
                    pygame.draw.rect(self.window, (40, 40, 40), rect)
                    pygame.draw.rect(self.window, GREEN, rect, 2)
                    text = self.font.render("+ Create New Scenario", True, GREEN)
                    self.window.blit(text, text.get_rect(center=rect.center))
                    self.home_click_zones.append({"type": "create", "rect": rect})

                # 2. Existing Scenarios
                keys = list(self.scenarios.keys())
                for i, name in enumerate(keys):
                    idx = i + 1
                    row = idx // cols
                    col = idx % cols
                    cx = start_x + col * (card_w + margin)
                    cy = start_y + self.home_scroll + row * (card_h + margin)

                    card_rect = pygame.Rect(cx, cy, card_w, card_h)
                    if not card_rect.colliderect(viewport_rect):
                        continue
                    pygame.draw.rect(self.window, (50, 50, 50), card_rect)
                    pygame.draw.rect(self.window, WHITE, card_rect, 1)

                    # Preview Window
                    preview_rect = pygame.Rect(cx + 10, cy + 10, card_w - 20, card_h - 60)
                    pygame.draw.rect(self.window, BG_COLOR, preview_rect)
                    self.draw_mini_preview(self.window, self.scenarios[name], preview_rect)

                    # Label
                    name_surf = self.font.render(name, True, WHITE)
                    self.window.blit(name_surf, (cx + 10, cy + card_h - 40))

                    btn_y = cy + card_h - 40
                    if i > 0:
                        btn_left = pygame.Rect(cx + card_w - 100, btn_y, 25, 25)
                        pygame.draw.rect(self.window, (100, 100, 100), btn_left)
                        self.window.blit(self.ui_font.render("<", True, WHITE), (btn_left.x+8, btn_left.y+5))
                        self.home_click_zones.append({"type": "move_left", "name": name, "rect": btn_left})
                    if i < len(keys) - 1:
                        btn_right = pygame.Rect(cx + card_w - 70, btn_y, 25, 25)
                        pygame.draw.rect(self.window, (100, 100, 100), btn_right)
                        self.window.blit(self.ui_font.render(">", True, WHITE), (btn_right.x+8, btn_right.y+5))
                        self.home_click_zones.append({"type": "move_right", "name": name, "rect": btn_right})
                    btn_del = pygame.Rect(cx + card_w - 35, btn_y, 25, 25)
                    pygame.draw.rect(self.window, ERROR_COLOR, btn_del)
                    self.window.blit(self.ui_font.render("X", True, WHITE), (btn_del.x+8, btn_del.y+5))
                    self.home_click_zones.append({"type": "delete", "name": name, "rect": btn_del})
                    self.home_click_zones.insert(0, {"type": "load", "name": name, "rect": card_rect})

            elif self.app_state == "EDITOR":
                mouse_in_grid = self.mouse_in_grid(lx, ly)

                if self.mode == "DRAW_GRID" and self.grid_draw_mode == "FREE" and self.grid_drag_action and mouse_in_grid:
                    col, row = lx // GRID_SIZE, ly // GRID_SIZE
                    if self.grid_drag_action == "ADD":
                        cell = (col, row)
                        if cell not in self.grid_obstacles:
                            can_add, _ = self.can_add_wall_cell(cell)
                            if can_add: self.grid_obstacles.add(cell)
                    elif self.grid_drag_action == "REMOVE":
                        self.grid_obstacles.discard((col, row))

                self.window.fill(BG_COLOR)

                # Draw the strictly bounded 500x500 Grid Area
                grid_bg_rect = pygame.Rect(self.grid_offset_x, self.grid_offset_y, GRID_WIDTH, GRID_HEIGHT)
                pygame.draw.rect(self.window, GRID_BG, grid_bg_rect)
                pygame.draw.rect(self.window, GRID_COLOR, grid_bg_rect, 2)

                # 1. Grid Obstacles (Rendered with offsets)
                for col, row in self.grid_obstacles:
                    rect = (self.grid_offset_x + col * GRID_SIZE, self.grid_offset_y + row * GRID_SIZE, GRID_SIZE, GRID_SIZE)
                    pygame.draw.rect(self.window, GRID_COLOR, rect)

                # 2. Continuous Obstacles
                for obs in self.cont_obstacles:
                    self.draw_car(obs['x'], obs['y'], obs['theta'], RED, "")

                # 3. Start and Goal
                if self.start_pose:
                    self.draw_car(self.start_pose['x'], self.start_pose['y'], self.start_pose['theta'], AQUA, "A")
                if self.goal_pose:
                    self.draw_car(self.goal_pose['x'], self.goal_pose['y'], self.goal_pose['theta'], ORANGE, "B")

                # 4. Preview / Ghosts
                if mouse_in_grid and self.mode == "DRAW_GRID":
                    ghost_rect = (self.grid_offset_x + lx//GRID_SIZE*GRID_SIZE, self.grid_offset_y + ly//GRID_SIZE*GRID_SIZE, GRID_SIZE, GRID_SIZE)
                    pygame.draw.rect(self.window, WHITE, ghost_rect, 1)

                if mouse_in_grid and self.mode in ("PLACE_START", "PLACE_GOAL", "PLACE_CONT"):
                    role = "start" if self.mode == "PLACE_START" else "goal" if self.mode == "PLACE_GOAL" else "obstacle"
                    preview_pose = {'x': lx, 'y': ly, 'theta': self.current_theta}
                    valid, _ = self.can_place_vehicle(preview_pose, role)
                    preview_color = GREEN if valid else ERROR_COLOR
                    label = "A" if self.mode == "PLACE_START" else "B" if self.mode == "PLACE_GOAL" else ""
                    self.draw_car(lx, ly, self.current_theta, (*preview_color, 110), label)

                if self.mode == "DRAW_GRID" and self.grid_draw_mode == "LINE" and self.line_start_cell:
                    start_col, start_row = self.line_start_cell
                    pygame.draw.rect(self.window, YELLOW, (self.grid_offset_x + start_col * GRID_SIZE, self.grid_offset_y + start_row * GRID_SIZE, GRID_SIZE, GRID_SIZE), 1)
                    if mouse_in_grid:
                        cur_col, cur_row = lx // GRID_SIZE, ly // GRID_SIZE
                        if cur_col == start_col:
                            r0, r1 = sorted((start_row, cur_row))
                            for r in range(r0, r1 + 1):
                                pygame.draw.rect(self.window, YELLOW, (self.grid_offset_x + start_col * GRID_SIZE, self.grid_offset_y + r * GRID_SIZE, GRID_SIZE, GRID_SIZE), 1)
                        elif cur_row == start_row:
                            c0, c1 = sorted((start_col, cur_col))
                            for c in range(c0, c1 + 1):
                                pygame.draw.rect(self.window, YELLOW, (self.grid_offset_x + c * GRID_SIZE, self.grid_offset_y + start_row * GRID_SIZE, GRID_SIZE, GRID_SIZE), 1)

                # 5. UI Overlay
                ui_y = self.window_h - self.ui_height
                pygame.draw.rect(self.window, (20, 20, 20), (0, ui_y, self.window_w, self.ui_height))
                ui_text = [
                    f"Editing: {self.current_scenario_name if self.current_scenario_name else 'Unsaved New Scenario'} | Mode: {self.mode}",
                    f"Grid Draw: {self.grid_draw_mode} (F: Free, V: Line)",
                    "1: Start | 2: Goal | 3: Grid Wall | 4: Cont Wall",
                    "Scroll: Rotate | Click: Place | S: Save | C: Clear Grid | ESC: Back to Home",
                    f"Status: {self.status_message}"
                ]
                for i, text in enumerate(ui_text):
                    color = self.status_color if i == len(ui_text) - 1 else WHITE
                    surf = self.ui_font.render(text, True, color)
                    self.window.blit(surf, (20, ui_y + 15 + i * 22))

                # Popup Prompts
                if self.prompt_active:
                    overlay = pygame.Surface((self.window_w, self.window_h), pygame.SRCALPHA)
                    overlay.fill((0, 0, 0, 130))
                    self.window.blit(overlay, (0, 0))

                    box_w, box_h = 430, 140
                    box_x = (self.window_w - box_w) // 2
                    box_y = (self.window_h - box_h) // 2

                    pygame.draw.rect(self.window, POPUP_BG, (box_x, box_y, box_w, box_h))
                    pygame.draw.rect(self.window, POPUP_BORDER, (box_x, box_y, box_w, box_h), 2)

                    title_surf = self.ui_font.render(self.prompt_title, True, WHITE)
                    self.window.blit(title_surf, (box_x + 12, box_y + 12))
                    hint_surf = self.ui_font.render(self.prompt_hint, True, WHITE)
                    self.window.blit(hint_surf, (box_x + 12, box_y + 40))

                    input_rect = pygame.Rect(box_x + 12, box_y + 66, box_w - 24, 30)
                    pygame.draw.rect(self.window, (35, 35, 35), input_rect)
                    pygame.draw.rect(self.window, WHITE, input_rect, 1)

                    input_surf = self.ui_font.render(self.prompt_text, True, WHITE)
                    self.window.blit(input_surf, (input_rect.x + 8, input_rect.y + 6))
                    help_surf = self.ui_font.render("Enter: confirm | Esc: cancel", True, WHITE)
                    self.window.blit(help_surf, (box_x + 12, box_y + 104))

            pygame.display.flip()
            self.clock.tick(60)

        pygame.quit()

if __name__ == "__main__":
    app = ScenarioBuilder()
    app.run()