import pygame
import time
import collections

# =====================================================================
# Configuration & Theme Styling
# =====================================================================
WIDTH, HEIGHT = 960, 700
FPS = 60

# Palette
BG_COLOR = (242, 245, 249)
PANEL_BG = (255, 255, 255)
PANEL_BORDER = (215, 222, 232)
CARD_BG = (248, 250, 253)

TEXT_MAIN = (20, 25, 35)
TEXT_MUTED = (90, 100, 115)
TEXT_DIM = (145, 155, 170)

# Colors
COLOR_P = (25, 115, 230)       # Royal Blue
COLOR_I = (220, 50, 50)        # Crimson Red
COLOR_D = (30, 150, 75)        # Emerald Green
COLOR_TARGET = (140, 60, 215)   # Purple
COLOR_THRUST = (245, 130, 30)   # Orange
TARGET_TINT = (248, 240, 255)

# =====================================================================
# Physical Constants (MEGN300 1-DOF Drone Model)
# =====================================================================
GRAVITY = 600.0   # mg downwards (MASS * 600)
MASS = 1.0        # kg
DRAG = 2.0        # Aerodynamic drag c
MAX_THRUST = 4000.0  # Max thrust limit

# =====================================================================
# PID Controller Class
# =====================================================================
class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, setpoint, measured_value, dt, is_constrained=False):
        if dt <= 0:
            return 0.0
        
        error = setpoint - measured_value
        p_out = self.kp * error
        
        if not is_constrained:
            self.integral += error * dt
            self.integral = max(min(self.integral, 400.0), -400.0)
        i_out = self.ki * self.integral
        
        derivative = (error - self.prev_error) / dt
        d_out = self.kd * derivative
        self.prev_error = error
        
        raw_output = p_out + i_out + d_out
        return max(0.0, min(raw_output, MAX_THRUST))

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

# =====================================================================
# UI Widgets: Slider & Button
# =====================================================================
class Slider:
    def __init__(self, x, y, w, h, label, min_val, max_val, initial_val, color):
        self.rect = pygame.Rect(x, y, w, h)
        self.label = label
        self.min_val = min_val
        self.max_val = max_val
        self.val = initial_val
        self.color = color
        self.handle_rect = pygame.Rect(x, y - 5, 14, h + 10)
        self.update_handle_pos()
        self.dragging = False

    def update_handle_pos(self):
        pos_ratio = (self.val - self.min_val) / (self.max_val - self.min_val)
        self.handle_rect.centerx = self.rect.x + pos_ratio * self.rect.width

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if self.handle_rect.collidepoint(event.pos) or self.rect.collidepoint(event.pos):
                self.dragging = True
                rel_x = max(0, min(event.pos[0] - self.rect.x, self.rect.width))
                self.val = self.min_val + (rel_x / self.rect.width) * (self.max_val - self.min_val)
                self.update_handle_pos()
                return True
        elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            self.dragging = False
        elif event.type == pygame.MOUSEMOTION and self.dragging:
            rel_x = max(0, min(event.pos[0] - self.rect.x, self.rect.width))
            self.val = self.min_val + (rel_x / self.rect.width) * (self.max_val - self.min_val)
            self.update_handle_pos()
            return True
        return False

    def draw(self, screen, font_title, font_val):
        pygame.draw.rect(screen, (225, 230, 238), self.rect, border_radius=4)
        fill_w = max(0, self.handle_rect.centerx - self.rect.x)
        if fill_w > 0:
            fill_rect = pygame.Rect(self.rect.x, self.rect.y, fill_w, self.rect.height)
            pygame.draw.rect(screen, self.color, fill_rect, border_radius=4)
        pygame.draw.rect(screen, (255, 255, 255), self.handle_rect, border_radius=5)
        pygame.draw.rect(screen, self.color, self.handle_rect, width=2, border_radius=5)
        lbl = font_title.render(self.label, True, TEXT_MAIN)
        screen.blit(lbl, (self.rect.x, self.rect.y - 20))
        val_str = f"{self.val:.2f}" if self.max_val <= 50 else f"{self.val:.0f} m"
        val_surf = font_val.render(val_str, True, self.color)
        screen.blit(val_surf, (self.rect.right - val_surf.get_width(), self.rect.y - 20))

class Button:
    def __init__(self, x, y, w, h, text, bg_color, text_color=(255, 255, 255), border_color=None):
        self.rect = pygame.Rect(x, y, w, h)
        self.text = text
        self.bg_color = bg_color
        self.text_color = text_color
        self.border_color = border_color
        self.hovered = False

    def handle_event(self, event):
        if event.type == pygame.MOUSEMOTION:
            self.hovered = self.rect.collidepoint(event.pos)
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if self.rect.collidepoint(event.pos):
                return True
        return False

    def draw(self, screen, font):
        c = self.bg_color
        if self.hovered:
            c = (min(c[0] + 18, 255), min(c[1] + 18, 255), min(c[2] + 18, 255))
        pygame.draw.rect(screen, c, self.rect, border_radius=6)
        if self.border_color:
            pygame.draw.rect(screen, self.border_color, self.rect, width=1, border_radius=6)
        txt = font.render(self.text, True, self.text_color)
        txt_rect = txt.get_rect(center=self.rect.center)
        screen.blit(txt, txt_rect)

# =====================================================================
# Rendering Functions
# =====================================================================
def draw_controls_and_graph(screen, sliders, buttons, history, target_history, fonts):
    font_xs, font_sm, font_bold, font_xl = fonts[0], fonts[1], fonts[2], fonts[3]
    
    # 1. Controls Panel Card
    ctrl_panel = pygame.Rect(20, 20, 280, 395)
    pygame.draw.rect(screen, PANEL_BG, ctrl_panel, border_radius=12)
    pygame.draw.rect(screen, PANEL_BORDER, ctrl_panel, width=1, border_radius=12)

    c_title = font_xl.render("PID Controller Controls", True, TEXT_MAIN)
    screen.blit(c_title, (ctrl_panel.x + 18, ctrl_panel.y + 15))
    c_sub = font_sm.render("Adjust Gains & Target in Real-Time", True, TEXT_MUTED)
    screen.blit(c_sub, (ctrl_panel.x + 18, ctrl_panel.y + 42))

    for s in sliders:
        s.draw(screen, font_bold, font_bold)

    preset_lbl = font_xs.render("TUNING PRESETS & ACTIONS", True, TEXT_MUTED)
    screen.blit(preset_lbl, (ctrl_panel.x + 18, 272))

    for b in buttons:
        b.draw(screen, font_bold)

    # 2. Response Graph Panel Card
    graph_panel = pygame.Rect(20, 430, 280, 250)
    pygame.draw.rect(screen, PANEL_BG, graph_panel, border_radius=12)
    pygame.draw.rect(screen, PANEL_BORDER, graph_panel, width=1, border_radius=12)

    g_title = font_bold.render("Live Response Graph", True, TEXT_MAIN)
    screen.blit(g_title, (graph_panel.x + 18, graph_panel.y + 12))
    
    # Legend
    pygame.draw.line(screen, COLOR_TARGET, (graph_panel.x + 18, graph_panel.y + 36), (graph_panel.x + 32, graph_panel.y + 36), 2)
    leg_t = font_xs.render("Target", True, TEXT_MUTED)
    screen.blit(leg_t, (graph_panel.x + 36, graph_panel.y + 30))

    pygame.draw.line(screen, COLOR_P, (graph_panel.x + 95, graph_panel.y + 36), (graph_panel.x + 109, graph_panel.y + 36), 2)
    leg_d = font_xs.render("Altitude", True, TEXT_MUTED)
    screen.blit(leg_d, (graph_panel.x + 113, graph_panel.y + 30))

    hint_r = font_xs.render("Key [R] = Reset", True, (130, 140, 155))
    screen.blit(hint_r, (graph_panel.right - hint_r.get_width() - 18, graph_panel.y + 30))

    # Graph canvas
    graph_rect = pygame.Rect(graph_panel.x + 15, graph_panel.y + 50, graph_panel.width - 30, 185)
    pygame.draw.rect(screen, CARD_BG, graph_rect, border_radius=6)
    pygame.draw.rect(screen, PANEL_BORDER, graph_rect, width=1, border_radius=6)

    # Reference grid lines
    for g_mark in [0, 250, 500]:
        gy = graph_rect.bottom - (g_mark / 550) * graph_rect.height
        pygame.draw.line(screen, (230, 235, 242), (graph_rect.x, gy), (graph_rect.right, gy), 1)
        g_lbl = font_xs.render(f"{g_mark}m", True, TEXT_DIM)
        screen.blit(g_lbl, (graph_rect.x + 4, gy - 12))

    if len(history) > 2:
        pts_target = [(graph_rect.x + (i / 400) * graph_rect.width,
                       graph_rect.bottom - (min(val, 550) / 550) * graph_rect.height)
                      for i, val in enumerate(target_history)]
        pygame.draw.lines(screen, COLOR_TARGET, False, pts_target, 1)

        pts_drone = [(graph_rect.x + (i / 400) * graph_rect.width,
                      graph_rect.bottom - (min(val, 550) / 550) * graph_rect.height)
                     for i, val in enumerate(history)]
        pygame.draw.lines(screen, COLOR_P, False, pts_drone, 2)


def draw_simulation(screen, drone_alt, drone_vel, target_alt, thrust, fonts, tick_count):
    font_xs, font_sm, font_bold, font_xl = fonts[0], fonts[1], fonts[2], fonts[3]
    panel = pygame.Rect(320, 20, 620, 660)
    pygame.draw.rect(screen, PANEL_BG, panel, border_radius=12)
    pygame.draw.rect(screen, PANEL_BORDER, panel, width=1, border_radius=12)

    title = font_xl.render("Drone Altitude Plant", True, TEXT_MAIN)
    screen.blit(title, (panel.x + 20, panel.y + 15))
    sub = font_sm.render("Interactive 1-DOF Vertical Flight Dynamics (m=1.0kg, g=600, drag=2.0)", True, TEXT_MUTED)
    screen.blit(sub, (panel.x + 20, panel.y + 42))

    # Inner simulation shaft
    sim_rect = pygame.Rect(panel.x + 15, panel.y + 65, panel.width - 30, 580)
    pygame.draw.rect(screen, (249, 251, 254), sim_rect, border_radius=8)
    pygame.draw.rect(screen, (225, 232, 242), sim_rect, width=1, border_radius=8)

    ground_y = sim_rect.bottom - 40
    
    # Altitude Ruler & dashed guide lines
    for alt_mark in range(0, 550, 100):
        y_mark = ground_y - alt_mark
        if sim_rect.top + 20 < y_mark <= ground_y:
            for gx in range(sim_rect.left + 50, sim_rect.right - 10, 15):
                pygame.draw.line(screen, (230, 236, 245), (gx, y_mark), (gx + 8, y_mark), 1)
            lbl = font_xs.render(f"{alt_mark} m", True, TEXT_DIM)
            screen.blit(lbl, (sim_rect.left + 12, y_mark - 7))
            pygame.draw.line(screen, (180, 190, 205), (sim_rect.left + 40, y_mark), (sim_rect.left + 48, y_mark), 2)

    # Launch / Landing Pad at 0m
    pygame.draw.rect(screen, (220, 226, 235), (sim_rect.left + 2, ground_y, sim_rect.width - 4, 38), border_bottom_left_radius=8, border_bottom_right_radius=8)
    pygame.draw.line(screen, (160, 170, 185), (sim_rect.left, ground_y), (sim_rect.right, ground_y), 3)
    pad_rect = pygame.Rect(sim_rect.centerx - 80, ground_y + 5, 160, 26)
    pygame.draw.rect(screen, (200, 210, 222), pad_rect, border_radius=4)
    pad_lbl = font_xs.render("LAUNCH / LANDING PAD (0 m)", True, (100, 110, 125))
    screen.blit(pad_lbl, pad_lbl.get_rect(center=pad_rect.center))

    # Target line & Setpoint Pill
    target_y = ground_y - target_alt
    if sim_rect.top + 10 < target_y < ground_y:
        pygame.draw.line(screen, COLOR_TARGET, (sim_rect.left + 45, target_y), (sim_rect.right - 10, target_y), 2)
        target_pill = pygame.Rect(sim_rect.right - 130, target_y - 12, 120, 24)
        pygame.draw.rect(screen, TARGET_TINT, target_pill, border_radius=12)
        pygame.draw.rect(screen, COLOR_TARGET, target_pill, width=1, border_radius=12)
        t_txt = font_xs.render(f"TARGET: {target_alt:.0f} m", True, COLOR_TARGET)
        screen.blit(t_txt, t_txt.get_rect(center=target_pill.center))

    # Drone position
    drone_y = ground_y - drone_alt
    drone_cx = sim_rect.centerx - 20
    
    # Animated Thrust Exhaust Flames (proportional to commanded thrust)
    if thrust > 15:
        flame_len = min(60, int((thrust / MAX_THRUST) * 70) + 5)
        flame_len += (tick_count % 3) * 2
        for thrust_x in [drone_cx - 24, drone_cx + 24]:
            pts_outer = [(thrust_x - 5, drone_y + 10), (thrust_x + 5, drone_y + 10), (thrust_x, drone_y + 10 + flame_len)]
            pygame.draw.polygon(screen, COLOR_THRUST, pts_outer)
            pts_inner = [(thrust_x - 2, drone_y + 10), (thrust_x + 2, drone_y + 10), (thrust_x, drone_y + 10 + int(flame_len * 0.6))]
            pygame.draw.polygon(screen, (255, 235, 100), pts_inner)

    # Drone fuselage
    drone_w, drone_h = 68, 20
    d_body = pygame.Rect(drone_cx - drone_w // 2, drone_y - drone_h // 2, drone_w, drone_h)
    pygame.draw.rect(screen, (35, 42, 54), d_body, border_radius=6)
    pygame.draw.rect(screen, (70, 80, 98), d_body, width=1, border_radius=6)
    
    # Center LED indicator
    led_col = COLOR_P if abs(drone_alt - target_alt) > 5 else (50, 210, 100)
    pygame.draw.circle(screen, led_col, (drone_cx, drone_y), 4)

    # Spinning rotors
    rotor_offset = (tick_count * 15) % 18
    pygame.draw.line(screen, (60, 70, 85), (drone_cx - 30, drone_y), (drone_cx - 30, drone_y - 8), 2)
    pygame.draw.line(screen, (100, 115, 135), (drone_cx - 42 + rotor_offset // 2, drone_y - 9), (drone_cx - 18 - rotor_offset // 2, drone_y - 9), 2)
    pygame.draw.line(screen, (60, 70, 85), (drone_cx + 30, drone_y), (drone_cx + 30, drone_y - 8), 2)
    pygame.draw.line(screen, (100, 115, 135), (drone_cx + 18 + rotor_offset // 2, drone_y - 9), (drone_cx + 42 - rotor_offset // 2, drone_y - 9), 2)

    # Landing skids
    pygame.draw.line(screen, (80, 90, 105), (drone_cx - 22, drone_y + 10), (drone_cx - 22, drone_y + 15), 2)
    pygame.draw.line(screen, (80, 90, 105), (drone_cx + 22, drone_y + 10), (drone_cx + 22, drone_y + 15), 2)
    pygame.draw.line(screen, (80, 90, 105), (drone_cx - 28, drone_y + 15), (drone_cx + 28, drone_y + 15), 2)

    # Floating Telemetry Tag
    tag_rect = pygame.Rect(drone_cx + 42, drone_y - 14, 120, 28)
    pygame.draw.rect(screen, (255, 255, 255), tag_rect, border_radius=5)
    pygame.draw.rect(screen, PANEL_BORDER, tag_rect, width=1, border_radius=5)
    tag_alt = font_xs.render(f"Alt: {drone_alt:.1f} m", True, TEXT_MAIN)
    vel_col = COLOR_D if drone_vel >= 0 else (210, 60, 60)
    tag_vel = font_xs.render(f"Vel: {drone_vel:+.1f} m/s", True, vel_col)
    screen.blit(tag_alt, (tag_rect.x + 6, tag_rect.y + 2))
    screen.blit(tag_vel, (tag_rect.x + 6, tag_rect.y + 14))


# =====================================================================
# Main Application Loop
# =====================================================================
def main():
    pygame.init()
    pygame.font.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("PID Controller Visualizer - Drone Altitude Flight Dynamics")
    clock = pygame.time.Clock()

    fonts = [
        pygame.font.SysFont("arial", 11),
        pygame.font.SysFont("arial", 13),
        pygame.font.SysFont("arial", 14, bold=True),
        pygame.font.SysFont("arial", 18, bold=True)
    ]

    sliders = [
        Slider(40, 92, 240, 8, "Kp (Proportional)", 0, 30, 5.0, COLOR_P),
        Slider(40, 142, 240, 8, "Ki (Integral)", 0, 40, 0.0, COLOR_I),
        Slider(40, 192, 240, 8, "Kd (Derivative)", 0, 10, 0.0, COLOR_D),
        Slider(40, 242, 240, 8, "Target Altitude", 50, 500, 300, COLOR_TARGET)
    ]

    buttons = [
        Button(38, 290, 244, 28, "Raw Ziegler-Nichols", (70, 78, 92)),
        Button(38, 324, 244, 28, "Apply Fine Tuning", (240, 130, 30)),
        Button(38, 358, 244, 28, "Reset Simulation [R]", (230, 235, 245), text_color=(60, 70, 85), border_color=(200, 210, 225))
    ]

    pid = PID(sliders[0].val, sliders[1].val, sliders[2].val)
    
    drone_alt = 0.0
    drone_vel = 0.0
    history = collections.deque(maxlen=400)
    target_history = collections.deque(maxlen=400)

    running = True
    last_time = time.time()
    tick_count = 0

    while running:
        screen.fill(BG_COLOR)
        now = time.time()
        dt = min(now - last_time, 0.05)
        last_time = now
        tick_count += 1

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_r:
                    pid.reset()
                    drone_alt = 0.0
                    drone_vel = 0.0
                    history.clear()
                    target_history.clear()

            for s in sliders:
                if s.handle_event(event):
                    pid.kp = sliders[0].val
                    pid.ki = sliders[1].val
                    pid.kd = sliders[2].val

            # Preset: Raw Ziegler-Nichols
            if buttons[0].handle_event(event):
                sliders[0].val = 10.8  # 0.6 * Ku
                sliders[1].val = 27.0  # 1.2 * Ku / Tu
                sliders[2].val = 1.08  # 0.075 * Ku * Tu
                for s in sliders[:3]:
                    s.update_handle_pos()
                pid.kp, pid.ki, pid.kd = sliders[0].val, sliders[1].val, sliders[2].val
                pid.reset()

            # Preset: Human-Adjusted Fine Tuning
            if buttons[1].handle_event(event):
                sliders[0].val = 10.0
                sliders[1].val = 10.0
                sliders[2].val = 3.5
                for s in sliders[:3]:
                    s.update_handle_pos()
                pid.kp, pid.ki, pid.kd = sliders[0].val, sliders[1].val, sliders[2].val
                pid.reset()

            # Action: Reset Simulation
            if buttons[2].handle_event(event):
                pid.reset()
                drone_alt = 0.0
                drone_vel = 0.0
                history.clear()
                target_history.clear()

        # --- PID & Physics Simulation Update ---
        target_alt = sliders[3].val
        is_constrained = (drone_alt <= 0 and target_alt > drone_alt)
        thrust = pid.update(target_alt, drone_alt, dt, is_constrained)
        
        # Plant dynamics: m * a = thrust - gravity - drag * v
        accel = (thrust / MASS) - GRAVITY - (drone_vel * DRAG)
        drone_vel += accel * dt
        drone_alt += drone_vel * dt
        
        if drone_alt < 0:
            drone_alt = 0.0
            drone_vel = 0.0
            
        history.append(drone_alt)
        target_history.append(target_alt)

        # --- Render Components ---
        draw_controls_and_graph(screen, sliders, buttons, history, target_history, fonts)
        draw_simulation(screen, drone_alt, drone_vel, target_alt, thrust, fonts, tick_count)

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()


if __name__ == "__main__":
    main()
