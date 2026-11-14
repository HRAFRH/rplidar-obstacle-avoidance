#!/usr/bin/env python3
"""Lidar Radar - 360 degree obstacle display for PX4 / ArduPilot (OBSTACLE_DISTANCE).
Achraf Rahibe - RPLidar obstacle avoidance project.

Start:  double-click "Start Lidar Radar.bat" (Windows) or run ./start_lidar_radar.sh (Linux),
        or:  python lidar_radar.py [COM12 | /dev/ttyACM0 | udp:14445]
With no argument it finds the flight controller's USB port by itself. If QGroundControl holds the
port, it listens to QGC's MAVLink forwarding on UDP 14445 instead (QGC: Settings > MAVLink >
"Enable MAVLink forwarding", host localhost:14445).

Front is up and matches the flight controller's arrow. Red inside CP_DIST (read from the vehicle,
2 m if unknown), yellow up to 2 x CP_DIST, green beyond.
Keys: mouse wheel or + / - zoom,  M sound,  T always on top,  Q quit.
"""
import math
import re
import sys
import threading
import time
import tkinter as tk

from pymavlink import mavutil

try:
    import serial.tools.list_ports as list_ports
except ImportError:
    list_ports = None
try:
    import winsound
except ImportError:
    winsound = None

OWNER = "Achraf Rahibe"
APP_NAME = "LIDAR RADAR"
STALE_S = 1.0
RANGES = [1.5, 3.0, 6.0, 12.0]

# USB vendor/product ids of flight controllers (pid None = any)
FC_USB = [(0x3185, None, "Holybro"), (0x2DAE, None, "CubePilot"), (0x1209, 0x5740, "ArduPilot"),
          (0x26AC, None, "PX4 FMU"), (0x1209, 0x5741, "ArduPilot")]

VEHICLE_KIND = {1: "plane", 16: "plane", 2: "quad", 3: "quad", 4: "heli", 13: "hexa", 14: "octo", 15: "tri",
                10: "rover", 11: "boat", 12: "boat"}
VEHICLE_NAME = {"plane": "Fixed wing", "quad": "Quadcopter", "heli": "Helicopter", "hexa": "Hexacopter",
                "octo": "Octocopter", "tri": "Tricopter", "rover": "Rover", "boat": "Boat", "vtol": "VTOL"}
AUTOPILOT = {12: "PX4", 3: "ArduPilot"}
EARTH_FRAMES = (1, 7)  # MAV_FRAME_LOCAL_NED, LOCAL_OFFSET_NED: angles from north, not the nose
SHELL_MSG = re.compile(r"increment:\s*([\d.]+)\s+angle_offset:\s*([-\d.]+)\s+distances:\s*\[([^\]]*)\]"
                       r"\s+min_distance:\s*(\d+)\s+max_distance:\s*(\d+)", re.S)


def find_fc_ports():
    """Serial ports that look like a flight controller's MAVLink port."""
    found = []
    if not list_ports:
        return found
    for p in list_ports.comports():
        if p.vid is None:
            continue
        desc = (p.description or "") + " " + (p.hwid or "")
        if "SLCAN" in desc.upper() or "MI_02" in desc.upper():
            continue  # ArduPilot's second USB interface is CAN, not MAVLink
        for vid, pid, maker in FC_USB:
            if p.vid == vid and (pid is None or p.pid == pid):
                found.append((p.device, maker))
                break
    return found


class Feed(threading.Thread):
    """Reads MAVLink and keeps the latest scan and vehicle state."""

    def __init__(self, arg):
        super().__init__(daemon=True)
        self.arg = arg
        self.lock = threading.Lock()
        self.scan = None
        self.scan_time = 0.0
        self.rate = 0.0
        self.yaw_deg = None
        self.mode, self.armed, self.mav_type, self.autopilot = "", False, None, None
        self.cp_dist = None
        self.link = "searching..."
        self.status = "looking for the flight controller"
        self.running = True
        self.shell_mode = False
        self.m = None
        self._stamps = []

    def open_link(self):
        candidates = []
        if self.arg:
            a = self.arg
            candidates.append(("udpin:0.0.0.0:" + a.split(":", 1)[1], "UDP " + a.split(":", 1)[1])
                              if a.startswith("udp:") else (a, a))
        else:
            for dev, maker in find_fc_ports():
                candidates.append((dev, "%s (%s USB)" % (dev, maker)))
            candidates.append(("udpin:0.0.0.0:14445", "UDP 14445 (QGC forwarding)"))
        errors = []
        for conn, label in candidates:
            try:
                self.m = mavutil.mavlink_connection(conn, baud=115200, source_system=254)
                self.link = label
                return True
            except Exception as e:
                errors.append("%s: %s" % (label, "in use (QGC open?)" if "denied" in str(e).lower() else e))
        self.status = "; ".join(errors) or "no flight controller found"
        return False

    def shell_send(self, text):
        flags = (mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND | mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE)
        data = text.encode()
        while True:
            chunk, data = data[:70], data[70:]
            self.m.mav.serial_control_send(10, flags, 0, 0, len(chunk), list(chunk) + [0] * (70 - len(chunk)))
            if not data:
                break

    def new_scan(self, dist, inc, offset, max_cm, earth):
        now = time.time()
        self._stamps = [t for t in self._stamps if now - t < 2.0] + [now]
        with self.lock:
            self.scan = (dist, inc, offset, max_cm, earth)
            self.scan_time = now
            self.rate = len(self._stamps) / 2.0
            self.status = "live"

    def run(self):
        while self.running and not self.open_link():
            time.sleep(2)
        serial_link = not self.link.startswith("UDP")
        self.status = "waiting for the vehicle on " + self.link
        self.m.target_system = 0
        t_start, last_req, last_poll, buf = time.time(), 0.0, 0.0, ""
        while self.running:
            now = time.time()
            if self.cp_dist is None and self.m.target_system and now - last_req > 3:
                self.m.mav.param_request_read_send(self.m.target_system, 1, b"CP_DIST", -1)
                last_req = now
            if serial_link and not self.shell_mode and self.scan is None and now - t_start > 5 \
                    and self.autopilot == 12:
                # stock PX4 does not stream OBSTACLE_DISTANCE: read it through the PX4 shell
                self.shell_mode = True
                self.shell_send("\n")
                time.sleep(0.3)
                self.shell_send("listener obstacle_distance -n 1000000 -r 10\n")
            if self.shell_mode and now - last_poll > 0.05:
                self.shell_send("")
                last_poll = now
            try:
                msg = self.m.recv_match(blocking=True, timeout=0.05)
            except Exception as e:
                self.status = "link lost: %s" % e
                time.sleep(1)
                continue
            while msg:
                t = msg.get_type()
                if t == "OBSTACLE_DISTANCE":
                    inc = msg.increment_f if msg.increment_f > 0 else float(msg.increment)
                    self.new_scan(list(msg.distances), inc or 5.0, msg.angle_offset, msg.max_distance,
                                  msg.frame in EARTH_FRAMES)
                elif t == "ATTITUDE":
                    self.yaw_deg = math.degrees(msg.yaw) % 360
                elif t == "HEARTBEAT" and msg.get_srcComponent() == 1 and msg.type != mavutil.mavlink.MAV_TYPE_GCS:
                    self.m.target_system = msg.get_srcSystem()
                    self.mode = mavutil.mode_string_v10(msg)
                    self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    self.mav_type, self.autopilot = msg.type, msg.autopilot
                elif t == "PARAM_VALUE" and msg.param_id == "CP_DIST":
                    self.cp_dist = msg.param_value
                elif t == "SERIAL_CONTROL" and self.shell_mode:
                    buf += bytes(msg.data[:msg.count]).decode(errors="replace")
                msg = self.m.recv_match(blocking=False)
            while self.shell_mode:
                mm = SHELL_MSG.search(buf)
                if not mm:
                    break
                buf = buf[mm.end():]
                try:
                    dist = [int(x) for x in mm.group(3).split(",") if x.strip()]
                except ValueError:
                    continue
                self.new_scan(dist, float(mm.group(1)), float(mm.group(2)), int(mm.group(5)), False)
            if len(buf) > 20000:
                buf = buf[-4000:]
        try:
            if self.shell_mode:
                self.shell_send("\x03\n")
                time.sleep(0.2)
            self.m.close()
        except Exception:
            pass


class Beeper(threading.Thread):
    """Parking-sensor beeps: silent beyond CP_DIST, faster when closer."""

    def __init__(self):
        super().__init__(daemon=True)
        self.enabled = False
        self.level = None

    def run(self):
        while True:
            lvl = self.level
            if not self.enabled or lvl is None:
                time.sleep(0.1)
                continue
            if winsound:
                winsound.Beep(int(900 + 900 * lvl), 70)
            else:
                sys.stdout.write("\a")
                sys.stdout.flush()
            time.sleep(max(0.04, 0.6 * (1 - lvl)))


class App:
    BG, PANEL, CARD, LINE = "#0d1014", "#151a20", "#1b2129", "#2a333d"
    TEXT, DIM, ACCENT = "#e8edf2", "#7d8a97", "#4aa3ff"
    RED, YELLOW, GREEN = "#ff453a", "#ffd60a", "#30d158"
    SHADOW = {"#ff453a": "#3b1714", "#ffd60a": "#372f0b", "#30d158": "#10301b"}
    FONT = "Segoe UI" if sys.platform.startswith("win") else "DejaVu Sans"

    def __init__(self, feed, beeper):
        self.feed, self.beeper = feed, beeper
        self.range_i = 2
        self.topmost = False
        self.root = tk.Tk()
        self.root.title("Lidar Radar - " + OWNER)
        self.root.configure(bg=self.BG)
        sh = self.root.winfo_screenheight()
        h = min(860, sh - 120)
        self.root.geometry("%dx%d+30+10" % (int(h * 0.82), h))
        self.root.minsize(420, 520)
        self.c = tk.Canvas(self.root, bg=self.BG, highlightthickness=0)
        self.c.pack(fill="both", expand=True)
        for k in ("<plus>", "<equal>", "<KP_Add>"):
            self.root.bind(k, lambda e: self.zoom(-1))
        for k in ("<minus>", "<KP_Subtract>"):
            self.root.bind(k, lambda e: self.zoom(1))
        self.root.bind("<MouseWheel>", lambda e: self.zoom(-1 if e.delta > 0 else 1))
        self.root.bind("<Button-4>", lambda e: self.zoom(-1))
        self.root.bind("<Button-5>", lambda e: self.zoom(1))
        for k in ("m", "M"):
            self.root.bind(k, lambda e: self.toggle_sound())
        for k in ("t", "T"):
            self.root.bind(k, lambda e: self.toggle_top())
        for k in ("q", "Q", "<Escape>"):
            self.root.bind(k, lambda e: self.close())
        self.c.tag_bind("zoom_in", "<Button-1>", lambda e: self.zoom(-1))
        self.c.tag_bind("zoom_out", "<Button-1>", lambda e: self.zoom(1))
        self.c.tag_bind("sound", "<Button-1>", lambda e: self.toggle_sound())
        self.c.tag_bind("pin", "<Button-1>", lambda e: self.toggle_top())
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.tick()

    # ---------- controls ----------
    def zoom(self, step):
        self.range_i = max(0, min(len(RANGES) - 1, self.range_i + step))

    def toggle_sound(self):
        self.beeper.enabled = not self.beeper.enabled

    def toggle_top(self):
        self.topmost = not self.topmost
        self.root.attributes("-topmost", self.topmost)

    def close(self):
        self.feed.running = False
        self.root.after(300, self.root.destroy)

    # ---------- helpers ----------
    def cp(self):
        return self.feed.cp_dist if self.feed.cp_dist and self.feed.cp_dist > 0 else 2.0

    def color(self, m):
        cp = self.cp()
        return self.RED if m < cp else (self.YELLOW if m < 2 * cp else self.GREEN)

    def f(self, size, bold=False):
        return (self.FONT, int(size), "bold") if bold else (self.FONT, int(size))

    def kind(self):
        t = self.feed.mav_type
        if t is None:
            return "quad"
        if 19 <= t <= 25:
            return "vtol"
        return VEHICLE_KIND.get(t, "quad")

    # ---------- drawing ----------
    def tick(self):
        c = self.c
        c.delete("all")
        W, H = max(c.winfo_width(), 400), max(c.winfo_height(), 500)
        s = H / 860.0  # scale for fonts and line widths
        f = self.feed
        with f.lock:
            scan, rate, status = f.scan, f.rate, f.status
            age = time.time() - f.scan_time if f.scan_time else None
        stale = age is None or age > STALE_S
        cp = self.cp()
        rng = RANGES[self.range_i]

        # header
        hh = int(64 * s)
        c.create_rectangle(0, 0, W, hh, fill=self.PANEL, outline="")
        c.create_line(0, hh, W, hh, fill=self.LINE)
        c.create_text(18 * s, hh * 0.36, anchor="w", text=APP_NAME, fill=self.TEXT, font=self.f(15 * s, True))
        ap = AUTOPILOT.get(f.autopilot, "")
        vt = VEHICLE_NAME.get(self.kind(), "") if f.mav_type is not None else ""
        sub = "  ·  ".join(x for x in (ap, vt, f.mode, "ARMED" if f.armed else ("disarmed" if f.mode else "")) if x)
        c.create_text(18 * s, hh * 0.74, anchor="w", text=sub or "waiting for vehicle",
                      fill=self.RED if f.armed else self.DIM, font=self.f(9.5 * s))
        c.create_text(W - 18 * s, hh * 0.36, anchor="e", text=OWNER, fill=self.ACCENT, font=self.f(11 * s, True))
        c.create_text(W - 18 * s, hh * 0.74, anchor="e", text=f.link, fill=self.DIM, font=self.f(9 * s))

        # direction cards
        side_min = [None, None, None, None]
        closest = None
        free = unknown = 0
        if scan and not stale:
            dist, inc, offset, max_cm, earth = scan
            yaw = f.yaw_deg or 0.0
            obstacles = []
            for i, d in enumerate(dist[:int(round(360 / inc))] if inc > 0 else []):
                if d == 65535:
                    unknown += 1
                    continue
                if d > max_cm or d <= 0:
                    free += 1
                    continue
                m = d / 100.0
                ang = (offset + i * inc - (yaw if earth else 0.0)) % 360
                obstacles.append((m, ang))
                if closest is None or m < closest[0]:
                    closest = (m, ang)
                k = int(((ang + 45) % 360) // 90)
                if side_min[k] is None or m < side_min[k]:
                    side_min[k] = m
        else:
            obstacles, inc = [], 5.0

        ct, chh = hh + 12 * s, 70 * s
        gap = 10 * s
        cw = (W - 2 * 18 * s - 3 * gap) / 4
        for k, name in enumerate(("FRONT", "RIGHT", "BACK", "LEFT")):
            x0 = 18 * s + k * (cw + gap)
            d = side_min[k]
            col = self.DIM if stale else (self.GREEN if d is None else self.color(d))
            c.create_rectangle(x0, ct, x0 + cw, ct + chh, fill=self.CARD, outline=self.LINE)
            c.create_rectangle(x0, ct, x0 + 4 * s, ct + chh, fill=col, outline="")
            c.create_text(x0 + 14 * s, ct + chh * 0.3, anchor="w", text=name, fill=self.DIM, font=self.f(9 * s, True))
            val = "--" if stale else ("clear" if d is None else "%.2f m" % d)
            c.create_text(x0 + 14 * s, ct + chh * 0.68, anchor="w", text=val, fill=col, font=self.f(16 * s, True))

        # radar geometry
        top = ct + chh + 14 * s
        fh = 34 * s
        area_h = H - top - fh
        cx, cy = W / 2, top + area_h / 2
        R = min(W / 2 - 46 * s, area_h / 2 - 34 * s)

        def px(m):
            return R * min(m, rng) / rng

        def xy(deg, r):
            a = math.radians(deg)
            return cx + r * math.sin(a), cy - r * math.cos(a)

        c.create_oval(cx - R - 8 * s, cy - R - 8 * s, cx + R + 8 * s, cy + R + 8 * s, fill=self.PANEL, outline=self.LINE)
        for q in range(1, 5):
            r = R * q / 4
            c.create_oval(cx - r, cy - r, cx + r, cy + r, outline=self.LINE)
            c.create_text(cx + 4 * s, cy - r - 7 * s, text="%g m" % (rng * q / 4), anchor="w",
                          fill=self.DIM, font=self.f(8 * s))
        for deg in range(0, 360, 30):
            x0, y0 = xy(deg, R)
            x1, y1 = xy(deg, R - (12 if deg % 90 == 0 else 7) * s)
            c.create_line(x0, y0, x1, y1, fill="#4a5662", width=max(1, int(2 * s)))
        for m, col in ((cp, self.RED), (2 * cp, self.YELLOW)):
            if m <= rng:
                r = px(m)
                c.create_oval(cx - r, cy - r, cx + r, cy + r, outline=col, dash=(6, 4), width=max(1, int(2 * s)))

        for m, ang in obstacles:
            col = self.color(m)
            arc = [ang - inc / 2 + j * inc / 3 for j in range(4)]
            inside = m <= rng
            r_in = px(m)
            if inside:
                pts = []
                for a in arc:
                    pts += xy(a, r_in)
                for a in reversed(arc):
                    pts += xy(a, R)
                c.create_polygon(pts, fill=self.SHADOW[col], outline="")
        for m, ang in obstacles:
            col = self.color(m)
            arc = [ang - inc / 2 + j * inc / 3 for j in range(4)]
            inside = m <= rng
            line = []
            for a in arc:
                line += xy(a, px(m) if inside else R + 5 * s)
            c.create_line(line, fill=col, width=max(2, int((6 if inside else 3) * s)), capstyle="round")

        self.draw_vehicle(cx, cy, 22 * s, self.kind(), s)

        # side letters on the ring
        for k, (name, deg) in enumerate((("F", 0), ("R", 90), ("B", 180), ("L", 270))):
            x, y = xy(deg, R + 22 * s)
            c.create_text(x, y, text=name, fill=self.TEXT, font=self.f(11 * s, True))
        if f.yaw_deg is not None:
            x, y = xy(-f.yaw_deg, R - 16 * s)  # inside the ring, clear of the F/R/B/L letters
            r = 10 * s
            c.create_oval(x - r, y - r, x + r, y + r, fill=self.ACCENT, outline="")
            c.create_text(x, y, text="N", fill="#0b1a2a", font=self.f(9 * s, True))

        # centre readout
        if stale:
            bw, bh = 200 * s, 54 * s
            c.create_rectangle(cx - bw, cy + R * 0.32, cx + bw, cy + R * 0.32 + bh, fill="#4a0f0c", outline=self.RED,
                               width=2)
            msg = "NO LIDAR DATA" if age is None else "NO LIDAR DATA  %.1f s" % age
            c.create_text(cx, cy + R * 0.32 + bh * 0.38, text=msg, fill="white", font=self.f(16 * s, True))
            c.create_text(cx, cy + R * 0.32 + bh * 0.78, text=status[:70], fill="#ffb3ad", font=self.f(8.5 * s))
        elif closest:
            c.create_text(cx, cy + R * 0.45, text="%.2f m" % closest[0], fill=self.color(closest[0]),
                          font=self.f(22 * s, True))
            c.create_text(cx, cy + R * 0.45 + 24 * s, text="closest, %s" % self.side_name(closest[1]),
                          fill=self.DIM, font=self.f(9 * s))
        else:
            c.create_text(cx, cy + R * 0.45, text="ALL CLEAR", fill=self.GREEN, font=self.f(20 * s, True))

        # zoom buttons
        bx_r = 17 * s
        for tag, label, x in (("zoom_out", "−", cx - R - 8 * s), ("zoom_in", "+", cx + R + 8 * s)):
            y = cy + R + 8 * s - bx_r
            c.create_oval(x - bx_r, y - bx_r, x + bx_r, y + bx_r, fill=self.CARD, outline=self.LINE, tags=tag)
            c.create_text(x, y, text=label, fill=self.TEXT, font=self.f(15 * s, True), tags=tag)

        # footer
        fy = H - fh
        c.create_rectangle(0, fy, W, H, fill=self.PANEL, outline="")
        c.create_line(0, fy, W, fy, fill=self.LINE)
        c.create_text(18 * s, fy + fh / 2, anchor="w", fill=self.DIM, font=self.f(9 * s),
                      text="range %g m   %.0f scans/s   CP_DIST %.1f m%s   %d free, %d unknown"
                      % (rng, rate, cp, "" if f.cp_dist else " (default)", free, unknown))
        c.create_text(W - 18 * s, fy + fh / 2, anchor="e", fill=self.ACCENT if self.topmost else self.DIM,
                      font=self.f(9 * s, True), text="ON TOP [T]" if self.topmost else "on top [T]", tags="pin")
        c.create_text(W - 120 * s, fy + fh / 2, anchor="e", fill=self.YELLOW if self.beeper.enabled else self.DIM,
                      font=self.f(9 * s, True), text="SOUND ON [M]" if self.beeper.enabled else "sound off [M]",
                      tags="sound")

        self.beeper.level = (max(0.0, min(1.0, 1 - closest[0] / cp))
                             if closest and not stale and closest[0] < cp else None)
        self.root.after(80, self.tick)

    def draw_vehicle(self, cx, cy, sz, kind, s):
        c, col, w = self.c, self.TEXT, max(2, int(2.5 * s))

        def rotors(n, a0, arm, rr):
            for k in range(n):
                a = math.radians(a0 + k * 360 / n)
                x, y = cx + arm * math.sin(a), cy - arm * math.cos(a)
                c.create_line(cx, cy, x, y, fill=col, width=w)
                c.create_oval(x - rr, y - rr, x + rr, y + rr, outline=col, width=w)
            c.create_oval(cx - sz * 0.22, cy - sz * 0.22, cx + sz * 0.22, cy + sz * 0.22, fill=col, outline="")

        if kind == "quad":
            rotors(4, 45, sz * 0.9, sz * 0.34)
        elif kind == "hexa":
            rotors(6, 30, sz * 0.95, sz * 0.28)
        elif kind == "octo":
            rotors(8, 22.5, sz, sz * 0.23)
        elif kind == "tri":
            rotors(3, 60, sz * 0.9, sz * 0.34)
        elif kind == "heli":
            c.create_oval(cx - sz, cy - sz, cx + sz, cy + sz, outline=col, width=w)
            c.create_line(cx, cy - sz * 0.35, cx, cy + sz * 1.2, fill=col, width=w)
            c.create_line(cx - sz * 0.28, cy + sz * 1.2, cx + sz * 0.28, cy + sz * 1.2, fill=col, width=w)
        elif kind in ("plane", "vtol"):
            c.create_line(cx, cy - sz * 1.05, cx, cy + sz, fill=col, width=w + 1)
            c.create_line(cx - sz * 1.15, cy - sz * 0.1, cx + sz * 1.15, cy - sz * 0.1, fill=col, width=w + 1)
            c.create_line(cx - sz * 0.45, cy + sz * 0.9, cx + sz * 0.45, cy + sz * 0.9, fill=col, width=w)
            if kind == "vtol":
                for dx in (-0.72, 0.72):
                    c.create_line(cx + dx * sz, cy - sz * 0.6, cx + dx * sz, cy + sz * 0.45, fill=col, width=w)
                    for dy in (-0.6, 0.45):
                        r = sz * 0.25
                        c.create_oval(cx + dx * sz - r, cy + dy * sz - r, cx + dx * sz + r, cy + dy * sz + r,
                                      outline=col, width=w)
        elif kind == "rover":
            c.create_rectangle(cx - sz * 0.45, cy - sz * 0.8, cx + sz * 0.45, cy + sz * 0.8, outline=col, width=w)
            for dx in (-0.62, 0.62):
                for dy in (-0.5, 0.5):
                    c.create_rectangle(cx + dx * sz - sz * 0.1, cy + dy * sz - sz * 0.22,
                                       cx + dx * sz + sz * 0.1, cy + dy * sz + sz * 0.22, fill=col, outline="")
        elif kind == "boat":
            c.create_polygon(cx, cy - sz, cx + sz * 0.45, cy - sz * 0.2, cx + sz * 0.4, cy + sz * 0.9,
                             cx - sz * 0.4, cy + sz * 0.9, cx - sz * 0.45, cy - sz * 0.2, outline=col, fill="", width=w)
        # flight controller front arrow
        c.create_polygon(cx, cy - sz * 1.65, cx - sz * 0.34, cy - sz * 1.15, cx + sz * 0.34, cy - sz * 1.15,
                         fill=self.ACCENT, outline="")

    @staticmethod
    def side_name(ang):
        names = ["front", "front-right", "right", "back-right", "back", "back-left", "left", "front-left"]
        return "%s %.0f°" % (names[int(((ang + 22.5) % 360) // 45)], ang)

    def run(self):
        self.root.mainloop()


def main():
    if sys.platform.startswith("win"):
        try:  # sharp text on high-DPI screens
            import ctypes
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except Exception:
            pass
    feed = Feed(sys.argv[1] if len(sys.argv) > 1 else None)
    feed.start()
    beeper = Beeper()
    beeper.start()
    App(feed, beeper).run()


if __name__ == "__main__":
    main()
