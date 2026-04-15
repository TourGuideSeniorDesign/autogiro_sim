#!/usr/bin/env python3
"""Autogiro Sim control panel: live map, click-to-navigate, Nav2 launcher."""

import math
import os
import subprocess
import sys
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener, TransformException

from PIL import Image, ImageTk


BG = "#1e1f24"
PANEL = "#272930"
PANEL_HI = "#31343d"
FG = "#e6e8ee"
MUTED = "#8a8f9c"
ACCENT = "#5aa9ff"
ACCENT_HOT = "#ff7a59"
OK = "#6dd38c"

FREE_RGB = (230, 232, 238)
OCC_RGB = (18, 19, 23)
UNK_RGB = (55, 58, 66)

MAP_FRAME = "map"
ROBOT_FRAME = "base_link"


KEYBOARD_LEGEND = [
    ("i / ,", "forward / backward"),
    ("j / l", "rotate left / right"),
    ("u o m .", "diagonal / arc motion"),
    ("k / K", "stop"),
    ("q / z", "all speed +/- 10%"),
    ("w / x", "linear +/- 10%"),
    ("e / c", "angular +/- 10%"),
]


class RosBridge(Node):
    def __init__(self):
        super().__init__("autogiro_gui")
        self.lock = threading.Lock()
        self.map_msg = None
        self.map_version = 0
        self.robot_pose = None  # (x, y, yaw) in map frame

        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.1, self._poll_tf)

    def _on_map(self, msg: OccupancyGrid):
        with self.lock:
            self.map_msg = msg
            self.map_version += 1

    def _poll_tf(self):
        try:
            tf = self.tf_buffer.lookup_transform(MAP_FRAME, ROBOT_FRAME, rclpy.time.Time())
        except TransformException:
            return
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self.lock:
            self.robot_pose = (t.x, t.y, yaw)

    def publish_goal(self, x: float, y: float, yaw: float):
        msg = PoseStamped()
        msg.header.frame_id = MAP_FRAME
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self.goal_pub.publish(msg)


def occgrid_to_image(msg: OccupancyGrid) -> Image.Image:
    w, h = msg.info.width, msg.info.height
    data = bytes(bytearray(
        UNK_RGB[0] if v < 0 else (OCC_RGB[0] if v >= 50 else FREE_RGB[0])
        for v in msg.data
    ))
    # Build grayscale then paletteize — faster than per-pixel RGB in pure Python.
    img = Image.frombytes("L", (w, h), data)
    rgb = Image.new("RGB", (w, h))
    # Map 3 unique grays back to RGB (they happen to be grayscale triples anyway).
    rgb.paste(img.convert("RGB"))
    # OccupancyGrid origin is bottom-left; Tk canvas is top-left.
    return rgb.transpose(Image.FLIP_TOP_BOTTOM)


class App:
    def __init__(self, root: tk.Tk, bridge: RosBridge):
        self.root = root
        self.bridge = bridge
        self.photo = None
        self.map_img = None
        self.map_version_shown = -1
        self.map_info = None  # (res, origin_x, origin_y, width_px, height_px)
        self.scale = 1.0  # canvas_px / map_px
        self.offset = (0, 0)  # (dx, dy) canvas pixel offset where map image starts
        self.goal_world = None  # (x, y)
        self.nav2_launched = False

        self._build()

    # ---------- layout ----------

    def _build(self):
        self.root.title("Autogiro Sim — Control Panel")
        self.root.geometry("1180x760")
        self.root.configure(bg=BG)

        style = ttk.Style()
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("TFrame", background=BG)
        style.configure("Panel.TFrame", background=PANEL)
        style.configure("TLabel", background=BG, foreground=FG)
        style.configure("Panel.TLabel", background=PANEL, foreground=FG)
        style.configure("Muted.TLabel", background=PANEL, foreground=MUTED)
        style.configure("Heading.TLabel", background=PANEL, foreground=FG,
                        font=("TkDefaultFont", 11, "bold"))
        style.configure("Title.TLabel", background=BG, foreground=FG,
                        font=("TkDefaultFont", 14, "bold"))
        style.configure("Accent.TButton", background=ACCENT, foreground="#0b1320",
                        borderwidth=0, focusthickness=0, padding=(14, 8),
                        font=("TkDefaultFont", 10, "bold"))
        style.map("Accent.TButton",
                  background=[("active", "#7fbcff"), ("disabled", "#3c4656")],
                  foreground=[("disabled", MUTED)])
        style.configure("Ghost.TButton", background=PANEL_HI, foreground=FG,
                        borderwidth=0, padding=(12, 6))
        style.map("Ghost.TButton", background=[("active", "#3c4048")])

        root = self.root

        header = ttk.Frame(root)
        header.pack(fill="x", padx=16, pady=(14, 6))
        ttk.Label(header, text="Autogiro Control Panel", style="Title.TLabel").pack(side="left")
        self.status = ttk.Label(header, text="Waiting for /map …", foreground=MUTED,
                                background=BG)
        self.status.pack(side="right")

        body = ttk.Frame(root)
        body.pack(fill="both", expand=True, padx=16, pady=10)

        # Map canvas
        map_card = ttk.Frame(body, style="Panel.TFrame")
        map_card.pack(side="left", fill="both", expand=True)
        self.canvas = tk.Canvas(map_card, bg=PANEL, highlightthickness=0,
                                cursor="crosshair")
        self.canvas.pack(fill="both", expand=True, padx=10, pady=10)
        self.canvas.bind("<Button-1>", self._on_click)
        self.canvas.bind("<Configure>", lambda _e: self._redraw())

        # Side panel
        side = ttk.Frame(body, style="Panel.TFrame", width=320)
        side.pack(side="right", fill="y", padx=(12, 0))
        side.pack_propagate(False)

        self._section(side, "Navigation")
        self.goal_label = ttk.Label(side, text="No goal placed.\nClick the map to set one.",
                                    style="Muted.TLabel", justify="left")
        self.goal_label.pack(fill="x", padx=16, pady=(0, 8))

        btns = ttk.Frame(side, style="Panel.TFrame")
        btns.pack(fill="x", padx=16, pady=(0, 12))
        self.nav_btn = ttk.Button(btns, text="Navigate to Goal", style="Accent.TButton",
                                  command=self._send_goal, state="disabled")
        self.nav_btn.pack(fill="x")
        ttk.Button(btns, text="Clear Goal", style="Ghost.TButton",
                   command=self._clear_goal).pack(fill="x", pady=(6, 0))

        self._section(side, "Nav2")
        self.nav2_btn = ttk.Button(side, text="Start Nav2 Stack", style="Accent.TButton",
                                   command=self._launch_nav2)
        self.nav2_btn.pack(fill="x", padx=16, pady=(0, 12))

        self._section(side, "Keyboard (teleop window)")
        legend = ttk.Frame(side, style="Panel.TFrame")
        legend.pack(fill="x", padx=16, pady=(0, 12))
        for k, desc in KEYBOARD_LEGEND:
            row = ttk.Frame(legend, style="Panel.TFrame")
            row.pack(fill="x", pady=1)
            ttk.Label(row, text=k, width=10, style="Panel.TLabel",
                      font=("TkFixedFont", 10, "bold")).pack(side="left")
            ttk.Label(row, text=desc, style="Muted.TLabel").pack(side="left")

        self.root.after(100, self._tick)

    def _section(self, parent, title):
        ttk.Label(parent, text=title, style="Heading.TLabel").pack(
            anchor="w", padx=16, pady=(14, 6))

    # ---------- map rendering ----------

    def _tick(self):
        with self.bridge.lock:
            mv = self.bridge.map_version
            msg = self.bridge.map_msg
            pose = self.bridge.robot_pose

        if msg is not None and mv != self.map_version_shown:
            self.map_img = occgrid_to_image(msg)
            self.map_info = (msg.info.resolution,
                             msg.info.origin.position.x,
                             msg.info.origin.position.y,
                             msg.info.width, msg.info.height)
            self.map_version_shown = mv
            self.status.config(text=f"Map {msg.info.width}×{msg.info.height} "
                                    f"@ {msg.info.resolution:.3f} m/px")
            self._redraw()
        self._draw_overlay(pose)
        self.root.after(100, self._tick)

    def _redraw(self):
        self.canvas.delete("all")
        if self.map_img is None:
            self.canvas.create_text(
                self.canvas.winfo_width() // 2 or 400,
                self.canvas.winfo_height() // 2 or 300,
                text="Waiting for /map …\nDrive the robot to build one.",
                fill=MUTED, font=("TkDefaultFont", 12), justify="center")
            return
        cw = max(self.canvas.winfo_width(), 2)
        ch = max(self.canvas.winfo_height(), 2)
        iw, ih = self.map_img.size
        scale = min(cw / iw, ch / ih)
        new_w = max(int(iw * scale), 1)
        new_h = max(int(ih * scale), 1)
        resized = self.map_img.resize((new_w, new_h), Image.NEAREST)
        self.photo = ImageTk.PhotoImage(resized)
        dx = (cw - new_w) // 2
        dy = (ch - new_h) // 2
        self.scale = scale
        self.offset = (dx, dy)
        self.canvas.create_image(dx, dy, image=self.photo, anchor="nw", tags="map")

    def _draw_overlay(self, pose):
        self.canvas.delete("overlay")
        if self.map_info is None:
            return
        if pose is not None:
            x, y, yaw = pose
            px, py = self._world_to_canvas(x, y)
            r = 8
            tip = (px + math.cos(-yaw) * r * 1.6, py + math.sin(-yaw) * r * 1.6)
            left = (px + math.cos(-yaw + 2.4) * r, py + math.sin(-yaw + 2.4) * r)
            right = (px + math.cos(-yaw - 2.4) * r, py + math.sin(-yaw - 2.4) * r)
            self.canvas.create_polygon(tip[0], tip[1], left[0], left[1],
                                       right[0], right[1],
                                       fill=ACCENT, outline="#0b1320", width=1,
                                       tags="overlay")
        if self.goal_world is not None:
            gx, gy = self.goal_world
            cx, cy = self._world_to_canvas(gx, gy)
            r = 9
            self.canvas.create_oval(cx - r, cy - r, cx + r, cy + r,
                                    outline=ACCENT_HOT, width=2, tags="overlay")
            self.canvas.create_line(cx - r - 4, cy, cx + r + 4, cy,
                                    fill=ACCENT_HOT, tags="overlay")
            self.canvas.create_line(cx, cy - r - 4, cx, cy + r + 4,
                                    fill=ACCENT_HOT, tags="overlay")

    # ---------- coordinate transforms ----------

    def _world_to_canvas(self, x, y):
        res, ox, oy, _w, h = self.map_info
        mx = (x - ox) / res
        my = (y - oy) / res
        # image is flipped vertically
        img_x = mx
        img_y = (h - 1) - my
        return (self.offset[0] + img_x * self.scale,
                self.offset[1] + img_y * self.scale)

    def _canvas_to_world(self, cx, cy):
        res, ox, oy, _w, h = self.map_info
        img_x = (cx - self.offset[0]) / self.scale
        img_y = (cy - self.offset[1]) / self.scale
        mx = img_x
        my = (h - 1) - img_y
        return (mx * res + ox, my * res + oy)

    # ---------- actions ----------

    def _on_click(self, event):
        if self.map_info is None:
            return
        # Reject clicks outside the map image.
        iw, ih = self.map_img.size
        if not (self.offset[0] <= event.x <= self.offset[0] + iw * self.scale and
                self.offset[1] <= event.y <= self.offset[1] + ih * self.scale):
            return
        wx, wy = self._canvas_to_world(event.x, event.y)
        self.goal_world = (wx, wy)
        self.goal_label.config(text=f"Goal: x = {wx:.2f} m,  y = {wy:.2f} m")
        self.nav_btn.config(state="normal")
        self._draw_overlay(self.bridge.robot_pose)

    def _clear_goal(self):
        self.goal_world = None
        self.goal_label.config(text="No goal placed.\nClick the map to set one.")
        self.nav_btn.config(state="disabled")
        self._draw_overlay(self.bridge.robot_pose)

    def _send_goal(self):
        if self.goal_world is None:
            return
        gx, gy = self.goal_world
        pose = self.bridge.robot_pose
        yaw = math.atan2(gy - pose[1], gx - pose[0]) if pose else 0.0
        self.bridge.publish_goal(gx, gy, yaw)
        self.status.config(text=f"Goal sent → ({gx:.2f}, {gy:.2f})", foreground=OK)

    def _launch_nav2(self):
        pkg_share_config = os.environ.get("AUTOGIRO_CONFIG_DIR", "")
        params = os.path.join(pkg_share_config, "nav2_params.yaml")
        ws_setup = os.environ.get("AUTOGIRO_WS_SETUP", "")
        cmd = [
            "xterm",
            "-fa", "DejaVu Sans Mono", "-fs", "11",
            "-title", "Nav2",
            "-geometry", "160x48",
            "-sb", "-rightbar", "-sl", "50000",
            "-bg", "#101217", "-fg", "#e6e8ee",
            "-xrm", "XTerm*selectToClipboard: true",
            "-xrm", "XTerm*trimSelection: true",
            "-xrm", "XTerm*cutNewline: false",
            "-xrm", "XTerm*on3Clicks: regex ^.*$",
            "-hold", "-e",
            "bash", "-lc",
            f"source /opt/ros/humble/setup.bash && "
            f"[ -f \"{ws_setup}\" ] && source \"{ws_setup}\"; "
            f"ros2 launch nav2_bringup navigation_launch.py "
            f"use_sim_time:=True params_file:={params}"
        ]
        subprocess.Popen(cmd, stderr=subprocess.DEVNULL)
        self.nav2_launched = True
        self.nav2_btn.config(state="disabled", text="Nav2 launched")
        self.status.config(text="Nav2 launched — place a goal on the map.",
                           foreground=OK)


def main():
    rclpy.init()
    bridge = RosBridge()

    spin_thread = threading.Thread(target=rclpy.spin, args=(bridge,), daemon=True)
    spin_thread.start()

    try:
        root = tk.Tk()
    except tk.TclError as e:
        print(f"Could not open GUI (no display?): {e}", file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)

    App(root, bridge)
    try:
        root.mainloop()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
