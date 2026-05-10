#!/usr/bin/env python3
"""Autogiro Sim control panel: live map, queued Nav2 goals, Nav2 launcher."""

import math
import os
import subprocess
import sys
import threading
import tkinter as tk
from dataclasses import dataclass
from tkinter import ttk
from typing import Optional

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
GOAL_REACHED_RADIUS_M = 0.35
DRAG_HEADING_THRESHOLD_PX = 5


@dataclass(frozen=True)
class Goal:
    x: float
    y: float
    yaw: float

    def summary(self) -> str:
        return f"x={self.x:.2f}, y={self.y:.2f}, yaw={math.degrees(self.yaw):.0f}°"


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
        self.goal_world: Optional[Goal] = None  # pending/preview goal
        self.goal_queue: list[Goal] = []
        self.active_goal: Optional[Goal] = None
        self.queue_running = False
        self.goal_reached_radius = GOAL_REACHED_RADIUS_M
        self._drag_start_canvas = None
        self.nav2_proc: Optional[subprocess.Popen] = None
        self.nav2_launch_count = 0
        self.nav2_error_message: Optional[str] = None

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
        self.canvas.bind("<B1-Motion>", self._on_drag)
        self.canvas.bind("<ButtonRelease-1>", self._on_release)
        self.canvas.bind("<Configure>", lambda _e: self._redraw())

        # Side panel
        side = ttk.Frame(body, style="Panel.TFrame", width=320)
        side.pack(side="right", fill="y", padx=(12, 0))
        side.pack_propagate(False)

        self._section(side, "Navigation")
        self.goal_label = ttk.Label(
            side,
            text="No goals queued.\nDrag on the map to add destinations.",
            style="Muted.TLabel",
            justify="left",
        )
        self.goal_label.pack(fill="x", padx=16, pady=(0, 8))

        btns = ttk.Frame(side, style="Panel.TFrame")
        btns.pack(fill="x", padx=16, pady=(0, 12))
        self.nav_btn = ttk.Button(btns, text="Start Queue", style="Accent.TButton",
                                  command=self._toggle_queue, state="disabled")
        self.nav_btn.pack(fill="x")
        ttk.Button(btns, text="Cancel Pending Drag", style="Ghost.TButton",
                   command=self._clear_goal).pack(fill="x", pady=(6, 0))

        self._section(side, "Nav2")
        self.nav2_btn = ttk.Button(side, text="Start Nav2 Stack", style="Accent.TButton",
                                   command=self._launch_nav2)
        self.nav2_btn.pack(fill="x", padx=16, pady=(0, 6))
        self.nav2_label = ttk.Label(side, text="Nav2 is not launched from this panel.",
                                    style="Muted.TLabel", justify="left")
        self.nav2_label.pack(fill="x", padx=16, pady=(0, 12))

        self._section(side, "Goal Queue")
        queue_frame = ttk.Frame(side, style="Panel.TFrame")
        queue_frame.pack(fill="x", padx=16, pady=(0, 12))
        self.queue_list = tk.Listbox(queue_frame, height=7, activestyle="none",
                                     bg=PANEL_HI, fg=FG, selectbackground=ACCENT,
                                     selectforeground="#0b1320", highlightthickness=0,
                                     borderwidth=0, exportselection=False)
        self.queue_list.pack(fill="x")
        self.queue_list.bind("<<ListboxSelect>>", self._on_queue_select)

        queue_btns = ttk.Frame(queue_frame, style="Panel.TFrame")
        queue_btns.pack(fill="x", pady=(6, 0))
        ttk.Button(queue_btns, text="↑", style="Ghost.TButton",
                   command=self._move_selected_goal_up).grid(
                       row=0, column=0, sticky="ew", padx=(0, 3))
        ttk.Button(queue_btns, text="↓", style="Ghost.TButton",
                   command=self._move_selected_goal_down).grid(row=0, column=1, sticky="ew", padx=3)
        ttk.Button(queue_btns, text="Remove", style="Ghost.TButton",
                   command=self._remove_selected_goal).grid(row=0, column=2, sticky="ew", padx=3)
        ttk.Button(queue_btns, text="Clear Queue", style="Ghost.TButton",
                   command=self._clear_queue).grid(row=0, column=3, sticky="ew", padx=(3, 0))
        for col in range(4):
            queue_btns.columnconfigure(col, weight=1)

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
        self._maybe_advance_queue(pose)
        self._refresh_nav2_state()
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
        for idx, goal in enumerate(self.goal_queue, start=1):
            self._draw_goal_marker(goal, ACCENT_HOT, str(idx))
        if self.active_goal is not None:
            self._draw_goal_marker(self.active_goal, OK, "▶")
        if self.goal_world is not None:
            self._draw_goal_marker(self.goal_world, ACCENT_HOT, "+")

    def _draw_goal_marker(self, goal: Goal, color, label):
        cx, cy = self._world_to_canvas(goal.x, goal.y)
        r = 9
        self.canvas.create_oval(cx - r, cy - r, cx + r, cy + r,
                                outline=color, width=2, tags="overlay")
        self.canvas.create_text(cx, cy, text=label, fill=color,
                                font=("TkDefaultFont", 9, "bold"), tags="overlay")
        arrow_len = 28
        ax = cx + math.cos(goal.yaw) * arrow_len
        ay = cy - math.sin(goal.yaw) * arrow_len
        self.canvas.create_line(cx, cy, ax, ay,
                                fill=color, width=2, arrow="last",
                                arrowshape=(10, 12, 4), tags="overlay")

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
        if self.map_info is None or not self._is_canvas_point_on_map(event.x, event.y):
            return
        wx, wy = self._canvas_to_world(event.x, event.y)
        self._drag_start_canvas = (event.x, event.y)
        self.goal_world = Goal(wx, wy, 0.0)
        self._set_navigation_help(
            f"Creating goal: x={wx:.2f}, y={wy:.2f}\n"
            "Drag before release to choose its heading. Release to add it to the queue.")
        self._draw_overlay(self.bridge.robot_pose)

    def _on_drag(self, event):
        if self.goal_world is None or self._drag_start_canvas is None:
            return
        sx, sy = self._drag_start_canvas
        dx, dy = event.x - sx, event.y - sy
        if math.hypot(dx, dy) < DRAG_HEADING_THRESHOLD_PX:
            return
        yaw = math.atan2(-dy, dx)
        self.goal_world = Goal(self.goal_world.x, self.goal_world.y, yaw)
        self._set_navigation_help(
            f"Creating goal: {self.goal_world.summary()}\n"
            "Release to queue it. Use Cancel Pending Drag to discard before release.")
        self._draw_overlay(self.bridge.robot_pose)

    def _on_release(self, event):
        if self.goal_world is None or self._drag_start_canvas is None:
            return
        sx, sy = self._drag_start_canvas
        dx, dy = event.x - sx, event.y - sy
        if math.hypot(dx, dy) < DRAG_HEADING_THRESHOLD_PX:
            pose = self.bridge.robot_pose
            yaw = (
                math.atan2(self.goal_world.y - pose[1], self.goal_world.x - pose[0])
                if pose else 0.0
            )
            self.goal_world = Goal(self.goal_world.x, self.goal_world.y, yaw)
        self._drag_start_canvas = None
        self._add_goal_to_queue(self.goal_world)
        self.goal_world = None
        self._draw_overlay(self.bridge.robot_pose)

    def _is_canvas_point_on_map(self, x, y):
        if self.map_img is None:
            return False
        iw, ih = self.map_img.size
        return (self.offset[0] <= x <= self.offset[0] + iw * self.scale and
                self.offset[1] <= y <= self.offset[1] + ih * self.scale)

    def _add_goal_to_queue(self, goal: Goal):
        self.goal_queue.append(goal)
        self._refresh_queue_display(select=len(self.goal_queue) - 1)
        self._set_navigation_help(
            f"Queued goal #{len(self.goal_queue)}: {goal.summary()}\n"
            "Drag on the map to add more, reorder below, then Start Queue.")
        self.status.config(text=f"Added goal #{len(self.goal_queue)} to queue.", foreground=OK)

    def _refresh_queue_display(self, select=None):
        self.queue_list.delete(0, tk.END)
        for idx, goal in enumerate(self.goal_queue, start=1):
            self.queue_list.insert(tk.END, f"{idx}.  {goal.summary()}")
        if select is not None and self.goal_queue:
            select = max(0, min(select, len(self.goal_queue) - 1))
            self.queue_list.selection_set(select)
            self.queue_list.activate(select)
        if self.queue_running:
            self.nav_btn.config(state="normal", text="Stop Queue")
        else:
            self.nav_btn.config(
                state="normal" if self.goal_queue else "disabled",
                text="Start Queue",
            )

    def _selected_queue_index(self):
        selection = self.queue_list.curselection()
        return selection[0] if selection else None

    def _on_queue_select(self, _event=None):
        idx = self._selected_queue_index()
        if idx is None:
            return
        goal = self.goal_queue[idx]
        self._set_navigation_help(
            f"Selected queued goal #{idx + 1}: {goal.summary()}\n"
            "Use ↑/↓ to reorder it, Remove to delete it, or Clear Queue to "
            "delete all queued goals.")

    def _move_selected_goal_up(self):
        idx = self._selected_queue_index()
        if idx is None or idx <= 0:
            return
        self.goal_queue[idx - 1], self.goal_queue[idx] = (
            self.goal_queue[idx], self.goal_queue[idx - 1])
        self._refresh_queue_display(select=idx - 1)
        self._draw_overlay(self.bridge.robot_pose)

    def _move_selected_goal_down(self):
        idx = self._selected_queue_index()
        if idx is None or idx >= len(self.goal_queue) - 1:
            return
        self.goal_queue[idx + 1], self.goal_queue[idx] = (
            self.goal_queue[idx], self.goal_queue[idx + 1])
        self._refresh_queue_display(select=idx + 1)
        self._draw_overlay(self.bridge.robot_pose)

    def _remove_selected_goal(self):
        idx = self._selected_queue_index()
        if idx is None:
            self._set_navigation_help("Select a queued goal first, then press Remove.")
            return
        removed = self.goal_queue.pop(idx)
        next_selection = min(idx, len(self.goal_queue) - 1) if self.goal_queue else None
        self._refresh_queue_display(select=next_selection)
        self._set_navigation_help(
            f"Removed queued goal: {removed.summary()}\n"
            "The active Nav2 goal, if any, is unchanged.")
        self._draw_overlay(self.bridge.robot_pose)

    def _clear_queue(self):
        if not self.goal_queue:
            self._set_navigation_help(
                "The queued-goals list is already empty.\n"
                "Drag on the map to add destinations.")
            return
        self.goal_queue.clear()
        self._refresh_queue_display()
        self._set_navigation_help(
            "Queued goals cleared.\n"
            "Active Nav2 goal, if any, is unchanged. Drag on the map to add more.")
        self._draw_overlay(self.bridge.robot_pose)

    def _clear_goal(self):
        if self.goal_world is None:
            self._set_navigation_help(
                "No pending drag to cancel.\n"
                "Queued goals are managed in the Goal Queue controls below.")
            return
        self.goal_world = None
        self._drag_start_canvas = None
        self._set_navigation_help("Pending drag canceled.\nQueued goals were not changed.")
        self._draw_overlay(self.bridge.robot_pose)

    def _toggle_queue(self):
        if self.queue_running:
            self.queue_running = False
            self.active_goal = None
            self._refresh_queue_display()
            self._set_navigation_help(
                "Queue stopped.\n"
                "Nav2 may continue toward the last sent goal; start the queue "
                "again for remaining goals.")
            self.status.config(
                text="Goal queue stopped. Current Nav2 goal may continue.",
                foreground=MUTED,
            )
            self._draw_overlay(self.bridge.robot_pose)
            return
        if not self.goal_queue:
            self._set_navigation_help(
                "Add at least one goal by dragging on the map before starting the queue.")
            return
        self.queue_running = True
        self._refresh_queue_display()
        self._send_next_goal()

    def _send_next_goal(self):
        if not self.goal_queue:
            self.queue_running = False
            self.active_goal = None
            self._refresh_queue_display()
            self._set_navigation_help("Queue complete.\nDrag on the map to add more goals.")
            self.status.config(text="Goal queue complete.", foreground=OK)
            self._draw_overlay(self.bridge.robot_pose)
            return
        self.active_goal = self.goal_queue.pop(0)
        self._refresh_queue_display(select=0 if self.goal_queue else None)
        goal = self.active_goal
        self.bridge.publish_goal(goal.x, goal.y, goal.yaw)
        self.status.config(
            text=f"Goal sent → ({goal.x:.2f}, {goal.y:.2f}) @ "
                 f"{math.degrees(goal.yaw):.0f}°",
            foreground=OK,
        )
        self._set_navigation_help(
            f"Driving to: {goal.summary()}\n"
            f"{len(self.goal_queue)} queued after this. Stop Queue pauses automatic advancement.")
        self._draw_overlay(self.bridge.robot_pose)

    def _maybe_advance_queue(self, pose):
        if not self.queue_running or self.active_goal is None or pose is None:
            return
        goal = self.active_goal
        if math.hypot(pose[0] - goal.x, pose[1] - goal.y) <= self.goal_reached_radius:
            self._send_next_goal()

    def _set_navigation_help(self, text):
        self.goal_label.config(text=text)

    def _launch_nav2(self):
        if self._nav2_is_running():
            self.status.config(text="Nav2 is already running in its xterm.", foreground=MUTED)
            return
        try:
            self.nav2_error_message = None
            self.nav2_proc = subprocess.Popen(self._nav2_command(), stderr=subprocess.DEVNULL)
        except OSError as exc:
            self.nav2_proc = None
            self.nav2_error_message = f"Could not launch Nav2: {exc}"
            self.nav2_btn.config(state="normal", text="Start Nav2 Stack")
            self.nav2_label.config(text=self.nav2_error_message)
            self.status.config(
                text="Could not launch Nav2 — check xterm/ROS environment.",
                foreground=ACCENT_HOT,
            )
            return
        self.nav2_launch_count += 1
        self._refresh_nav2_state()
        self.status.config(text="Nav2 launched — queue goals, then Start Queue.", foreground=OK)

    def _nav2_command(self):
        pkg_share_config = os.environ.get("AUTOGIRO_CONFIG_DIR", "")
        params = os.path.join(pkg_share_config, "nav2_params.yaml")
        ws_setup = os.environ.get("AUTOGIRO_WS_SETUP", "")
        return [
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

    def _nav2_is_running(self):
        return self.nav2_proc is not None and self.nav2_proc.poll() is None

    def _refresh_nav2_state(self):
        if self._nav2_is_running():
            self.nav2_btn.config(state="disabled", text="Nav2 Running")
            self.nav2_label.config(
                text=f"Running in xterm (PID {self.nav2_proc.pid}). "
                     "Close that window to relaunch.")
            return
        if self.nav2_proc is not None:
            code = self.nav2_proc.poll()
            self.nav2_proc = None
            self.nav2_btn.config(state="normal", text="Relaunch Nav2 Stack")
            self.nav2_label.config(text=f"Nav2 exited with code {code}. You can relaunch it here.")
            return
        if self.nav2_error_message:
            self.nav2_btn.config(state="normal", text="Start Nav2 Stack")
            self.nav2_label.config(text=self.nav2_error_message)
        elif self.nav2_launch_count:
            self.nav2_btn.config(state="normal", text="Relaunch Nav2 Stack")
            self.nav2_label.config(text="Nav2 is not running. You can relaunch it here.")
        else:
            self.nav2_btn.config(state="normal", text="Start Nav2 Stack")
            self.nav2_label.config(text="Nav2 is not launched from this panel yet.")


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
