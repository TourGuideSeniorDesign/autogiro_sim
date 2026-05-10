#!/usr/bin/env python3
"""Autogiro Sim control panel: live map, queued Nav2 goals, Nav2 launcher."""

import math
import os
import subprocess
import sys
import threading
import tkinter as tk
from dataclasses import dataclass
from tkinter import messagebox, ttk
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
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
SEPARATOR = "#3a3d47"


@dataclass(frozen=True)
class Goal:
    x: float
    y: float
    yaw: float

    def summary(self) -> str:
        return f"x={self.x:.2f}, y={self.y:.2f}, yaw={math.degrees(self.yaw):.0f}°"


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
        self.nav_client = ActionClient(self, NavigateThroughPoses, "navigate_through_poses")
        self._route_goal_handle = None

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

    def make_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = MAP_FRAME
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        return msg

    def send_route(self, goals, done_cb, feedback_cb=None) -> bool:
        if not self.nav_client.wait_for_server(timeout_sec=0.1):
            return False
        action_goal = NavigateThroughPoses.Goal()
        action_goal.poses = [self.make_pose(goal.x, goal.y, goal.yaw) for goal in goals]
        future = self.nav_client.send_goal_async(
            action_goal,
            feedback_callback=feedback_cb,
        )
        future.add_done_callback(lambda f: self._on_route_goal_response(f, done_cb))
        return True

    def cancel_route(self):
        if self._route_goal_handle is not None:
            self._route_goal_handle.cancel_goal_async()
            self._route_goal_handle = None

    def _on_route_goal_response(self, future, done_cb):
        goal_handle = future.result()
        if not goal_handle.accepted:
            done_cb(False, "Route was rejected by Nav2.")
            return
        self._route_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self._on_route_result(f, done_cb))

    def _on_route_result(self, future, done_cb):
        result = future.result()
        self._route_goal_handle = None
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            done_cb(True, "Route complete.")
        elif result.status == GoalStatus.STATUS_CANCELED:
            done_cb(False, "Route canceled.")
        else:
            done_cb(False, f"Route failed with status {result.status}.")


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
        self.root.geometry("1360x800")
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
        style.configure("Coord.TLabel", background=PANEL, foreground=ACCENT,
                        font=("Courier", 10))
        style.configure("Readout.TLabel", background=PANEL, foreground=ACCENT,
                        font=("Courier", 11))

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
        self.canvas.bind("<Motion>", self._on_motion)
        self.canvas.bind("<Leave>", lambda _e: self.coord_label.config(text=""))
        self.coord_label = ttk.Label(map_card, text="", style="Coord.TLabel")
        self.coord_label.pack(fill="x", padx=10, pady=(0, 6))

        self.root.bind("<Delete>", lambda _e: self._remove_selected_goal())
        self.root.bind("<BackSpace>", lambda _e: self._remove_selected_goal())

        # Side panel
        side = ttk.Frame(body, style="Panel.TFrame", width=400)
        side.pack(side="right", fill="y", padx=(12, 0))
        side.pack_propagate(False)

        self._section(side, "Robot")
        self.robot_info = ttk.Label(side, text="Waiting for TF …",
                                     style="Readout.TLabel")
        self.robot_info.pack(fill="x", padx=16, pady=(0, 10))
        ttk.Separator(side, orient="horizontal").pack(fill="x", padx=16, pady=(0, 2))

        self._section(side, "Navigation")
        self.goal_label = ttk.Label(
            side,
            text="Route is empty.\nDrag on the map to add a destination.",
            style="Muted.TLabel",
            justify="left",
            wraplength=360,
        )
        self.goal_label.pack(fill="x", padx=16, pady=(0, 8))

        btns = ttk.Frame(side, style="Panel.TFrame")
        btns.pack(fill="x", padx=16, pady=(0, 12))
        self.nav_btn = ttk.Button(btns, text="Start Route", style="Accent.TButton",
                                  command=self._toggle_queue, state="disabled")
        self.nav_btn.pack(fill="x")

        ttk.Separator(side, orient="horizontal").pack(fill="x", padx=16, pady=(8, 2))

        self._section(side, "Nav2")
        self.nav2_btn = ttk.Button(side, text="Launch Nav2", style="Accent.TButton",
                                   command=self._launch_nav2)
        self.nav2_btn.pack(fill="x", padx=16, pady=(0, 6))
        self.nav2_label = ttk.Label(
            side,
            text="Nav2 is not running from this panel.",
            style="Muted.TLabel",
            justify="left",
            wraplength=360,
        )
        self.nav2_label.pack(fill="x", padx=16, pady=(0, 12))

        ttk.Separator(side, orient="horizontal").pack(fill="x", padx=16, pady=(0, 2))

        self._section(side, "Route")
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
        ttk.Button(queue_btns, text="Move Up", style="Ghost.TButton",
                   command=self._move_selected_goal_up).grid(
                       row=0, column=0, sticky="ew", padx=(0, 3), pady=(0, 6))
        ttk.Button(queue_btns, text="Move Down", style="Ghost.TButton",
                   command=self._move_selected_goal_down).grid(
                       row=0, column=1, sticky="ew", padx=(3, 0), pady=(0, 6))
        ttk.Button(queue_btns, text="Remove", style="Ghost.TButton",
                   command=self._remove_selected_goal).grid(
                       row=1, column=0, sticky="ew", padx=(0, 3))
        ttk.Button(queue_btns, text="Clear All", style="Ghost.TButton",
                   command=self._clear_queue).grid(
                       row=1, column=1, sticky="ew", padx=(3, 0))
        for col in range(2):
            queue_btns.columnconfigure(col, weight=1)

        self.route_info = ttk.Label(side, text="", style="Muted.TLabel")
        self.route_info.pack(fill="x", padx=16, pady=(6, 0))

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
        self._update_robot_info(pose)
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
        self._draw_scale_bar()

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

    # ---------- info overlays ----------

    def _on_motion(self, event):
        if self.map_info is None or not self._is_canvas_point_on_map(event.x, event.y):
            self.coord_label.config(text="")
            return
        wx, wy = self._canvas_to_world(event.x, event.y)
        self.coord_label.config(text=f"x {wx:+.3f}   y {wy:+.3f}")

    def _update_robot_info(self, pose):
        if pose is None:
            self.robot_info.config(text="Waiting for TF …")
            return
        x, y, yaw = pose
        self.robot_info.config(
            text=f"x {x:+7.3f}   y {y:+7.3f}   θ {math.degrees(yaw):+6.1f}°")

    def _update_route_info(self):
        if not self.goal_queue:
            self.route_info.config(text="")
            return
        total = 0.0
        for i in range(1, len(self.goal_queue)):
            a, b = self.goal_queue[i - 1], self.goal_queue[i]
            total += math.hypot(b.x - a.x, b.y - a.y)
        n = len(self.goal_queue)
        self.route_info.config(
            text=f"{n} waypoint{'s' if n != 1 else ''}  ·  {total:.2f} m path")

    def _draw_scale_bar(self):
        if self.map_info is None:
            return
        res = self.map_info[0]
        px_per_m = self.scale / res
        bar_m = 1.0
        for candidate in (0.25, 0.5, 1.0, 2.0, 5.0, 10.0):
            bar_m = candidate
            if 40 <= px_per_m * candidate <= 160:
                break
        bar_px = px_per_m * bar_m
        if bar_px < 10:
            return
        cw = self.canvas.winfo_width()
        ch = self.canvas.winfo_height()
        x0 = cw - bar_px - 16
        y0 = ch - 18
        self.canvas.create_line(x0, y0, x0 + bar_px, y0,
                                fill=MUTED, width=2, tags="overlay")
        self.canvas.create_line(x0, y0 - 3, x0, y0 + 3,
                                fill=MUTED, width=1, tags="overlay")
        self.canvas.create_line(x0 + bar_px, y0 - 3, x0 + bar_px, y0 + 3,
                                fill=MUTED, width=1, tags="overlay")
        self.canvas.create_text(x0 + bar_px / 2, y0 - 10, text=f"{bar_m:g} m",
                                fill=MUTED, font=("TkDefaultFont", 9), tags="overlay")

    # ---------- actions ----------

    def _on_click(self, event):
        if self.map_info is None or not self._is_canvas_point_on_map(event.x, event.y):
            return
        wx, wy = self._canvas_to_world(event.x, event.y)
        self._drag_start_canvas = (event.x, event.y)
        self.goal_world = Goal(wx, wy, 0.0)
        self._set_navigation_help(
            f"Draft goal: x={wx:.2f}, y={wy:.2f}\n"
            "Drag to set heading, then release to add it to the route.")
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
            f"Draft goal: {self.goal_world.summary()}\n"
            "Release to add it to the route.")
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
            f"Added destination #{len(self.goal_queue)}: {goal.summary()}\n"
            "Add more, reorder the route, or press Start Route.")
        self.status.config(text=f"Added destination #{len(self.goal_queue)}.", foreground=OK)

    def _refresh_queue_display(self, select=None):
        self.queue_list.delete(0, tk.END)
        for idx, goal in enumerate(self.goal_queue, start=1):
            self.queue_list.insert(tk.END, f"{idx}.  {goal.summary()}")
        if select is not None and self.goal_queue:
            select = max(0, min(select, len(self.goal_queue) - 1))
            self.queue_list.selection_set(select)
            self.queue_list.activate(select)
        if self.queue_running:
            self.nav_btn.config(state="normal", text="Pause Route")
        else:
            self.nav_btn.config(
                state="normal" if self.goal_queue else "disabled",
                text="Start Route",
            )
        self._update_route_info()

    def _selected_queue_index(self):
        selection = self.queue_list.curselection()
        return selection[0] if selection else None

    def _on_queue_select(self, _event=None):
        idx = self._selected_queue_index()
        if idx is None:
            return
        goal = self.goal_queue[idx]
        self._set_navigation_help(
            f"Selected destination #{idx + 1}: {goal.summary()}\n"
            "Use Move Up/Down to reorder, Remove to delete, or Clear All to reset.")

    def _move_selected_goal_up(self):
        if self.queue_running:
            self._set_navigation_help("Pause the route before reordering destinations.")
            return
        idx = self._selected_queue_index()
        if idx is None or idx <= 0:
            return
        self.goal_queue[idx - 1], self.goal_queue[idx] = (
            self.goal_queue[idx], self.goal_queue[idx - 1])
        self._refresh_queue_display(select=idx - 1)
        self._draw_overlay(self.bridge.robot_pose)

    def _move_selected_goal_down(self):
        if self.queue_running:
            self._set_navigation_help("Pause the route before reordering destinations.")
            return
        idx = self._selected_queue_index()
        if idx is None or idx >= len(self.goal_queue) - 1:
            return
        self.goal_queue[idx + 1], self.goal_queue[idx] = (
            self.goal_queue[idx], self.goal_queue[idx + 1])
        self._refresh_queue_display(select=idx + 1)
        self._draw_overlay(self.bridge.robot_pose)

    def _remove_selected_goal(self):
        if self.queue_running:
            self._set_navigation_help("Pause the route before removing destinations.")
            return
        idx = self._selected_queue_index()
        if idx is None:
            self._set_navigation_help("Select a destination first, then press Remove.")
            return
        removed = self.goal_queue.pop(idx)
        next_selection = min(idx, len(self.goal_queue) - 1) if self.goal_queue else None
        self._refresh_queue_display(select=next_selection)
        self._set_navigation_help(
            f"Removed destination: {removed.summary()}\n"
            "The current Nav2 goal was not changed.")
        self._draw_overlay(self.bridge.robot_pose)

    def _clear_queue(self):
        if self.queue_running:
            self._set_navigation_help("Pause the route before clearing saved destinations.")
            return
        if not self.goal_queue:
            self._set_navigation_help(
                "The route is already empty.\n"
                "Drag on the map to add a destination.")
            return
        self.goal_queue.clear()
        self._refresh_queue_display()
        self._set_navigation_help(
            "Route cleared.\n"
            "Drag on the map to build a new route.")
        self._draw_overlay(self.bridge.robot_pose)

    def _toggle_queue(self):
        if self.queue_running:
            self._pause_route()
            return
        if not self.goal_queue:
            self._set_navigation_help(
                "Add at least one destination before starting the route.")
            return
        self._start_next_route_stop()

    def _start_next_route_stop(self):
        if not self.goal_queue:
            self.queue_running = False
            self.active_goal = None
            self._refresh_queue_display()
            self._set_navigation_help("Route complete.\nDrag on the map to plan another route.")
            self.status.config(text="Route complete.", foreground=OK)
            self._draw_overlay(self.bridge.robot_pose)
            return
        next_goal = self.goal_queue[0]
        if not self.bridge.send_route(
            [next_goal],
            done_cb=lambda ok, msg: self.root.after(0, self._on_route_done, ok, msg),
            feedback_cb=self._on_route_feedback,
        ):
            self.queue_running = False
            self.active_goal = None
            self._set_navigation_help(
                "Nav2 is not ready yet.\nLaunch Nav2 and wait for the action server, then try again.")
            self.status.config(text="Nav2 action server is not ready.", foreground=ACCENT_HOT)
            self._refresh_queue_display(select=0)
            self._draw_overlay(self.bridge.robot_pose)
            return
        self.queue_running = True
        self.active_goal = next_goal
        self._refresh_queue_display(select=0)
        self._set_navigation_help(
            f"Navigating to next destination: {next_goal.summary()}\n"
            "The route will pause there and ask before continuing.")
        self.status.config(text="Navigating to next route destination.", foreground=OK)
        self._draw_overlay(self.bridge.robot_pose)

    def _pause_route(self):
        self.bridge.cancel_route()
        self.queue_running = False
        self.active_goal = None
        self._refresh_queue_display(select=0 if self.goal_queue else None)
        self._set_navigation_help(
            "Route paused.\n"
            "The Nav2 action was canceled. Press Start Route to continue from the next destination.")
        self.status.config(text="Route paused.", foreground=MUTED)
        self._draw_overlay(self.bridge.robot_pose)

    def _on_route_feedback(self, feedback_msg):
        feedback = feedback_msg.feedback
        remaining = getattr(feedback, "number_of_poses_remaining", None)
        if remaining is None:
            return
        self.root.after(0, self._update_route_progress, remaining)

    def _update_route_progress(self, remaining):
        if not self.queue_running:
            return
        self.active_goal = self.goal_queue[0] if self.goal_queue else None
        self.status.config(text="Navigating to next route destination.")
        self._draw_overlay(self.bridge.robot_pose)

    def _on_route_done(self, ok, message):
        if not self.queue_running and message == "Route canceled.":
            return
        reached_goal = self.active_goal
        self.queue_running = False
        self.active_goal = None
        if ok and reached_goal is not None:
            self._refresh_queue_display(select=0)
            self._draw_overlay(self.bridge.robot_pose)
            self.status.config(text="Destination reached. Route paused.", foreground=OK)
            remaining_after_this = max(0, len(self.goal_queue) - 1)
            continue_route = messagebox.askyesno(
                "Destination reached",
                f"Reached destination: {reached_goal.summary()}\n\n"
                f"Continue to the next stop? ({remaining_after_this} remaining after this)",
                parent=self.root,
            )
            if continue_route:
                if self.goal_queue and self.goal_queue[0] == reached_goal:
                    self.goal_queue.pop(0)
                elif reached_goal in self.goal_queue:
                    self.goal_queue.remove(reached_goal)
                self._refresh_queue_display(select=0 if self.goal_queue else None)
                if self.goal_queue:
                    self._start_next_route_stop()
                else:
                    self._set_navigation_help("Route complete.\nDrag on the map to plan another route.")
                    self.status.config(text="Route complete.", foreground=OK)
                    self._draw_overlay(self.bridge.robot_pose)
            else:
                self._set_navigation_help(
                    "Route paused at the reached destination.\n"
                    "Press Start Route to retry/continue when ready.")
                self._refresh_queue_display(select=0 if self.goal_queue else None)
                self._draw_overlay(self.bridge.robot_pose)
            return
        if ok:
            self._set_navigation_help("Route complete.\nDrag on the map to plan another route.")
            self.status.config(text=message, foreground=OK)
        else:
            self._set_navigation_help(
                f"{message}\nThe saved route is still available to edit or retry.")
            self.status.config(text=message, foreground=ACCENT_HOT)
        self._refresh_queue_display(select=0 if self.goal_queue else None)
        self._draw_overlay(self.bridge.robot_pose)

    def _set_navigation_help(self, text):
        self.goal_label.config(text=text)

    def _launch_nav2(self):
        if self._nav2_is_running():
            self.status.config(text="Nav2 is already running.", foreground=MUTED)
            return
        try:
            self.nav2_error_message = None
            self.nav2_proc = subprocess.Popen(self._nav2_command(), stderr=subprocess.DEVNULL)
        except OSError as exc:
            self.nav2_proc = None
            self.nav2_error_message = f"Nav2 launch failed: {exc}"
            self.nav2_btn.config(state="normal", text="Launch Nav2")
            self.nav2_label.config(text=self.nav2_error_message)
            self.status.config(
                text="Nav2 launch failed. Check xterm and the ROS environment.",
                foreground=ACCENT_HOT,
            )
            return
        self.nav2_launch_count += 1
        self._refresh_nav2_state()
        self.status.config(
            text="Nav2 launched. Add destinations, then start the route.",
            foreground=OK,
        )

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
                text=f"Running in xterm, PID {self.nav2_proc.pid}. "
                     "Close the Nav2 window to relaunch.")
            return
        if self.nav2_proc is not None:
            code = self.nav2_proc.poll()
            self.nav2_proc = None
            self.nav2_btn.config(state="normal", text="Relaunch Nav2")
            self.nav2_label.config(text=f"Nav2 exited with code {code}. Relaunch when ready.")
            return
        if self.nav2_error_message:
            self.nav2_btn.config(state="normal", text="Launch Nav2")
            self.nav2_label.config(text=self.nav2_error_message)
        elif self.nav2_launch_count:
            self.nav2_btn.config(state="normal", text="Relaunch Nav2")
            self.nav2_label.config(text="Nav2 is not running. Relaunch when ready.")
        else:
            self.nav2_btn.config(state="normal", text="Launch Nav2")
            self.nav2_label.config(text="Nav2 is not running from this panel.")


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
