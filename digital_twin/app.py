"""Interactive Digital Twin GUI (Tkinter telemetry/control + PyBullet 3D glove)."""

from __future__ import annotations

import argparse
import math
import tkinter as tk
from tkinter import ttk
from typing import Optional

from .glove_model import FINGER_NAMES, HAS_PYBULLET, PyBulletGloveTwin
from .pneumatics import FingerKinematics, PneumaticParameters
from .serial_bridge import list_serial_ports
from .sync_engine import DigitalTwinEngine, SyncSnapshot


class DigitalTwinApp:
    """Real-time Digital Twin desktop interface."""

    def __init__(self, port: str = "SIM", use_pybullet: bool = True, auto_connect: bool = True) -> None:
        self.engine = DigitalTwinEngine(PneumaticParameters())
        self.glove_3d: Optional[PyBulletGloveTwin] = None
        if use_pybullet and HAS_PYBULLET:
            try:
                self.glove_3d = PyBulletGloveTwin(gui=True)
            except Exception:
                self.glove_3d = None

        self.root = tk.Tk()
        self.root.title("Textile Robotic Glove — Pneumatic Digital Twin (STM32U545)")
        self.root.geometry("1040x700")
        self.root.configure(bg="#11161f")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self.port_var = tk.StringVar(value=port)
        self.target_kpa_var = tk.DoubleVar(value=60.0)
        self.curl_pct_var = tk.DoubleVar(value=0.0)
        self.pwm1_var = tk.IntVar(value=0)
        self.pwm2_var = tk.IntVar(value=0)
        self.all_fingers_var = tk.BooleanVar(value=False)
        self.last_event_count = 0

        self._build_ui()
        self.root.bind("<space>", lambda _e: self._reset_3d_camera("default"))
        self.root.bind("<Key-r>", lambda _e: self._reset_3d_camera("default"))
        self.root.bind("<Key-R>", lambda _e: self._reset_3d_camera("default"))
        self._refresh_ports()
        if auto_connect:
            self._connect()

        self._tick()

    def _build_ui(self) -> None:
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TFrame", background="#11161f")
        style.configure("Card.TFrame", background="#18202c", relief="flat")
        style.configure("TLabel", background="#11161f", foreground="#e6edf5", font=("Segoe UI", 10))
        style.configure("Card.TLabel", background="#18202c", foreground="#e6edf5", font=("Segoe UI", 10))
        style.configure("Title.TLabel", background="#18202c", foreground="#64b5f6", font=("Segoe UI", 11, "bold"))
        style.configure("Metric.TLabel", background="#18202c", foreground="#90caf9", font=("Consolas", 11, "bold"))

        # Top connection bar
        top = ttk.Frame(self.root, style="Card.TFrame", padding=10)
        top.pack(fill="x", padx=12, pady=(12, 6))

        ttk.Label(top, text="Serial Port:", style="Card.TLabel").pack(side="left", padx=(0, 6))
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=14, state="readonly")
        self.port_combo.pack(side="left", padx=(0, 6))
        ttk.Button(top, text="Refresh", command=self._refresh_ports).pack(side="left", padx=2)
        self.connect_btn = ttk.Button(top, text="Connect", command=self._toggle_connect)
        self.connect_btn.pack(side="left", padx=6)
        ttk.Button(top, text="Ping", command=self.engine.ping).pack(side="left", padx=4)
        ttk.Button(top, text="Tare (Zero)", command=lambda: self.engine.tare(False)).pack(side="left", padx=4)
        self.log_btn = ttk.Button(top, text="Start CSV Log", command=self._toggle_logging)
        self.log_btn.pack(side="left", padx=4)

        self.conn_status_lbl = ttk.Label(top, text="DISCONNECTED", style="Metric.TLabel")
        self.conn_status_lbl.pack(side="right", padx=8)

        # Main grid
        main = ttk.Frame(self.root)
        main.pack(fill="both", expand=True, padx=12, pady=6)
        main.columnconfigure(0, weight=3)
        main.columnconfigure(1, weight=2)
        main.rowconfigure(0, weight=1)

        # Left column: Pressure Plot + Finger 2D Canvas
        left = ttk.Frame(main)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        left.rowconfigure(0, weight=3)
        left.rowconfigure(1, weight=2)
        left.columnconfigure(0, weight=1)

        plot_card = ttk.Frame(left, style="Card.TFrame", padding=10)
        plot_card.grid(row=0, column=0, sticky="nsew", pady=(0, 8))
        ttk.Label(
            plot_card,
            text="Real-Time Pressure Synchronization (Physical S1 vs. Twin Model)",
            style="Title.TLabel",
        ).pack(anchor="w")
        self.plot_canvas = tk.Canvas(plot_card, bg="#0c1017", highlightthickness=0, height=250)
        self.plot_canvas.pack(fill="both", expand=True, pady=(6, 0))

        finger_card = ttk.Frame(left, style="Card.TFrame", padding=10)
        finger_card.grid(row=1, column=0, sticky="nsew")
        ttk.Label(
            finger_card,
            text="Virtual Index Finger Kinematics (Channel 1 Bending Model)",
            style="Title.TLabel",
        ).pack(anchor="w")
        self.finger_canvas = tk.Canvas(finger_card, bg="#0c1017", highlightthickness=0, height=200)
        self.finger_canvas.pack(fill="both", expand=True, pady=(6, 0))

        # Right column: Controls + Metrics + Log
        right = ttk.Frame(main)
        right.grid(row=0, column=1, sticky="nsew")
        right.rowconfigure(2, weight=1)
        right.columnconfigure(0, weight=1)

        ctrl_card = ttk.Frame(right, style="Card.TFrame", padding=10)
        ctrl_card.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        ttk.Label(ctrl_card, text="Pneumatic Actuation & Virtual Control", style="Title.TLabel").pack(
            anchor="w", pady=(0, 6)
        )

        # Valve 1 / 2 quick buttons
        vrow = ttk.Frame(ctrl_card, style="Card.TFrame")
        vrow.pack(fill="x", pady=3)
        ttk.Button(vrow, text="V1 Inflate ON", command=lambda: self.engine.set_valve(1, True)).pack(
            side="left", expand=True, fill="x", padx=2
        )
        ttk.Button(vrow, text="V1 OFF", command=lambda: self.engine.set_valve(1, False)).pack(
            side="left", expand=True, fill="x", padx=2
        )
        ttk.Button(vrow, text="V2 Exhaust ON", command=lambda: self.engine.set_valve(2, True)).pack(
            side="left", expand=True, fill="x", padx=2
        )
        ttk.Button(vrow, text="V2 OFF", command=lambda: self.engine.set_valve(2, False)).pack(
            side="left", expand=True, fill="x", padx=2
        )

        # PWM Sliders
        ttk.Label(ctrl_card, text="Valve 1 PWM (0-999):", style="Card.TLabel").pack(anchor="w", pady=(6, 0))
        s1 = tk.Scale(
            ctrl_card,
            from_=0,
            to=999,
            orient="horizontal",
            variable=self.pwm1_var,
            bg="#18202c",
            fg="#e6edf5",
            highlightthickness=0,
            command=lambda val: self.engine.set_pwm(1, int(float(val))),
        )
        s1.pack(fill="x")

        ttk.Label(ctrl_card, text="Valve 2 PWM (0-999):", style="Card.TLabel").pack(anchor="w", pady=(4, 0))
        s2 = tk.Scale(
            ctrl_card,
            from_=0,
            to=999,
            orient="horizontal",
            variable=self.pwm2_var,
            bg="#18202c",
            fg="#e6edf5",
            highlightthickness=0,
            command=lambda val: self.engine.set_pwm(2, int(float(val))),
        )
        s2.pack(fill="x")

        # Virtual -> Physical closed-loop target curl / pressure
        ttk.Label(ctrl_card, text="Virtual Finger Curl Target (% -> STM32 TARGET):", style="Card.TLabel").pack(
            anchor="w", pady=(8, 0)
        )
        curl_scale = tk.Scale(
            ctrl_card,
            from_=0,
            to=100,
            orient="horizontal",
            variable=self.curl_pct_var,
            bg="#18202c",
            fg="#81c784",
            highlightthickness=0,
        )
        curl_scale.pack(fill="x")

        trow = ttk.Frame(ctrl_card, style="Card.TFrame")
        trow.pack(fill="x", pady=(4, 0))
        ttk.Button(trow, text="Send Finger Curl Target", command=self._apply_curl_target).pack(
            side="left", expand=True, fill="x", padx=2
        )
        ttk.Button(trow, text="Release (Vent)", command=self._release_pressure).pack(
            side="left", expand=True, fill="x", padx=2
        )

        tk.Checkbutton(
            ctrl_card,
            text="Mirror Channel 1 Across All 5 Fingers (3D Hand)",
            variable=self.all_fingers_var,
            bg="#18202c",
            fg="#cfd8dc",
            selectcolor="#0c1017",
            activebackground="#18202c",
            activeforeground="#ffffff",
            highlightthickness=0,
        ).pack(anchor="w", pady=(6, 0))

        cam_row = ttk.Frame(ctrl_card, style="Card.TFrame")
        cam_row.pack(fill="x", pady=(6, 0))
        ttk.Button(
            cam_row,
            text="Reset 3D View (Space)",
            command=lambda: self._reset_3d_camera("default"),
        ).pack(side="left", expand=True, fill="x", padx=2)
        ttk.Button(
            cam_row,
            text="Front",
            command=lambda: self._reset_3d_camera("front"),
        ).pack(side="left", padx=2)
        ttk.Button(
            cam_row,
            text="Side",
            command=lambda: self._reset_3d_camera("side"),
        ).pack(side="left", padx=2)
        ttk.Button(
            cam_row,
            text="Top",
            command=lambda: self._reset_3d_camera("top"),
        ).pack(side="left", padx=2)

        # Metrics Card
        met_card = ttk.Frame(right, style="Card.TFrame", padding=10)
        met_card.grid(row=1, column=0, sticky="ew", pady=(0, 8))
        ttk.Label(met_card, text="Synchronization Evaluation Metrics", style="Title.TLabel").pack(
            anchor="w", pady=(0, 4)
        )
        self.metrics_lbl = ttk.Label(met_card, text="Waiting for telemetry...", style="Metric.TLabel", justify="left")
        self.metrics_lbl.pack(anchor="w")

        # Event Log Card
        log_card = ttk.Frame(right, style="Card.TFrame", padding=10)
        log_card.grid(row=2, column=0, sticky="nsew")
        ttk.Label(log_card, text="STM32 Event Stream", style="Title.TLabel").pack(anchor="w")
        self.event_text = tk.Text(
            log_card,
            height=8,
            bg="#0c1017",
            fg="#cfd8dc",
            font=("Consolas", 9),
            highlightthickness=0,
            state="disabled",
        )
        self.event_text.pack(fill="both", expand=True, pady=(4, 0))

    def _refresh_ports(self) -> None:
        ports = ["SIM"] + list_serial_ports()
        self.port_combo["values"] = ports
        if self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    def _connect(self) -> None:
        port = self.port_var.get() or "SIM"
        try:
            self.engine.connect(port=port)
        except Exception as exc:
            self.connect_btn.configure(text="Connect")
            self.conn_status_lbl.configure(text="PORT ERROR", foreground="#e57373")
            self._append_event(f"ERROR,CONNECT_FAILED,{port},{exc}")
            return
        self.connect_btn.configure(text="Disconnect")
        self.engine.ping()

    def _toggle_connect(self) -> None:
        snap = self.engine.get_snapshot()
        if snap.connected:
            self.engine.disconnect()
            self.connect_btn.configure(text="Connect")
        else:
            self._connect()

    def _toggle_logging(self) -> None:
        snap = self.engine.get_snapshot()
        if snap.logging_active:
            self.engine.stop_logging()
            self.log_btn.configure(text="Start CSV Log")
        else:
            path = self.engine.start_logging()
            self.log_btn.configure(text="Stop CSV Log")
            self._append_event(f"LOG,STARTED,{path}")

    def _apply_curl_target(self) -> None:
        target = self.engine.set_target_curl(self.curl_pct_var.get() / 100.0)
        self.target_kpa_var.set(target)

    def _release_pressure(self) -> None:
        self.curl_pct_var.set(0.0)
        self.engine.set_target_pressure(None)
        self.engine.set_valve(1, False)
        self.engine.set_valve(2, True)

    def _append_event(self, text: str) -> None:
        self.event_text.configure(state="normal")
        self.event_text.insert("end", f"{text}\n")
        self.event_text.see("end")
        self.event_text.configure(state="disabled")

    def _tick(self) -> None:
        snap = self.engine.get_snapshot()
        self._update_status_labels(snap)
        self._draw_plot(snap)
        self._draw_finger(snap)
        self._sync_pybullet(snap)

        events = self.engine.get_events()
        if len(events) > self.last_event_count:
            for ev in events[self.last_event_count :]:
                self._append_event(ev.raw)
            self.last_event_count = len(events)

        self.root.after(30, self._tick)

    def _update_status_labels(self, snap: SyncSnapshot) -> None:
        if snap.connected:
            fw_tag = " [Legacy 0-1->kPa]" if snap.legacy_firmware else ""
            self.conn_status_lbl.configure(text=f"CONNECTED ({snap.port}){fw_tag}", foreground="#81c784")
        else:
            self.conn_status_lbl.configure(text="DISCONNECTED", foreground="#e57373")

        rtt_txt = f"{snap.rtt_ms:.1f} ms" if snap.rtt_ms is not None else "--"
        tgt_txt = f"{snap.target_kpa:.1f} kPa" if snap.target_kpa is not None else "Manual"
        kin = snap.finger_kinematics
        self.metrics_lbl.configure(
            text=(
                f"S1 (Tube)   : {snap.s1_kpa:7.2f} kPa   | S0: {snap.s0_kpa:7.2f} kPa\n"
                f"Twin P_sim  : {snap.sim_kpa:7.2f} kPa   | Target: {tgt_txt}\n"
                f"Valves      : V1={snap.v1} ({snap.pwm1}) | V2={snap.v2} ({snap.pwm2}) | T1={snap.temp1_c:.1f}C\n"
                f"Finger Bend : MCP={kin.mcp_deg:4.1f}° PIP={kin.pip_deg:4.1f}° DIP={kin.dip_deg:4.1f}°\n"
                f"Sync Quality: {snap.sample_rate_hz:5.1f} Hz | Jitter={snap.jitter_ms:4.2f} ms | RTT={rtt_txt}\n"
                f"Twin Error  : RMSE={snap.rmse_kpa:5.2f} kPa | MAE={snap.mae_kpa:5.2f} kPa | r={snap.correlation_r:.3f}"
            )
        )

    def _draw_plot(self, snap: SyncSnapshot) -> None:
        c = self.plot_canvas
        c.delete("all")
        w = max(200, c.winfo_width())
        h = max(120, c.winfo_height())
        pad_l, pad_r, pad_t, pad_b = 46, 16, 16, 24

        times, s1_vals, sim_vals = self.engine.get_history()
        y_max = max(120.0, max(s1_vals, default=0.0) * 1.15, max(sim_vals, default=0.0) * 1.15)

        # Grid
        for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
            y = pad_t + (1.0 - frac) * (h - pad_t - pad_b)
            c.create_line(pad_l, y, w - pad_r, y, fill="#1f2937")
            c.create_text(pad_l - 6, y, text=f"{frac * y_max:.0f}", anchor="e", fill="#78909c", font=("Consolas", 8))

        if len(times) < 2:
            c.create_text(w // 2, h // 2, text="Waiting for 50 Hz STM32 stream...", fill="#78909c")
            return

        t_end = times[-1]
        t_start = max(0.0, t_end - 10.0)
        span_t = max(1.0, t_end - t_start)

        def to_xy(t_val: float, p_val: float) -> tuple[float, float]:
            x = pad_l + ((t_val - t_start) / span_t) * (w - pad_l - pad_r)
            y = pad_t + (1.0 - max(0.0, min(1.0, p_val / y_max))) * (h - pad_t - pad_b)
            return x, y

        pts_s1 = []
        pts_sim = []
        for tv, p_meas, p_sim in zip(times, s1_vals, sim_vals):
            if tv < t_start:
                continue
            pts_s1.extend(to_xy(tv, p_meas))
            pts_sim.extend(to_xy(tv, p_sim))

        if snap.target_kpa is not None:
            _, yt = to_xy(t_end, snap.target_kpa)
            c.create_line(pad_l, yt, w - pad_r, yt, fill="#81c784", dash=(4, 4), width=1.5)

        if len(pts_sim) >= 4:
            c.create_line(*pts_sim, fill="#ffb74d", width=2, smooth=False)
        if len(pts_s1) >= 4:
            c.create_line(*pts_s1, fill="#4fc3f7", width=2, smooth=False)

        c.create_text(pad_l + 8, pad_t + 8, text="— Physical S1 (kPa)", anchor="w", fill="#4fc3f7", font=("Segoe UI", 9, "bold"))
        c.create_text(pad_l + 150, pad_t + 8, text="— Twin P_sim (kPa)", anchor="w", fill="#ffb74d", font=("Segoe UI", 9, "bold"))

    def _draw_finger(self, snap: SyncSnapshot) -> None:
        c = self.finger_canvas
        c.delete("all")
        w = max(200, c.winfo_width())
        h = max(120, c.winfo_height())

        kin = snap.finger_kinematics
        # Base palm segment horizontal on bottom-left, flexing upward-right
        x0, y0 = w * 0.22, h * 0.78
        palm_len = min(w, h) * 0.28
        seg_lens = [palm_len * 0.72, palm_len * 0.52, palm_len * 0.40]
        angles_rad = [kin.mcp_rad, kin.pip_rad, kin.dip_rad]

        # Draw palm
        px, py = x0 + palm_len, y0
        c.create_line(x0, y0, px, py, fill="#37474f", width=14, capstyle="round")
        c.create_text(x0 + palm_len * 0.5, y0 + 18, text="Palm", fill="#90a4ae", font=("Segoe UI", 8))

        # Cumulative bending angle from horizontal right -> curling upward/back
        cur_x, cur_y = px, py
        cum_ang = 0.0
        for idx, (slen, ang) in enumerate(zip(seg_lens, angles_rad)):
            cum_ang += ang
            nx = cur_x + slen * math.cos(cum_ang)
            ny = cur_y - slen * math.sin(cum_ang)
            color = "#29b6f6" if idx < 2 else "#ffa726"
            c.create_line(cur_x, cur_y, nx, ny, fill=color, width=11 - idx * 2, capstyle="round")
            c.create_oval(cur_x - 4, cur_y - 4, cur_x + 4, cur_y + 4, fill="#eceff1", outline="")
            cur_x, cur_y = nx, ny

        c.create_oval(cur_x - 4, cur_y - 4, cur_x + 4, cur_y + 4, fill="#ffa726", outline="")
        c.create_text(
            w - 16,
            20,
            text=f"Total Curl: {kin.total_curl_deg:.1f}° / 230°",
            anchor="e",
            fill="#81c784",
            font=("Consolas", 10, "bold"),
        )

    def _reset_3d_camera(self, preset: str = "default") -> None:
        if self.glove_3d is not None:
            try:
                self.glove_3d.reset_camera(preset)
            except Exception:
                pass

    def _sync_pybullet(self, snap: SyncSnapshot) -> None:
        if self.glove_3d is None:
            return
        idle = FingerKinematics(0.0, 0.0, 0.0)
        mirror_all = self.all_fingers_var.get()
        finger_map = {
            name: (snap.finger_kinematics if (mirror_all or name == "index") else idle)
            for name in FINGER_NAMES
        }
        try:
            self.glove_3d.update_from_pressures(
                finger_map,
                active_kpa=snap.s1_kpa,
                mirror_all=mirror_all,
            )
        except Exception:
            self.glove_3d = None

    def _on_close(self) -> None:
        self.engine.disconnect()
        if self.glove_3d is not None:
            self.glove_3d.close()
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    parser = argparse.ArgumentParser(description="Textile Robotic Glove Digital Twin")
    parser.add_argument("--port", default="SIM", help="Serial COM port (e.g. COM3) or SIM")
    parser.add_argument("--no-pybullet", action="store_true", help="Disable PyBullet 3D window")
    args = parser.parse_args()

    app = DigitalTwinApp(port=args.port, use_pybullet=not args.no_pybullet)
    app.run()


if __name__ == "__main__":
    main()
