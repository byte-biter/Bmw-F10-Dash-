"""
BMW F10 Cluster + Shifter Controller GUI
Sends serial commands to the ESP32-S3 running bmw_f10_merged.ino
Requires: pip install pyserial
"""

import tkinter as tk
from tkinter import ttk
import serial
import serial.tools.list_ports
import threading


class F10Controller:
    def __init__(self, root):
        self.root = root
        self.root.title("BMW F10 Controller")
        self.root.configure(bg="#1a1a2e")
        self.root.resizable(True, True)
        self.root.minsize(950, 600)
        self.ser = None
        self.toggle_states = {}

        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TFrame", background="#1a1a2e")
        style.configure("TLabelframe", background="#1a1a2e", foreground="#e0e0e0")
        style.configure("TLabelframe.Label", background="#1a1a2e", foreground="#e0e0e0",
                         font=("Segoe UI", 9, "bold"))
        style.configure("TLabel", background="#1a1a2e", foreground="#e0e0e0",
                         font=("Segoe UI", 8))
        style.configure("TButton", font=("Segoe UI", 8))
        style.configure("Green.TButton", foreground="#1a1a2e", background="#4ecca3")
        style.configure("Red.TButton", foreground="white", background="#e74c3c")

        # ── Connection bar ────────────────────────────────────────────────
        conn = ttk.Frame(root)
        conn.pack(fill="x", padx=6, pady=(6, 3))

        ttk.Label(conn, text="Port:").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn, textvariable=self.port_var, width=12,
                                        state="readonly")
        self.port_combo.pack(side="left", padx=(4, 2))

        ttk.Button(conn, text="Refresh", command=self.refresh_ports, width=7).pack(side="left", padx=2)
        self.conn_btn = ttk.Button(conn, text="Connect", command=self.toggle_connect,
                                    width=10, style="Green.TButton")
        self.conn_btn.pack(side="left", padx=2)

        self.status_label = ttk.Label(conn, text="Disconnected", foreground="#e74c3c")
        self.status_label.pack(side="left", padx=8)

        ttk.Button(conn, text="STATUS", command=lambda: self.send("STATUS:1"),
                   width=8).pack(side="right", padx=2)

        # ── 3-column layout ─────────────────────────────────────────────
        main = ttk.Frame(root)
        main.pack(fill="both", padx=6, pady=3, expand=True)

        col1 = ttk.Frame(main)
        col1.pack(side="left", fill="both", expand=True)

        col2 = ttk.Frame(main)
        col2.pack(side="left", fill="both", expand=True, padx=(6, 0))

        col3 = ttk.Frame(main)
        col3.pack(side="left", fill="both", expand=True, padx=(6, 0))

        P = 4  # standard padding

        # =====================================================================
        # COLUMN 1 — Gauges
        # =====================================================================

        # ── Ignition + Menu ──────────────────────────────────────────────
        ign_frame = ttk.LabelFrame(col1, text="Ignition", padding=P)
        ign_frame.pack(fill="x", pady=(0, P))

        ign_row = ttk.Frame(ign_frame)
        ign_row.pack(fill="x")

        self.ign_on = True
        self.ign_btn = tk.Button(ign_row, text="IGN ON", width=8, relief="raised",
                                  bg="#4ecca3", fg="#1a1a2e", activebackground="#3d3d54",
                                  font=("Segoe UI", 9, "bold"),
                                  command=self.toggle_ignition)
        self.ign_btn.pack(side="left", padx=2)

        ttk.Button(ign_row, text="Menu", width=5,
                   command=lambda: self.send("MENUBUTTON")).pack(side="left", padx=2)
        ttk.Button(ign_row, text="UDS Boot", width=7,
                   command=lambda: self.send("UDSBOOT")).pack(side="left", padx=2)

        # ── Gear ─────────────────────────────────────────────────────────
        gear_frame = ttk.LabelFrame(col1, text="Gear", padding=P)
        gear_frame.pack(fill="x", pady=(0, P))

        gear_row = ttk.Frame(gear_frame)
        gear_row.pack()
        for g in ["P", "R", "N", "D"]:
            ttk.Button(gear_row, text=g, width=4,
                       command=lambda g=g: self.send(f"GEAR:{g}")).pack(side="left", padx=1)

        gear_row2 = ttk.Frame(gear_frame)
        gear_row2.pack(pady=(2, 0))
        ttk.Label(gear_row2, text="M/S:").pack(side="left")
        for i in range(1, 9):
            ttk.Button(gear_row2, text=str(i), width=3,
                       command=lambda i=i: self.send(f"GEAR:{i}")).pack(side="left", padx=1)

        # ── Speed ────────────────────────────────────────────────────────
        speed_frame = ttk.LabelFrame(col1, text="Speed (km/h)", padding=P)
        speed_frame.pack(fill="x", pady=(0, P))

        self.speed_var = tk.IntVar(value=0)
        self.speed_label = ttk.Label(speed_frame, text="0 km/h",
                                      font=("Segoe UI", 12, "bold"))
        self.speed_label.pack()

        ttk.Scale(speed_frame, from_=0, to=260, variable=self.speed_var,
                  orient="horizontal", command=self.on_speed_change).pack(fill="x")

        spd_btns = ttk.Frame(speed_frame)
        spd_btns.pack(fill="x", pady=(2, 0))
        for spd in [0, 30, 60, 100, 130, 180, 220, 260]:
            ttk.Button(spd_btns, text=str(spd), width=4,
                       command=lambda s=spd: self.set_speed(s)).pack(side="left", padx=1)

        # ── RPM ──────────────────────────────────────────────────────────
        rpm_frame = ttk.LabelFrame(col1, text="RPM", padding=P)
        rpm_frame.pack(fill="x", pady=(0, P))

        self.rpm_var = tk.IntVar(value=1000)
        self.rpm_label = ttk.Label(rpm_frame, text="1000 RPM",
                                    font=("Segoe UI", 12, "bold"))
        self.rpm_label.pack()

        ttk.Scale(rpm_frame, from_=0, to=6900, variable=self.rpm_var,
                  orient="horizontal", command=self.on_rpm_change).pack(fill="x")

        rpm_btns = ttk.Frame(rpm_frame)
        rpm_btns.pack(fill="x", pady=(2, 0))
        for r in [0, 800, 1000, 2000, 3000, 4500, 6000, 6900]:
            ttk.Button(rpm_btns, text=str(r), width=4,
                       command=lambda r=r: self.set_rpm(r)).pack(side="left", padx=1)

        # ── Temp / Fuel / Backlight (compact) ────────────────────────────
        gauge_frame = ttk.LabelFrame(col1, text="Temp / Fuel / Backlight", padding=P)
        gauge_frame.pack(fill="x", pady=(0, P))

        for label, var_init, from_, to_, cmd, fmt in [
            ("Temp", 100, 0, 200, "TEMP", "{v} C"),
            ("Fuel", 100, 0, 100, "FUEL", "{v}%"),
            ("Light", 100, 0, 100, "BRIGHT", "{v}%"),
        ]:
            row = ttk.Frame(gauge_frame)
            row.pack(fill="x", pady=1)
            lbl = ttk.Label(row, text=label, width=5)
            lbl.pack(side="left")
            var = tk.IntVar(value=var_init)
            val_lbl = ttk.Label(row, text=str(var_init), width=4)
            val_lbl.pack(side="left")
            scale = ttk.Scale(row, from_=from_, to=to_, variable=var,
                              orient="horizontal",
                              command=lambda v, c=cmd, vl=val_lbl: self._gauge_change(v, c, vl))
            scale.pack(side="left", fill="x", expand=True, padx=(4, 0))

        # =====================================================================
        # COLUMN 2 — Lights, Warnings, Drive Mode
        # =====================================================================

        # ── Drive Mode ───────────────────────────────────────────────────
        mode_frame = ttk.LabelFrame(col2, text="Drive Mode", padding=P)
        mode_frame.pack(fill="x", pady=(0, P))

        mode_row = ttk.Frame(mode_frame)
        mode_row.pack()
        for name, val in [("Comfort", 2), ("Sport", 4), ("Sport+", 5),
                          ("Eco", 7), ("Traction", 1), ("DSC Off", 6)]:
            ttk.Button(mode_row, text=name, width=7,
                       command=lambda v=val: self.send(f"MODE:{v}")).pack(side="left", padx=1, pady=1)

        # ── Lights ───────────────────────────────────────────────────────
        lights_frame = ttk.LabelFrame(col2, text="Lights", padding=P)
        lights_frame.pack(fill="x", pady=(0, P))

        lights_grid = ttk.Frame(lights_frame)
        lights_grid.pack()
        for i, (label, cmd) in enumerate([
            ("High Beam", "HIGHBEAM"), ("Front Fog", "FRONTFOG"),
            ("Rear Fog", "REARFOG"), ("L Blink", "LEFTBLINK"),
            ("R Blink", "RIGHTBLINK"),
        ]):
            self.make_toggle(lights_grid, label, cmd, row=i // 3, col=i % 3)

        # ── Warnings ────────────────────────────────────────────────────
        warn_frame = ttk.LabelFrame(col2, text="Warnings", padding=P)
        warn_frame.pack(fill="x", pady=(0, P))

        warn_grid = ttk.Frame(warn_frame)
        warn_grid.pack()
        warnings = [
            ("Door", "DOOR"), ("DSC", "DSC"), ("Handbrake", "HANDBRAKE"),
            ("Check Eng", "CHECKENG"), ("DSC Off", "DSCOFF"), ("Park Brake", "PARKBRAKE"),
            ("SOS", "SOSCALL"), ("Chassis", "CHASSIS"), ("Cruise", "CRUISE"),
            ("Brake Fail", "BRAKE"), ("Dipped", "DIPPED"), ("Trailer", "TRAILER"),
            ("Restraint", "RESTRAINT"), ("Seatbelt", "FASTEN"),
        ]
        for i, (label, cmd) in enumerate(warnings):
            self.make_toggle(warn_grid, label, cmd, row=i // 4, col=i % 4)

        # ── Cruise Control ──────────────────────────────────────────────
        cc_frame = ttk.LabelFrame(col2, text="Cruise Control", padding=P)
        cc_frame.pack(fill="x", pady=(0, P))

        cc_row = ttk.Frame(cc_frame)
        cc_row.pack()
        for name, cmd in [("SET", "CCSET"), ("RESUME", "CCRESUME"),
                          ("CANCEL", "CCCANCEL"), ("OFF", "CCOFF"),
                          ("+", "CCPLUS"), ("-", "CCMINUS")]:
            ttk.Button(cc_row, text=name, width=7,
                       command=lambda c=cmd: self.send(c)).pack(side="left", padx=1, pady=1)

        # =====================================================================
        # COLUMN 3 — Alerts, Clock, Lane, UDS
        # =====================================================================

        # ── CC-ID Alert + Overspeed ─────────────────────────────────────
        alert_frame = ttk.LabelFrame(col3, text="Alerts / Overspeed", padding=P)
        alert_frame.pack(fill="x", pady=(0, P))

        alert_row = ttk.Frame(alert_frame)
        alert_row.pack(fill="x")
        ttk.Label(alert_row, text="CC-ID:").pack(side="left")
        self.alert_entry = ttk.Entry(alert_row, width=5, font=("Consolas", 9))
        self.alert_entry.pack(side="left", padx=(2, 2))
        self.alert_entry.insert(0, "34")
        ttk.Button(alert_row, text="SET", width=5,
                   command=self.alert_set).pack(side="left", padx=1)
        ttk.Button(alert_row, text="CLR", width=5,
                   command=self.alert_clear).pack(side="left", padx=1)

        ospd_row = ttk.Frame(alert_frame)
        ospd_row.pack(fill="x", pady=(3, 0))
        ttk.Label(ospd_row, text="Overspeed:").pack(side="left")
        self.ospd_var = tk.IntVar(value=120)
        tk.Spinbox(ospd_row, from_=0, to=260, textvariable=self.ospd_var,
                   width=4, font=("Consolas", 9)).pack(side="left", padx=(2, 2))
        ttk.Button(ospd_row, text="Set", width=4,
                   command=self.set_overspeed).pack(side="left", padx=1)
        ttk.Label(ospd_row, text="km/h (0=off)", foreground="#888").pack(side="left", padx=2)

        # ── Clock & Date ────────────────────────────────────────────────
        clock_frame = ttk.LabelFrame(col3, text="Clock & Date (0x2F8)", padding=P)
        clock_frame.pack(fill="x", pady=(0, P))

        time_row = ttk.Frame(clock_frame)
        time_row.pack(fill="x")
        ttk.Label(time_row, text="Time:").pack(side="left")
        self.clock_entry = ttk.Entry(time_row, width=6, font=("Consolas", 9))
        self.clock_entry.pack(side="left", padx=(2, 2))
        self.clock_entry.insert(0, "12:00")
        ttk.Button(time_row, text="Set", width=4,
                   command=self.set_time).pack(side="left", padx=1)

        date_row = ttk.Frame(clock_frame)
        date_row.pack(fill="x", pady=(3, 0))
        ttk.Label(date_row, text="Date:").pack(side="left")
        self.date_entry = ttk.Entry(date_row, width=10, font=("Consolas", 9))
        self.date_entry.pack(side="left", padx=(2, 2))
        self.date_entry.insert(0, "26.02.2026")
        ttk.Button(date_row, text="Set", width=4,
                   command=self.set_date).pack(side="left", padx=1)

        # ── Lane Assistant ──────────────────────────────────────────────
        lane_frame = ttk.LabelFrame(col3, text="Lane Assistant (0x327)", padding=P)
        lane_frame.pack(fill="x", pady=(0, P))

        lane_row = ttk.Frame(lane_frame)
        lane_row.pack(fill="x")

        self.lane_on = False
        self.lane_btn = tk.Button(lane_row, text="LANE OFF", width=9,
                                  bg="#e74c3c", fg="white", font=("Segoe UI", 9, "bold"),
                                  activebackground="#c0392b",
                                  command=self.toggle_lane)
        self.lane_btn.pack(side="left", padx=2)

        ttk.Label(lane_row, text="L:").pack(side="left")
        self.lane_left_var = tk.IntVar(value=3)
        tk.Spinbox(lane_row, from_=0, to=255, textvariable=self.lane_left_var,
                   width=3, font=("Consolas", 9),
                   command=self.update_lane_values).pack(side="left", padx=(1, 3))

        ttk.Label(lane_row, text="R:").pack(side="left")
        self.lane_right_var = tk.IntVar(value=3)
        tk.Spinbox(lane_row, from_=0, to=255, textvariable=self.lane_right_var,
                   width=3, font=("Consolas", 9),
                   command=self.update_lane_values).pack(side="left", padx=1)

        lane_scan_row = ttk.Frame(lane_frame)
        lane_scan_row.pack(fill="x", pady=(3, 0))

        ttk.Button(lane_scan_row, text="Full Scan", width=7,
                   command=lambda: self.send("LANESCAN")).pack(side="left", padx=1)

        ttk.Label(lane_scan_row, text="ID:").pack(side="left", padx=(3, 0))
        self.lane_id_entry = ttk.Entry(lane_scan_row, width=5, font=("Consolas", 9))
        self.lane_id_entry.pack(side="left", padx=(1, 1))
        self.lane_id_entry.insert(0, "0x327")
        ttk.Button(lane_scan_row, text="Scan ID", width=6,
                   command=self.scan_lane_id).pack(side="left", padx=1)

        lane_xor_row = ttk.Frame(lane_frame)
        lane_xor_row.pack(fill="x", pady=(2, 0))

        ttk.Label(lane_xor_row, text="XOR:").pack(side="left")
        self.lane_xor_entry = ttk.Entry(lane_xor_row, width=4, font=("Consolas", 9))
        self.lane_xor_entry.pack(side="left", padx=(1, 1))
        self.lane_xor_entry.insert(0, "0x27")
        ttk.Button(lane_xor_row, text="Test", width=5,
                   command=self.test_lane_xor).pack(side="left", padx=1)

        ttk.Label(lane_frame, text="Full=9 CAN IDs x 256 XOR | Scan ID=one ID x 256 XOR",
                  foreground="#888", font=("Segoe UI", 7)).pack(anchor="w")

        # ── UDS Diagnostics ─────────────────────────────────────────────
        uds_frame = ttk.LabelFrame(col3, text="UDS Diagnostics (KOMBI 0x86)", padding=P)
        uds_frame.pack(fill="x", pady=(0, P))

        # Quick buttons row 1
        uds_btns = ttk.Frame(uds_frame)
        uds_btns.pack(fill="x")
        ttk.Button(uds_btns, text="Ext Sess", width=7,
                   command=lambda: self.send("UDSRAW:10 03")).pack(side="left", padx=1)
        ttk.Button(uds_btns, text="Read ID", width=7,
                   command=lambda: self.send("UDSRAW:22 F1 90")).pack(side="left", padx=1)
        ttk.Button(uds_btns, text="Coding", width=6,
                   command=lambda: self.send("UDSRAW:22 F1 00")).pack(side="left", padx=1)
        ttk.Button(uds_btns, text="CAFD", width=5,
                   command=lambda: self.send("UDSRAW:22 17 04")).pack(side="left", padx=1)

        # Quick buttons row 2
        uds_btns2 = ttk.Frame(uds_frame)
        uds_btns2.pack(fill="x", pady=(2, 0))
        ttk.Button(uds_btns2, text="Read VIN", width=7,
                   command=lambda: self.send("UDSVIN")).pack(side="left", padx=1)
        ttk.Button(uds_btns2, text="Full Info", width=7,
                   command=lambda: self.send("UDSINFO")).pack(side="left", padx=1)
        ttk.Button(uds_btns2, text="Boot Prog", width=7,
                   command=lambda: self.send("UDSBOOTPROG")).pack(side="left", padx=1)
        ttk.Button(uds_btns2, text="Test Write", width=8,
                   command=lambda: self.send("UDSTESTWRITE")).pack(side="left", padx=1)

        # Quick buttons row 3
        uds_btns3 = ttk.Frame(uds_frame)
        uds_btns3.pack(fill="x", pady=(2, 0))
        ttk.Button(uds_btns3, text="UDS Scan", width=7,
                   command=lambda: self.send("UDSSCAN")).pack(side="left", padx=1)
        ttk.Button(uds_btns3, text="Sec Seed", width=7,
                   command=lambda: self.send("UDSSEED")).pack(side="left", padx=1)
        self.brute_btn = tk.Button(uds_btns3, text="BRUTE FORCE", width=11,
                                    relief="raised", bg="#e74c3c", fg="white",
                                    activebackground="#c0392b", activeforeground="white",
                                    font=("Segoe UI", 8, "bold"),
                                    command=lambda: self.send("UDSBRUTE"))
        self.brute_btn.pack(side="left", padx=1)

        uds_raw_row = ttk.Frame(uds_frame)
        uds_raw_row.pack(fill="x", pady=(3, 0))
        ttk.Label(uds_raw_row, text="Raw hex:").pack(side="left")
        self.uds_entry = ttk.Entry(uds_raw_row, width=20, font=("Consolas", 9))
        self.uds_entry.pack(side="left", padx=(2, 2), fill="x", expand=True)
        self.uds_entry.insert(0, "10 03")
        ttk.Button(uds_raw_row, text="Send", width=5,
                   command=self.send_uds).pack(side="left", padx=1)

        ttk.Label(uds_frame,
                  text="Responses show in log as UDS_RX",
                  foreground="#888", font=("Segoe UI", 7)).pack(anchor="w")

        # ── Serial Log (column 3, fills remaining space) ────────────────
        log_frame = ttk.LabelFrame(col3, text="Serial Log", padding=P)
        log_frame.pack(fill="both", pady=(0, P), expand=True)

        self.log_text = tk.Text(log_frame, height=6, bg="#0f0f23", fg="#4ecca3",
                                 font=("Consolas", 8), insertbackground="#4ecca3",
                                 state="disabled")
        self.log_text.pack(fill="both", expand=True)

        # ── Manual command (bottom, full width) ─────────────────────────
        cmd_row = ttk.Frame(root)
        cmd_row.pack(fill="x", padx=6, pady=(0, 6))

        self.cmd_entry = ttk.Entry(cmd_row, font=("Consolas", 9))
        self.cmd_entry.pack(side="left", fill="x", expand=True)
        self.cmd_entry.bind("<Return>", lambda e: self.send_manual())

        ttk.Button(cmd_row, text="Send", command=self.send_manual, width=8).pack(side="left", padx=(4, 0))

        self.refresh_ports()

    # ── Toggle helpers ───────────────────────────────────────────────────
    def make_toggle(self, parent, label, cmd, row, col):
        self.toggle_states[cmd] = False
        btn = tk.Button(parent, text=label, width=9, relief="raised",
                        bg="#2d2d44", fg="#e0e0e0", activebackground="#3d3d54",
                        font=("Segoe UI", 8),
                        command=lambda: self.do_toggle(cmd, btn))
        btn.grid(row=row, column=col, padx=1, pady=1)

    def do_toggle(self, cmd, btn):
        self.toggle_states[cmd] = not self.toggle_states[cmd]
        on = self.toggle_states[cmd]
        btn.configure(bg="#4ecca3" if on else "#2d2d44",
                      fg="#1a1a2e" if on else "#e0e0e0")
        self.send(f"{cmd}:{'ON' if on else '0'}")

    def toggle_ignition(self):
        self.ign_on = not self.ign_on
        self.ign_btn.configure(
            text="IGN ON" if self.ign_on else "IGN OFF",
            bg="#4ecca3" if self.ign_on else "#e74c3c",
            fg="#1a1a2e" if self.ign_on else "white")
        self.send(f"IGN:{'ON' if self.ign_on else '0'}")

    # ── Alert / Overspeed ────────────────────────────────────────────────
    def alert_set(self):
        ccid = self.alert_entry.get().strip()
        if ccid.isdigit():
            self.send(f"ALERT:{ccid}")

    def alert_clear(self):
        ccid = self.alert_entry.get().strip()
        if ccid.isdigit():
            self.send(f"CLEARALERT:{ccid}")

    def set_overspeed(self):
        self.send(f"OVERSPEED:{self.ospd_var.get()}")

    # ── Clock / Date ─────────────────────────────────────────────────────
    def set_time(self):
        t = self.clock_entry.get().strip()
        if ":" in t:
            self.send(f"TIME:{t}")

    def set_date(self):
        d = self.date_entry.get().strip()
        if "." in d:
            self.send(f"DATE:{d}")

    # ── Lane Assistant ───────────────────────────────────────────────────
    def toggle_lane(self):
        self.lane_on = not self.lane_on
        self.lane_btn.configure(
            text="LANE ON" if self.lane_on else "LANE OFF",
            bg="#4ecca3" if self.lane_on else "#e74c3c",
            fg="#1a1a2e" if self.lane_on else "white")
        self.send(f"LANE:{'ON' if self.lane_on else '0'}")

    def update_lane_values(self):
        self.send(f"LANELEFT:{self.lane_left_var.get()}")
        self.send(f"LANERIGHT:{self.lane_right_var.get()}")

    def scan_lane_id(self):
        can_id = self.lane_id_entry.get().strip()
        self.send(f"LANESCAN:{can_id}")

    def test_lane_xor(self):
        xor_val = self.lane_xor_entry.get().strip()
        can_id = self.lane_id_entry.get().strip()
        self.send(f"LANEXOR:{xor_val}:{can_id}")

    # ── UDS ──────────────────────────────────────────────────────────────
    def send_uds(self):
        raw = self.uds_entry.get().strip()
        if raw:
            self.send(f"UDSRAW:{raw}")

    # ── Gauge compact helper ─────────────────────────────────────────────
    def _gauge_change(self, val, cmd, val_lbl):
        v = int(float(val))
        val_lbl.configure(text=str(v))
        self.send(f"{cmd}:{v}")

    # ── Serial ───────────────────────────────────────────────────────────
    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports:
            self.port_combo.current(0)

    def toggle_connect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.ser = None
            self.conn_btn.configure(text="Connect", style="Green.TButton")
            self.status_label.configure(text="Disconnected", foreground="#e74c3c")
            return

        port = self.port_var.get()
        if not port:
            return

        try:
            self.ser = serial.Serial(port, 115200, timeout=0.1)
            self.conn_btn.configure(text="Disconnect", style="Red.TButton")
            self.status_label.configure(text=f"Connected: {port}", foreground="#4ecca3")
            self.read_thread = threading.Thread(target=self.read_serial, daemon=True)
            self.read_thread.start()
        except serial.SerialException as e:
            self.log(f"ERROR: {e}")

    def send(self, cmd):
        if not self.ser or not self.ser.is_open:
            self.log(f"[NOT CONNECTED] {cmd}")
            return
        try:
            self.ser.write(f"{cmd}\n".encode())
            self.log(f"> {cmd}")
        except serial.SerialException as e:
            self.log(f"SEND ERROR: {e}")

    def send_manual(self):
        cmd = self.cmd_entry.get().strip()
        if cmd:
            self.send(cmd)
            self.cmd_entry.delete(0, "end")

    def read_serial(self):
        while self.ser and self.ser.is_open:
            try:
                line = self.ser.readline().decode("utf-8", errors="replace").strip()
                if line:
                    self.root.after(0, self.log, f"< {line}")
            except Exception:
                break

    def log(self, msg):
        self.log_text.configure(state="normal")
        self.log_text.insert("end", msg + "\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    # ── Speed / RPM presets ──────────────────────────────────────────────
    def set_speed(self, val):
        self.speed_var.set(val)
        self.speed_label.configure(text=f"{val} km/h")
        self.send(f"SPEED:{val}")

    def set_rpm(self, val):
        self.rpm_var.set(val)
        self.rpm_label.configure(text=f"{val} RPM")
        self.send(f"RPM:{val}")

    def on_speed_change(self, val):
        v = int(float(val))
        self.speed_label.configure(text=f"{v} km/h")
        self.send(f"SPEED:{v}")

    def on_rpm_change(self, val):
        v = int(float(val))
        self.rpm_label.configure(text=f"{v} RPM")
        self.send(f"RPM:{v}")


if __name__ == "__main__":
    root = tk.Tk()
    app = F10Controller(root)
    root.mainloop()
