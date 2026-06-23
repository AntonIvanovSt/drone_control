import tkinter as tk
import tkinter.messagebox as mb
import serial
import time
import threading


def create_message(l_pwm: int, r_pwm: int) -> str:
    return f"{l_pwm},{r_pwm}\n"


def create_pose_message(x: float, y: float, theta: float) -> str:
    return f"POSE X={x:.2f} Y={y:.2f} Th={theta:.2f}\n"


class DroneControl(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Drone Control")
        self.resizable(False, False)
        self.configure(bg="#0d0f14")

        self.l_pwm = tk.IntVar(value=0)
        self.r_pwm = tk.IntVar(value=0)

        # Pose inputs
        self.pose_x = tk.DoubleVar(value=0.0)
        self.pose_y = tk.DoubleVar(value=0.0)
        self.pose_theta = tk.DoubleVar(value=0.0)

        # Log widget reference — set in _build_ui
        self.log_text: tk.Text | None = None

        self.ser = None
        self._open_serial()
        self._build_ui()

        self._reading = threading.Event()
        self._reading.set()
        self._reader_thread = threading.Thread(target=self._serial_reader, daemon=True)
        self._reader_thread.start()

        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _log(self, message: str):
        """Prepend a timestamped line to the log widget (thread-safe via after())."""
        if self.log_text is None:
            return
        ts = time.strftime("%H:%M:%S")
        self.log_text.configure(state="normal")
        self.log_text.insert("1.0", f"[{ts}]  {message}\n")
        self.log_text.configure(state="disabled")

    def _open_serial(self, port="/dev/ttyACM0", baud=460800):
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            time.sleep(1)
            self.after(0, self._log, f"Connected: {port}")
        except serial.SerialException as e:
            self.ser = None
            self.after(0, self._log, f"Serial error: {e}")

    def _send(self, message: str):
        if self.ser is None or not self.ser.is_open:
            mb.showerror("Serial Error", "Serial port is not open")
            return
        try:
            self.ser.write(message.encode("utf-8"))
            self._log(f"→ {message.strip()}")
        except serial.SerialException as e:
            self._log(f"Send error: {e}")

    def _serial_reader(self):
        while self._reading.is_set():
            if self.ser is None or not self.ser.is_open:
                time.sleep(0.1)
                continue
            try:
                line = self.ser.readline()
                if line:
                    text = line.decode("utf-8", errors="replace").strip()
                    self.after(0, self._log, f"← {text}")
            except serial.SerialException:
                break

    def _build_ui(self):
        self.geometry("1200x640")

        # ── Title ─────────────────────────────────────────────────────────────
        tk.Label(
            self,
            text="Drone Controller",
            font=("Arial", 14),
            fg="#8892a4",
            bg="#0d0f14",
        ).pack(pady=(20, 5))

        # ── PWM sliders ───────────────────────────────────────────────────────
        btn_frame = tk.Frame(self, bg="#0d0f14")
        btn_frame.pack(pady=(10, 0))

        def create_pwm_scale(parent, variable, label):
            scale = tk.Scale(
                parent,
                variable=variable,
                label=label,
                orient="horizontal",
                from_=-100,
                to=100,
                tickinterval=50,
                resolution=10,
                length=400,
                font=("Arial", 10),
                fg="white",
                bg="#0d0f14",
                activebackground="#3a6ad4",
                highlightthickness=0,
                troughcolor="#1c2030",
                sliderrelief="flat",
                cursor="hand2",
            )
            scale.pack(pady=8)
            return scale

        self.left_scale = create_pwm_scale(btn_frame, self.l_pwm, "Left PWM")
        self.right_scale = create_pwm_scale(btn_frame, self.r_pwm, "Right PWM")

        tk.Button(
            btn_frame,
            text="Send PWM",
            command=self._on_click,
            font=("Arial", 12),
            fg="white",
            bg="#5b8af0",
            activebackground="#3a6ad4",
            relief="flat",
            padx=16,
            pady=8,
            cursor="hand2",
        ).pack(pady=8)

        # ── Divider ───────────────────────────────────────────────────────────
        tk.Frame(self, bg="#1c2030", height=1).pack(fill="x", padx=24, pady=(12, 4))

        # ── Pose inputs ───────────────────────────────────────────────────────
        pose_frame = tk.Frame(self, bg="#0d0f14")
        pose_frame.pack(pady=(6, 0))

        tk.Label(
            pose_frame,
            text="Set Pose",
            font=("Arial", 11),
            fg="#8892a4",
            bg="#0d0f14",
        ).grid(row=0, column=0, columnspan=6, pady=(0, 8))

        def create_pose_input(parent, label_text, variable, col_index):
            lbl = tk.Label(
                parent,
                text=label_text,
                bg="#0d0f14",
                fg="white",
                font=("Arial", 10),
            )
            lbl.grid(row=1, column=col_index * 2, padx=(10, 4), sticky="e")

            ent = tk.Entry(
                parent,
                textvariable=variable,
                bg="#1c2030",
                fg="white",
                insertbackground="white",
                relief="flat",
            )
            ent.grid(row=1, column=col_index * 2 + 1, padx=(0, 10))

            return lbl, ent

        poses = [
            ("X (mm)", self.pose_x),
            ("Y (mm)", self.pose_y),
            ("θ (rad)", self.pose_theta),
        ]

        for col, (label, var) in enumerate(poses):
            create_pose_input(pose_frame, label, var, col)

        tk.Button(
            pose_frame,
            text="Send Pose",
            command=self._on_send_pose,
            font=("Arial", 12),
            fg="white",
            bg="#5b8af0",
            activebackground="#3a6ad4",
            relief="flat",
            padx=16,
            pady=8,
            cursor="hand2",
        ).grid(row=2, column=0, columnspan=6, pady=12)

        # ── Divider ───────────────────────────────────────────────────────────
        tk.Frame(self, bg="#1c2030", height=1).pack(fill="x", padx=24, pady=(4, 8))

        # ── Log text area ─────────────────────────────────────────────────────
        log_frame = tk.Frame(self, bg="#0d0f14")
        log_frame.pack(fill="both", expand=True, padx=20, pady=(0, 16))

        tk.Label(
            log_frame,
            text="Log",
            font=("Arial", 10),
            fg="#8892a4",
            bg="#0d0f14",
            anchor="w",
        ).pack(fill="x")

        scrollbar = tk.Scrollbar(log_frame)
        scrollbar.pack(side="right", fill="y")

        self.log_text = tk.Text(
            log_frame,
            font=("Courier", 12),
            fg="#ffffff",
            bg="#000000",
            relief="flat",
            state="disabled",
            yscrollcommand=scrollbar.set,
            wrap="word",
            padx=8,
            pady=6,
            cursor="arrow",
        )
        self.log_text.pack(fill="both", expand=True)
        scrollbar.config(command=self.log_text.yview)

    def _on_click(self):
        l_p = self.l_pwm.get()
        r_p = self.r_pwm.get()
        message = create_message(l_p, r_p)
        self._send(message)

    def _on_send_pose(self):
        try:
            x = float(self.pose_x.get())
            y = float(self.pose_y.get())
            theta = float(self.pose_theta.get())
        except (tk.TclError, ValueError):
            mb.showerror("Input Error", "X, Y and θ must be numbers")
            return
        self._send(create_pose_message(x, y, theta))

    def _on_close(self):
        self._reading.clear()
        self._reader_thread.join(timeout=2)
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(
                    b"0,0\n"
                )  # stop motors directly — log widget may be gone
            except serial.SerialException:
                pass
            self.ser.close()
        self.destroy()


if __name__ == "__main__":
    app = DroneControl()
    app.mainloop()
