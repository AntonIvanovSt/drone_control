import tkinter as tk
import tkinter.messagebox as mb
import serial
import time
import threading
import re

import matplotlib

matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


def create_message(l_pwm: int, r_pwm: int) -> str:
    return f"{l_pwm},{r_pwm}\n"


def create_spd_message(l_spd: int, r_spd: int) -> str:
    return f"SPD L={l_spd} R={r_spd}\n"


def create_pose_message(x: float, y: float, theta: float) -> str:
    return f"POSE X={x:.2f} Y={y:.2f} Th={theta:.2f}\n"


def create_pid_message(kp: float, ki: float, kd: float, kff: float) -> str:
    return f"SET_COEFF {kp:.4f} {ki:.4f} {kd:.4f} {kff:.4f}\n"


# ── Style constants ────────────────────────────────────────────────────────────
BG = "#0d0f14"
BG2 = "#1c2030"
FG = "#ffffff"
FG_DIM = "#8892a4"
ACCENT = "#5b8af0"
ACCENT_H = "#3a6ad4"
FONT = ("Arial", 10)
FONT_LG = ("Arial", 12)
FONT_HDR = ("Arial", 11)
FONT_TTL = ("Arial", 14)
FONT_LOG = ("Courier", 10)

# Telemetry line pattern:  TGT L=500 R=500  ACT L=487 R=493 mm/s
_TELEM_RE = re.compile(
    r"TGT L=([\d\.\-]+) R=([\d\.\-]+)\s+ACT L=([\d\.\-]+) R=([\d\.\-]+) mm/s"
)

# Rolling window shown on chart (seconds)
CHART_WINDOW_S = 20

BTN_KW = dict(
    font=FONT_LG,
    fg=FG,
    bg=ACCENT,
    activebackground=ACCENT_H,
    relief="flat",
    padx=16,
    pady=6,
    cursor="hand2",
)
LBL_KW = dict(font=FONT, fg=FG, bg=BG)
HDR_KW = dict(font=FONT_HDR, fg=FG_DIM, bg=BG)
ENT_KW = dict(
    font=FONT,
    fg=FG,
    bg=BG2,
    insertbackground=FG,
    relief="flat",
    width=10,
)


class DroneControl(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Drone Control")
        self.resizable(False, False)
        self.configure(bg=BG)

        # ── Variables ──────────────────────────────────────────────────────────
        self.l_pwm = tk.StringVar(value="0")
        self.r_pwm = tk.StringVar(value="0")
        self.l_spd = tk.StringVar(value="0")
        self.r_spd = tk.StringVar(value="0")
        self.pose_x = tk.StringVar(value="0.0")
        self.pose_y = tk.StringVar(value="0.0")
        self.pose_theta = tk.StringVar(value="0.0")
        self.kp = tk.StringVar(value="0.1")
        self.ki = tk.StringVar(value="0.0")
        self.kd = tk.StringVar(value="0.0")
        self.kff = tk.StringVar(value="0.0")

        self.log_text: tk.Text | None = None

        # ── Chart data ─────────────────────────────────────────────────────────
        self._chart_lock = threading.Lock()
        self._t_data: list[float] = []
        self._tgt_l_data: list[float] = []
        self._tgt_r_data: list[float] = []
        self._act_l_data: list[float] = []
        self._act_r_data: list[float] = []
        self._chart_start = time.time()

        self.ser = None
        self._open_serial()
        self._build_ui()
        self._start_chart_refresh()

        self._reading = threading.Event()
        self._reading.set()
        self._reader_thread = threading.Thread(target=self._serial_reader, daemon=True)
        self._reader_thread.start()

        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ── Serial helpers ─────────────────────────────────────────────────────────

    def _log(self, message: str):
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
                    self._parse_telemetry(text)
            except serial.SerialException:
                break

    def _parse_telemetry(self, line: str):
        """Extract TGT/ACT speeds from a telemetry line and append to chart data."""
        m = _TELEM_RE.search(line)
        if not m:
            return
        try:
            tgt_l = float(m.group(1))
            tgt_r = float(m.group(2))
            act_l = float(m.group(3))
            act_r = float(m.group(4))
        except ValueError:
            return
        t = time.time() - self._chart_start
        with self._chart_lock:
            self._t_data.append(t)
            self._tgt_l_data.append(tgt_l)
            self._tgt_r_data.append(tgt_r)
            self._act_l_data.append(act_l)
            self._act_r_data.append(act_r)

    # ── UI builder ─────────────────────────────────────────────────────────────

    def _build_ui(self):
        self.geometry("1300x780")

        # Root: col 0 = left panel (fixed), col 1 = right panel (expands)
        self.grid_columnconfigure(0, weight=0)
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)

        left = tk.Frame(self, bg=BG)
        right = tk.Frame(self, bg=BG)
        left.grid(row=0, column=0, sticky="nsew", padx=(16, 8), pady=16)
        right.grid(row=0, column=1, sticky="nsew", padx=(8, 16), pady=16)

        # Right panel: row 0 = chart, row 1 = log (expands)
        right.grid_columnconfigure(0, weight=1)
        right.grid_rowconfigure(0, weight=3)  # chart gets 3 parts
        right.grid_rowconfigure(2, weight=1)  # log gets 1 part (at least 5 rows)

        self._build_left(left)
        self._build_right(right)

    # ── Left panel ─────────────────────────────────────────────────────────────

    def _build_left(self, parent: tk.Frame):
        r = 0

        tk.Label(parent, text="Drone Controller", font=FONT_TTL, fg=FG_DIM, bg=BG).grid(
            row=r, column=0, columnspan=2, sticky="w", pady=(0, 16)
        )
        r += 1

        # ── PWM ───────────────────────────────────────────────────────────────
        tk.Label(parent, text="PWM", **HDR_KW).grid(
            row=r, column=0, columnspan=2, sticky="w", pady=(0, 6)
        )
        r += 1
        r = self._add_int_row(parent, r, "Left PWM", self.l_pwm, -100, 100)
        r = self._add_int_row(parent, r, "Right PWM", self.r_pwm, -100, 100)
        tk.Button(parent, text="Send PWM", command=self._on_send_pwm, **BTN_KW).grid(
            row=r, column=0, columnspan=2, sticky="ew", pady=(8, 16)
        )
        r += 1

        tk.Frame(parent, bg=BG2, height=1).grid(
            row=r, column=0, columnspan=2, sticky="ew", pady=(0, 16)
        )
        r += 1

        # ── Speed ─────────────────────────────────────────────────────────────
        tk.Label(parent, text="Speed", **HDR_KW).grid(
            row=r, column=0, columnspan=2, sticky="w", pady=(0, 6)
        )
        r += 1
        r = self._add_int_row(parent, r, "Left SPD", self.l_spd, -1000, 1000)
        r = self._add_int_row(parent, r, "Right SPD", self.r_spd, -1000, 1000)
        tk.Button(parent, text="Send SPD", command=self._on_send_spd, **BTN_KW).grid(
            row=r, column=0, columnspan=2, sticky="ew", pady=(8, 16)
        )
        r += 1

        tk.Frame(parent, bg=BG2, height=1).grid(
            row=r, column=0, columnspan=2, sticky="ew", pady=(0, 16)
        )
        r += 1

        # ── Pose ──────────────────────────────────────────────────────────────
        tk.Label(parent, text="Pose", **HDR_KW).grid(
            row=r, column=0, columnspan=2, sticky="w", pady=(0, 6)
        )
        r += 1
        r = self._add_float_row(parent, r, "X (mm)", self.pose_x)
        r = self._add_float_row(parent, r, "Y (mm)", self.pose_y)
        r = self._add_float_row(parent, r, "θ (rad)", self.pose_theta)
        tk.Button(parent, text="Send Pose", command=self._on_send_pose, **BTN_KW).grid(
            row=r, column=0, columnspan=2, sticky="ew", pady=(8, 16)
        )
        r += 1

        tk.Frame(parent, bg=BG2, height=1).grid(
            row=r, column=0, columnspan=2, sticky="ew", pady=(0, 16)
        )
        r += 1

        # ── PID ───────────────────────────────────────────────────────────────
        tk.Label(parent, text="PID", **HDR_KW).grid(
            row=r, column=0, columnspan=2, sticky="w", pady=(0, 6)
        )
        r += 1
        r = self._add_float_row(parent, r, "Kp", self.kp)
        r = self._add_float_row(parent, r, "Ki", self.ki)
        r = self._add_float_row(parent, r, "Kd", self.kd)
        r = self._add_float_row(parent, r, "Kff", self.kff)
        tk.Button(parent, text="Send PID", command=self._on_send_pid, **BTN_KW).grid(
            row=r, column=0, columnspan=2, sticky="ew", pady=(8, 0)
        )

    def _add_int_row(
        self,
        parent: tk.Frame,
        row: int,
        label: str,
        var: tk.StringVar,
        lo: int,
        hi: int,
    ) -> int:
        tk.Label(parent, text=f"{label}  [{lo}…{hi}]", **LBL_KW).grid(
            row=row, column=0, sticky="e", padx=(0, 8), pady=3
        )
        ent = tk.Entry(parent, textvariable=var, **ENT_KW)
        ent.grid(row=row, column=1, sticky="w", pady=3)
        ent.bind("<FocusOut>", lambda e, v=var, l=lo, h=hi: self._clamp_int(v, l, h))
        ent.bind("<Return>", lambda e, v=var, l=lo, h=hi: self._clamp_int(v, l, h))
        return row + 1

    def _add_float_row(
        self, parent: tk.Frame, row: int, label: str, var: tk.StringVar
    ) -> int:
        tk.Label(parent, text=label, **LBL_KW).grid(
            row=row, column=0, sticky="e", padx=(0, 8), pady=3
        )
        ent = tk.Entry(parent, textvariable=var, **ENT_KW)
        ent.grid(row=row, column=1, sticky="w", pady=3)
        ent.bind("<FocusOut>", lambda e, v=var: self._validate_float(v))
        ent.bind("<Return>", lambda e, v=var: self._validate_float(v))
        return row + 1

    # ── Right panel: chart + log ───────────────────────────────────────────────

    def _build_right(self, parent: tk.Frame):
        # ── Chart ─────────────────────────────────────────────────────────────
        self._fig = Figure(facecolor=BG)
        self._ax = self._fig.add_subplot(111)
        self._ax.set_facecolor(BG2)
        self._ax.set_title("Wheel Speed Response", color=FG_DIM, fontsize=10)
        self._ax.set_xlabel("Time (s)", color=FG_DIM)
        self._ax.set_ylabel("Speed (mm/s)", color=FG_DIM)
        self._ax.tick_params(colors=FG_DIM)
        for spine in self._ax.spines.values():
            spine.set_edgecolor(BG2)
        self._fig.tight_layout(pad=1.5)

        self._canvas = FigureCanvasTkAgg(self._fig, master=parent)
        self._canvas.get_tk_widget().configure(bg=BG, highlightthickness=0)
        self._canvas.get_tk_widget().grid(
            row=0, column=0, columnspan=2, sticky="nsew", pady=(0, 8)
        )

        # ── Divider ───────────────────────────────────────────────────────────
        tk.Frame(parent, bg=BG2, height=1).grid(
            row=1, column=0, columnspan=2, sticky="ew", pady=(0, 8)
        )

        # ── Log ───────────────────────────────────────────────────────────────
        tk.Label(parent, text="Log", **HDR_KW).grid(
            row=2, column=0, sticky="w", pady=(0, 4)
        )

        scrollbar = tk.Scrollbar(parent)
        scrollbar.grid(row=3, column=1, sticky="ns")

        self.log_text = tk.Text(
            parent,
            font=FONT_LOG,
            fg=FG,
            bg="#000000",
            relief="flat",
            state="disabled",
            yscrollcommand=scrollbar.set,
            wrap="word",
            padx=8,
            pady=6,
            cursor="arrow",
            height=5,  # minimum 5 rows; expands with window
        )
        self.log_text.grid(row=3, column=0, sticky="nsew")
        scrollbar.config(command=self.log_text.yview)

        parent.grid_rowconfigure(3, weight=1)

    # ── Chart refresh (runs on main thread via after()) ────────────────────────

    def _start_chart_refresh(self):
        self._refresh_chart()

    def _refresh_chart(self):
        with self._chart_lock:
            t = list(self._t_data)
            tl = list(self._tgt_l_data)
            tr = list(self._tgt_r_data)
            al = list(self._act_l_data)
            ar = list(self._act_r_data)

        ax = self._ax
        ax.clear()
        ax.set_facecolor(BG2)
        ax.set_title("Wheel Speed Response", color=FG_DIM, fontsize=10)
        ax.set_xlabel("Time (s)", color=FG_DIM)
        ax.set_ylabel("Speed (mm/s)", color=FG_DIM)
        ax.tick_params(colors=FG_DIM)
        for spine in ax.spines.values():
            spine.set_edgecolor(BG2)

        if t:
            ax.plot(t, al, color="#5b8af0", linewidth=1.5, label="Act L")
            ax.plot(
                t,
                tl,
                color="#5b8af0",
                linewidth=1,
                linestyle="--",
                alpha=0.6,
                label="Tgt L",
            )
            ax.plot(t, ar, color="#f0875b", linewidth=1.5, label="Act R")
            ax.plot(
                t,
                tr,
                color="#f0875b",
                linewidth=1,
                linestyle="--",
                alpha=0.6,
                label="Tgt R",
            )
            ax.legend(
                facecolor=BG2, labelcolor=FG, fontsize=8, framealpha=0.8, edgecolor=BG2
            )

            now = t[-1]
            x_min = max(0.0, now - CHART_WINDOW_S)
            ax.set_xlim(x_min, max(now, x_min + CHART_WINDOW_S))

        self._fig.tight_layout(pad=1.5)
        self._canvas.draw_idle()
        self.after(1000, self._refresh_chart)

    # ── Clamp / validation ─────────────────────────────────────────────────────

    @staticmethod
    def _clamp_int(var: tk.StringVar, lo: int, hi: int):
        try:
            val = int(float(var.get()))
        except ValueError:
            var.set(str(lo if lo > 0 else 0))
            return
        var.set(str(max(lo, min(hi, val))))

    @staticmethod
    def _validate_float(var: tk.StringVar):
        try:
            float(var.get())
        except ValueError:
            var.set("0.0")

    # ── Send handlers ──────────────────────────────────────────────────────────

    def _on_send_pwm(self):
        self._clamp_int(self.l_pwm, -100, 100)
        self._clamp_int(self.r_pwm, -100, 100)
        self._send(create_message(int(self.l_pwm.get()), int(self.r_pwm.get())))

    def _on_send_spd(self):
        self._clamp_int(self.l_spd, -1000, 1000)
        self._clamp_int(self.r_spd, -1000, 1000)
        self._send(create_spd_message(int(self.l_spd.get()), int(self.r_spd.get())))

    def _on_send_pose(self):
        self._validate_float(self.pose_x)
        self._validate_float(self.pose_y)
        self._validate_float(self.pose_theta)
        try:
            x = float(self.pose_x.get())
            y = float(self.pose_y.get())
            theta = float(self.pose_theta.get())
        except (tk.TclError, ValueError):
            mb.showerror("Input Error", "X, Y and θ must be numbers")
            return
        self._send(create_pose_message(x, y, theta))

    def _on_send_pid(self):
        for v in (self.kp, self.ki, self.kd, self.kff):
            self._validate_float(v)
        try:
            kp = float(self.kp.get())
            ki = float(self.ki.get())
            kd = float(self.kd.get())
            kff = float(self.kff.get())
        except (tk.TclError, ValueError):
            mb.showerror("Input Error", "PID values must be numbers")
            return
        self._send(create_pid_message(kp, ki, kd, kff))

    # ── Window close ──────────────────────────────────────────────────────────

    def _on_close(self):
        self._reading.clear()
        self._reader_thread.join(timeout=2)
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(b"0,0\n")
            except serial.SerialException:
                pass
            self.ser.close()
        self.destroy()


if __name__ == "__main__":
    app = DroneControl()
    app.mainloop()
