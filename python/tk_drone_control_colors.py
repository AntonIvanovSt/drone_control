import tkinter as tk
import tkinter.font as tkfont
import serial
import time
import threading
import re
import matplotlib

matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.font_manager as fm

# ── Palette ────────────────────────────────────────────────────────────────────
BG = "#000000"  # pure black – all panels
BG_GRAPH = "#1a1a1a"  # dark-gray graph area only
FG = "#ffffff"  # white text
BORDER = "#ffffff"  # white borders / dividers
SCALE_TROUGH = "#1a1a1a"  # trough matches graph gray

# Graph line colours – cyan & magenta pair
C_ML = "#00e5ff"  # measured left  – bright cyan
C_TL = "#007a8a"  # target left    – dim cyan, dashed
C_MR = "#ff00cc"  # measured right – bright magenta
C_TR = "#880066"  # target right   – dim magenta, dashed


def resolve_font(preferred_families, size, bold=False):
    weight = "bold" if bold else "normal"
    available = set(f.lower() for f in tkfont.families())
    for family in preferred_families:
        if family.lower() in available:
            return tkfont.Font(family=family, size=size, weight=weight)
    return tkfont.Font(family="Courier", size=size, weight=weight)


def resolve_mpl_font(preferred_families):
    available = {f.name for f in fm.fontManager.ttflist}
    for family in preferred_families:
        if family in available:
            return family
    return "monospace"


MONO_FAMILIES = [
    "JetBrainsMono Nerd Font",
    "JetBrains Mono",
    "JetBrainsMono NF",
    "JetBrainsMono NFM",
    "FiraCode Nerd Font",
    "Fira Code",
    "Hack Nerd Font",
    "Hack",
    "Cascadia Code",
    "Inconsolata",
    "Liberation Mono",
    "DejaVu Sans Mono",
    "Courier New",
    "Courier",
]


class SerialReader(threading.Thread):
    def __init__(self, serial_port, baudrate, callback):
        super().__init__()
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.callback = callback
        self._stop_event = threading.Event()
        try:
            self.ser = serial.Serial(serial_port, baudrate, timeout=1)
        except Exception as e:
            print("Error opening serial port:", e)
            self.ser = None

    def run(self):
        if not self.ser:
            return
        while not self._stop_event.is_set():
            try:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if line:
                    self.callback(line)
            except Exception as e:
                print("Error reading from serial:", e)
            time.sleep(0.01)

    def stop(self):
        self._stop_event.set()
        if self.ser:
            self.ser.close()


class DroneControl(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Drone Control")
        self.resizable(False, False)
        self.configure(bg=BG)

        self.font_ui = resolve_font(MONO_FAMILIES, 9)
        self.font_label = resolve_font(MONO_FAMILIES, 9, bold=True)
        self.font_header = resolve_font(MONO_FAMILIES, 10, bold=True)
        self.font_log = resolve_font(MONO_FAMILIES, 9)
        self.mpl_family = resolve_mpl_font(MONO_FAMILIES)

        self.serial_port = "/dev/ttyACM0"
        self.baudrate = 460800
        self.serial_thread = None

        self.create_ui()

        self.time_data = []
        self.measured_left_data = []
        self.target_left_data = []
        self.measured_right_data = []
        self.target_right_data = []
        self.start_time = time.time()

        self.start_serial()
        self.update_plot()

    # ── Layout ─────────────────────────────────────────────────────────────────

    def create_ui(self):
        self.geometry("1300x780")
        self.grid_columnconfigure(0, weight=0)
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)

        left = tk.Frame(self, bg=BG, highlightbackground=BORDER, highlightthickness=1)
        right = tk.Frame(self, bg=BG)
        left.grid(row=0, column=0, sticky="nsew", padx=(16, 0), pady=16)
        right.grid(row=0, column=1, sticky="nsew", padx=(16, 16), pady=16)

        right.grid_columnconfigure(0, weight=1)
        right.grid_rowconfigure(0, weight=3)
        right.grid_rowconfigure(2, weight=1)

        self.build_left(left)
        self.build_right(right)

    # ── Left panel helpers ──────────────────────────────────────────────────────

    def _divider(self, parent, row):
        tk.Frame(parent, height=1, bg=BORDER).grid(
            row=row, column=0, sticky="ew", padx=10, pady=5
        )

    def _section_label(self, parent, text, row):
        tk.Label(
            parent,
            text=text,
            bg=BG,
            fg=FG,
            font=self.font_header,
        ).grid(row=row, column=0, sticky="w", padx=14, pady=(12, 2))

    def _param_label(self, parent, text, row):
        tk.Label(
            parent,
            text=text,
            bg=BG,
            fg=FG,
            font=self.font_label,
        ).grid(row=row, column=0, sticky="w", padx=14, pady=(6, 0))

    def _scale(self, parent, row, **kwargs):
        s = tk.Scale(
            parent,
            orient=tk.HORIZONTAL,
            length=210,
            bg=BG,
            fg=FG,
            troughcolor=SCALE_TROUGH,
            activebackground=BORDER,
            highlightthickness=0,
            bd=0,
            font=self.font_ui,
            **kwargs,
        )
        s.grid(row=row, column=0, padx=14, pady=(0, 4))
        return s

    def build_left(self, parent: tk.Frame):
        parent.grid_columnconfigure(0, weight=1)

        self._section_label(parent, "PID COEFFICIENTS", 0)

        self._param_label(parent, "Kp", 1)
        self.kp_scale = self._scale(
            parent,
            2,
            from_=0,
            to=1,
            resolution=0.01,
            command=self.update_coefficients_from_sliders,
        )
        self.kp_scale.set(0.0)

        self._divider(parent, 3)

        self._param_label(parent, "Ki", 4)
        self.ki_scale = self._scale(
            parent,
            5,
            from_=0,
            to=1,
            resolution=0.01,
            command=self.update_coefficients_from_sliders,
        )
        self.ki_scale.set(0.3)

        self._divider(parent, 6)

        self._param_label(parent, "Kd", 7)
        self.kd_scale = self._scale(
            parent,
            8,
            from_=0,
            to=1,
            resolution=0.01,
            command=self.update_coefficients_from_sliders,
        )
        self.kd_scale.set(0.0)

        self._divider(parent, 9)

        self._param_label(parent, "Kff", 10)
        self.kff_scale = self._scale(
            parent,
            11,
            from_=0,
            to=1,
            resolution=0.01,
            command=self.update_coefficients_from_sliders,
        )
        self.kff_scale.set(0.35)

        # ── major divider between sections ────────────────────────────────────
        tk.Frame(parent, height=1, bg=BORDER).grid(
            row=12, column=0, sticky="ew", padx=10, pady=(14, 6)
        )

        self._section_label(parent, "TARGET WHEEL SPEEDS", 13)

        self._param_label(parent, "Left  (mm/s)", 14)
        self.target_left_scale = self._scale(
            parent,
            15,
            from_=-500,
            to=500,
            resolution=1,
            command=self.update_speed_from_sliders,
        )
        self.target_left_scale.set(0)

        self._param_label(parent, "Right  (mm/s)", 17)
        self.target_right_scale = self._scale(
            parent,
            18,
            from_=-500,
            to=500,
            resolution=1,
            command=self.update_speed_from_sliders,
        )
        self.target_right_scale.set(0)

        self._section_label(parent, "LINEAR and ANGULAR", 19)

        self._param_label(parent, "Linear  (mm/s)", 20)
        self.linear_spd_scale = self._scale(
            parent,
            21,
            from_=-500,
            to=500,
            resolution=1,
            # command=self.update_speed_from_sliders,
        )
        self.target_left_scale.set(0)

        self._param_label(parent, "Angular  (rad/s)", 22)
        self.angular_spd_scale = self._scale(
            parent,
            23,
            from_=-1,
            to=1,
            resolution=0.1,
            # command=self.update_speed_from_sliders,
        )
        self.target_right_scale.set(0)
        tk.Button(parent, text="Send L/A", command=self.update_linear_and_angular).grid(
            row=24, column=0, sticky="ew", pady=(8, 0)
        )

    # ── Right panel ─────────────────────────────────────────────────────────────

    def build_right(self, parent: tk.Frame):
        # ── Chart ─────────────────────────────────────────────────────────────
        self.figure = Figure(figsize=(12, 4), dpi=100)
        self.figure.patch.set_facecolor(BG)

        self.ax = self.figure.add_subplot(111)
        self._style_axes()

        self.canvas = FigureCanvasTkAgg(self.figure, master=parent)
        self.canvas.get_tk_widget().configure(bg=BG, highlightthickness=0)
        self.canvas.get_tk_widget().grid(
            row=0, column=0, columnspan=2, sticky="nsew", pady=(0, 8)
        )

        # ── white 1-px divider ─────────────────────────────────────────────────
        tk.Frame(parent, height=1, bg=BORDER).grid(
            row=1, column=0, columnspan=2, sticky="ew", pady=(0, 6)
        )

        # ── Log header ────────────────────────────────────────────────────────
        tk.Label(
            parent,
            text="LOG",
            bg=BG,
            fg=FG,
            font=self.font_header,
        ).grid(row=2, column=0, sticky="w", pady=(0, 4))

        log_border = tk.Frame(parent, bg=BORDER, padx=1, pady=1)
        log_border.grid(row=3, column=0, columnspan=2, sticky="nsew")
        log_border.grid_columnconfigure(0, weight=1)
        log_border.grid_rowconfigure(0, weight=1)

        scrollbar = tk.Scrollbar(
            log_border, bg=BG, troughcolor=BG, activebackground=BORDER
        )
        scrollbar.grid(row=0, column=1, sticky="ns")

        self.status_text = tk.Text(
            log_border,
            bg=BG,
            fg=FG,
            insertbackground=FG,
            relief="flat",
            state="disabled",
            yscrollcommand=scrollbar.set,
            wrap="word",
            padx=8,
            pady=6,
            cursor="arrow",
            height=5,
            font=self.font_log,
        )
        self.status_text.grid(row=0, column=0, sticky="nsew")
        scrollbar.config(command=self.status_text.yview)
        parent.grid_rowconfigure(3, weight=1)

    def _style_axes(self):
        self.ax.set_facecolor(BG_GRAPH)
        self.ax.set_title(
            "Wheel Speed Response",
            color=FG,
            pad=10,
            fontfamily=self.mpl_family,
            fontsize=10,
            fontweight="bold",
        )
        self.ax.set_xlabel("Time (s)", color=FG, fontfamily=self.mpl_family, fontsize=9)
        self.ax.set_ylabel(
            "Speed (mm/s)", color=FG, fontfamily=self.mpl_family, fontsize=9
        )
        self.ax.tick_params(colors=FG, labelsize=8)
        for spine in self.ax.spines.values():
            spine.set_edgecolor(BORDER)
            spine.set_linewidth(0.8)
        self.ax.grid(color="#2a2a2a", linewidth=0.6)
        for label in self.ax.get_xticklabels() + self.ax.get_yticklabels():
            label.set_fontfamily(self.mpl_family)

    # ── Serial ──────────────────────────────────────────────────────────────────

    def start_serial(self):
        self.serial_thread = SerialReader(
            self.serial_port, self.baudrate, self.handle_serial_line
        )
        self.serial_thread.start()

    def handle_serial_line(self, line):
        self.status_text.configure(state="normal")
        self.status_text.insert(tk.END, line + "\n")
        self.status_text.see(tk.END)
        self.status_text.configure(state="disabled")

        pattern = (
            r"SPD L=([\d\.\-]+) mm/s R=([\d\.\-]+) mm/s"
            r".*Target L=([\d\.\-]+) mm/s R=([\d\.\-]+) mm/s"
        )
        match = re.search(pattern, line)
        if match:
            try:
                measured_left = float(match.group(1))
                measured_right = float(match.group(2))
                target_left = float(match.group(3))
                target_right = float(match.group(4))
                t = time.time() - self.start_time
                self.time_data.append(t)
                self.measured_left_data.append(measured_left)
                self.target_left_data.append(target_left)
                self.measured_right_data.append(measured_right)
                self.target_right_data.append(target_right)
            except Exception as e:
                print("Error parsing speed data:", e)

    # ── Commands ────────────────────────────────────────────────────────────────

    def _send(self, cmd):
        if self.serial_thread and self.serial_thread.ser:
            try:
                self.serial_thread.ser.write(cmd.encode("utf-8"))
                self.status_text.configure(state="normal")
                self.status_text.insert(tk.END, "Sent: " + cmd)
                self.status_text.see(tk.END)
                self.status_text.configure(state="disabled")
            except Exception as e:
                print("Error sending command:", e)

    def update_coefficients_from_sliders(self, _value):
        self._send(
            "SET_COEFF {} {} {} {}\n".format(
                self.kp_scale.get(),
                self.ki_scale.get(),
                self.kd_scale.get(),
                self.kff_scale.get(),
            )
        )

    def update_linear_and_angular(self):
        self._send(
            "L_A_SPD {} {}\n".format(
                self.linear_spd_scale.get(), self.angular_spd_scale.get()
            )
        )

    def update_speed_from_sliders(self, _value):
        self._send(
            "SPD L={} R={}\n".format(
                self.target_left_scale.get(), self.target_right_scale.get()
            )
        )

    # ── Plot ────────────────────────────────────────────────────────────────────

    def update_plot(self):
        self.ax.clear()
        self._style_axes()

        self.ax.plot(
            self.time_data,
            self.measured_left_data,
            label="Measured Left",
            color=C_ML,
            linewidth=1.6,
        )
        self.ax.plot(
            self.time_data,
            self.target_left_data,
            label="Target Left",
            color=C_TL,
            linewidth=1.2,
            linestyle="--",
        )
        self.ax.plot(
            self.time_data,
            self.measured_right_data,
            label="Measured Right",
            color=C_MR,
            linewidth=1.6,
        )
        self.ax.plot(
            self.time_data,
            self.target_right_data,
            label="Target Right",
            color=C_TR,
            linewidth=1.2,
            linestyle="--",
        )

        self.ax.legend(
            facecolor=BG_GRAPH,
            edgecolor=BORDER,
            labelcolor=FG,
            prop={"family": self.mpl_family, "size": 8},
        )

        if self.time_data:
            t = self.time_data[-1]
            self.ax.set_xlim(0, 20) if t < 20 else self.ax.set_xlim(t - 20, t)

        self.canvas.draw()
        self.after(1000, self.update_plot)

    def on_closing(self):
        if self.serial_thread:
            self.serial_thread.stop()
        self.destroy()


if __name__ == "__main__":
    app = DroneControl()
    app.mainloop()
