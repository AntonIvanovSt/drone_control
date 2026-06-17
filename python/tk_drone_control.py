import tkinter as tk
import tkinter.messagebox as mb
import serial
import time
import threading


def create_message(l_pwm: int, r_pwm: int) -> str:
    return f"{l_pwm},{r_pwm}\n"


class DroneControl(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Drone Control")
        self.geometry("640x480")
        self.resizable(False, False)
        self.configure(bg="#0d0f14")

        self.l_pwm = tk.IntVar(value=0)
        self.r_pwm = tk.IntVar(value=0)
        self.status_var = tk.StringVar(value="Not connected")

        self.ser = None
        self._open_serial()
        self._build_ui()

        self._reading = threading.Event()
        self._reading.set()
        self._reader_thread = threading.Thread(target=self._serial_reader, daemon=True)
        self._reader_thread.start()

        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _open_serial(self, port="/dev/ttyACM0", baud=460800):
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            time.sleep(1)
            self.status_var.set(f"Connected: {port}")
        except serial.SerialException as e:
            self.ser = None
            self.status_var.set(f"Serial error: {e}")

    def _send(self, message: str):
        if self.ser is None or not self.ser.is_open:
            mb.showerror("Serial Error", "Serial port is not open")
            return
        try:
            self.ser.write(message.encode("utf-8"))
            self.status_var.set(f"Sent: {message.strip()}")
        except serial.SerialException as e:
            self.status_var.set(f"Send error: {e}")

    def _serial_reader(self):
        while self._reading.is_set():
            if self.ser is None or not self.ser.is_open:
                time.sleep(0.1)
                continue
            try:
                line = self.ser.readline()
                if line:
                    text = line.decode("utf-8", errors="replace").strip()
                    self.after(0, self.status_var.set, f"ESP32: {text}")
            except serial.SerialException:
                break

    def _build_ui(self):
        tk.Label(
            self,
            text="Drone Controller",
            font=("Arial", 14),
            fg="#8892a4",
            bg="#0d0f14",
        ).pack(pady=(20, 5))

        btn_frame = tk.Frame(self, bg="#0d0f14")
        btn_frame.pack(pady=15)

        tk.Scale(
            btn_frame,
            from_=-100,
            to=100,
            orient="horizontal",
            variable=self.l_pwm,
            label="Left PWM",
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
        ).pack(pady=10)

        tk.Scale(
            btn_frame,
            from_=-100,
            to=100,
            orient="horizontal",
            variable=self.r_pwm,
            label="Right PWM",
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
        ).pack(pady=10)

        tk.Button(
            btn_frame,
            text="Send",
            command=self._on_click,
            font=("Arial", 12),
            fg="white",
            bg="#5b8af0",
            activebackground="#3a6ad4",
            relief="flat",
            padx=16,
            pady=8,
            cursor="hand2",
        ).pack(pady=10)

        tk.Label(
            self,
            textvariable=self.status_var,
            font=("Arial", 10),
            fg="#8892a4",
            bg="#0d0f14",
        ).pack(side="bottom", pady=10)

    def _on_click(self):
        l_p = self.l_pwm.get()
        r_p = self.r_pwm.get()
        message = create_message(l_p, r_p)
        self._send(message)

    def _on_close(self):
        self._reading.clear()
        self._reader_thread.join()
        self._send("0,0\n")
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.destroy()


if __name__ == "__main__":
    app = DroneControl()
    app.mainloop()
