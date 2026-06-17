import serial
import time
import threading


def create_message(l_pwm, r_pwm):
    message = f"{l_pwm},{r_pwm}\n"
    return message


def prompt_command(ser, close_flag):
    while True:
        try:
            raw = input("Enter command or 'q' to quit: ").strip()
            if raw.lower() == "q":
                print("Exiting")
                message = "0,0\n"
                ser.write(message.encode("utf-8"))
                print(f"Sent: {message.strip()}")
                close_flag.set()
                break

            parts = raw.split(",")
            if len(parts) != 2:
                print("Invalid format.")
                continue
            try:
                l_pwm = int(parts[0].strip())
                r_pwm = int(parts[1].strip())
                if l_pwm > 100:
                    l_pwm = 100
                elif l_pwm < -100:
                    l_pwm = -100
                if r_pwm > 100:
                    r_pwm = 100
                elif r_pwm < -100:
                    r_pwm = -100
                message = create_message(l_pwm, r_pwm)
                ser.write(message.encode("utf-8"))
                print(f"Sent: {message.strip()}")
            except ValueError:
                print("Cant convert to int")
        except ValueError:
            print("Parse error")


def read_responce(ser, close_flag):
    while True:
        if ser.in_waiting > 0:
            raw = ser.read(ser.in_waiting)
            decoded = raw.decode("utf-8").strip()

            if decoded:
                print(f"Received: {decoded}")
        if close_flag.is_set():
            break
        time.sleep(0.1)


def main():
    port = "/dev/ttyACM0"
    baud_rate = 460800
    ser = None
    close_flag = threading.Event()

    try:
        # Open serial port
        ser = serial.Serial(port, baud_rate, timeout=5)
        time.sleep(1)  # Wait for connection to establish
        speed_thread = threading.Thread(target=prompt_command, args=(ser, close_flag))
        enc_thread = threading.Thread(target=read_responce, args=(ser, close_flag))

        speed_thread.start()
        enc_thread.start()

        speed_thread.join()
        enc_thread.join()

    except serial.SerialException as e:
        print(f"Serial port error: {e}")
        print("Check if:")
        print("  - Device is connected")
        print("  - Port /dev/ttyACM0 exists")
        print("  - You have permissions (try: sudo usermod -a -G dialout $USER)")

    except PermissionError:
        print("Permission denied accessing /dev/ttyACM0")
        print("Run: sudo chmod 666 /dev/ttyACM0")
        print("Or add user to dialout group: sudo usermod -a -G dialout $USER")

    except KeyboardInterrupt:
        print("\nExiting.")
    except Exception as e:
        print(f"Unexpected error: {type(e).__name__}: {e}")
    finally:
        if ser is not None and ser.is_open:
            print("Close serial port.")
            ser.close()


if __name__ == "__main__":
    main()
