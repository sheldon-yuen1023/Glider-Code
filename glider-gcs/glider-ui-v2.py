import tkinter as tk
from tkinter import ttk, messagebox
import threading
import json
import serial
import time

# === Configuration ===
COM_PORT = 'COM18'    # CANoverSerial bridge port
BAUD_RATE = 9600    # Match Arduino/CAN bridge baud

# === Serial Setup (for both sending commands and receiving JSON telemetry) ===
try:
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  # Allow port to settle
except serial.SerialException as e:
    messagebox.showerror("Serial Port Error", f"Could not open {COM_PORT}:\n{e}")
    ser = None

# -----------------------------------------------------------
# Shared Telemetry Dictionary + Lock
# -----------------------------------------------------------
telemetry_lock = threading.Lock()
telemetry = {
    'depth':       "N/A",
    'pitch':       "N/A",
    'roll':        "N/A",
    'yaw':         "N/A",
    'floor_dist':  "N/A",
    'leak':        "N/A",
    'vbd1':        "N/A",
    'vbd2':        "N/A",
    'pitchPos':    "N/A",
    'rollPos':     "N/A",
    'bms1':        "N/A",
    'bms2':        "N/A",
    'bms3':        "N/A",
    'bms4':        "N/A",
    'bms5':        "N/A"
}

# -----------------------------------------------------------
# Helper: Send a text command over serial
# -----------------------------------------------------------
def send_serial_command(cmd_str: str):
    """
    Send a command string over the serial link (ending with newline).
    """
    if not ser:
        return
    try:
        ser.write((cmd_str + "\n").encode('utf-8'))
        print(f"Sent: {cmd_str}")
    except Exception as e:
        print(f"Serial write error: {e}")

# -----------------------------------------------------------
# Command Functions
# -----------------------------------------------------------
def send_pitch():
    """Send a PITCH command: 'PITCH,<value>'."""
    try:
        pitch = int(entry_pitch.get())
    except ValueError:
        messagebox.showwarning("Invalid Input", "Pitch must be an integer.")
        return
    # Format: PITCH,<pitch>
    cmd = f"PITCH,{pitch}"
    send_serial_command(cmd)

def send_vbd():
    """Send a VBD command: 'VBD,IN/MID/OUT'."""
    state = vbd_state.get()
    cmd = f"VBD,{state}"
    send_serial_command(cmd)

def start_mission():
    """
    Send a START command:
      'START,<depthLimit>,<floorDist>,<cycles>,<diveAngle>,<riseAngle>'
    """
    try:
        depth_m     = float(entry_depth_limit.get())
        floor_m     = float(entry_floor_distance.get())
        cycles      = int(entry_cycles.get())
        dive_angle  = int(entry_dive_angle.get())
        rise_angle  = int(entry_rise_angle.get())
    except ValueError:
        messagebox.showwarning(
            "Invalid Input",
            "Depth/Floor must be numbers; Cycles/Angles must be integers."
        )
        return

    # Format floats with one decimal if needed
    cmd = f"START,{depth_m:.2f},{floor_m:.2f},{cycles},{dive_angle},{rise_angle}"
    send_serial_command(cmd)

def stop_mission():
    """Send 'STOP'."""
    send_serial_command("STOP")

def emergency_surface():
    """Send 'EMERGENCY'."""
    send_serial_command("EMERGENCY")

# -----------------------------------------------------------
# Serial Listener Thread: Parse JSON Telemetry
# -----------------------------------------------------------
def serial_listener():
    """
    Read lines from serial (each line is JSON),
    parse and update telemetry dict.
    """
    if not ser:
        return
    while True:
        try:
            raw = ser.readline().decode('utf-8').strip()
            if not raw:
                continue
            data = json.loads(raw)
            with telemetry_lock:
                # Pressure → depth
                pressure_kpa = data.get('sensors', {}).get('pressure')
                if pressure_kpa is not None:
                    # Rough conversion: 1 kPa ≈ 0.01 m depth (freshwater)
                    depth_m = (pressure_kpa - 101.3) * 0.01
                    telemetry['depth'] = f"{depth_m:.2f} m"
                else:
                    telemetry['depth'] = "N/A"

                # Orientation
                vehicle = data.get('vehicle', {})
                telemetry['pitch'] = f"{vehicle.get('pitch', 0):.2f}°"
                telemetry['roll']  = f"{vehicle.get('roll', 0):.2f}°"
                telemetry['yaw']   = f"{vehicle.get('yaw', 0):.2f}°"

                # Distance to bottom
                dist_bottom = data.get('sensors', {}).get('distanceToBottom')
                telemetry['floor_dist'] = f"{dist_bottom:.2f} m" if dist_bottom is not None else "N/A"

                # Leak sensors
                leaks = data.get('sensors', {}).get('leakSensors', {})
                telemetry['leak'] = "LEAK" if any(leaks.values()) else "OK"

                # Actuator positions
                actuators = data.get('actuators', {})
                telemetry['vbd1']     = f"{actuators.get('vbd1Position', 0):.1f}"
                telemetry['vbd2']     = f"{actuators.get('vbd2Position', 0):.1f}"
                telemetry['pitchPos'] = f"{actuators.get('pitchPosition', 0):.1f}"
                telemetry['rollPos']  = f"{actuators.get('rollPosition', 0):.1f}"

                # BMS readings for bms1..bms5
                bms_data = data.get('bms', {})
                for idx in range(1, 6):
                    key = f"bms{idx}"
                    entry = bms_data.get(key, {})
                    if entry:
                        parts = [f"{fld}: {val}" for fld, val in entry.items()]
                        telemetry[key] = "; ".join(parts)
                    else:
                        telemetry[key] = "N/A"
        except json.JSONDecodeError:
            continue
        except Exception:
            continue

# -----------------------------------------------------------
# Build the GUI
# -----------------------------------------------------------
root = tk.Tk()
root.title("Underwater Glider Control & Telemetry")
root.configure(bg="#2F343F")

style = ttk.Style(root)
style.theme_use("clam")
style.configure("TFrame", background="#2F343F")
style.configure("TLabel", background="#2F343F", foreground="#ECECEC", font=("Helvetica", 11))
style.configure("Header.TLabel", font=("Helvetica", 14, "bold"), foreground="#FFD966")
style.configure("TButton", font=("Helvetica", 11), padding=6)
style.configure("Danger.TButton", background="#D9534F", foreground="white", font=("Helvetica", 11, "bold"))

root.columnconfigure(0, weight=1, pad=10)
root.columnconfigure(1, weight=1, pad=10)
root.rowconfigure(0, weight=0, pad=10)
root.rowconfigure(1, weight=1, pad=10)

# --- Telemetry Frame ---
frame_telemetry = ttk.Frame(root, relief="ridge")
frame_telemetry.grid(row=0, column=0, columnspan=2, sticky="nsew", padx=10, pady=5)
for i in range(4):
    frame_telemetry.columnconfigure(i, weight=1)

lbl_tele_header = ttk.Label(frame_telemetry, text="Telemetry", style="Header.TLabel")
lbl_tele_header.grid(row=0, column=0, columnspan=4, pady=(5, 10))

lbl_depth     = ttk.Label(frame_telemetry, text="Depth: N/A")
lbl_depth.grid(row=1, column=0, padx=5, pady=2, sticky="w")

lbl_pitch_val = ttk.Label(frame_telemetry, text="Pitch: N/A")
lbl_pitch_val.grid(row=1, column=1, padx=5, pady=2, sticky="w")

lbl_roll_val  = ttk.Label(frame_telemetry, text="Roll: N/A")
lbl_roll_val.grid(row=1, column=2, padx=5, pady=2, sticky="w")

lbl_yaw_val   = ttk.Label(frame_telemetry, text="Yaw: N/A")
lbl_yaw_val.grid(row=1, column=3, padx=5, pady=2, sticky="w")

lbl_floor     = ttk.Label(frame_telemetry, text="Floor Dist: N/A")
lbl_floor.grid(row=2, column=0, padx=5, pady=2, sticky="w")

lbl_leak      = ttk.Label(frame_telemetry, text="Leak: N/A")
lbl_leak.grid(row=2, column=1, padx=5, pady=2, sticky="w")

lbl_vbd1      = ttk.Label(frame_telemetry, text="VBD1 Pos: N/A")
lbl_vbd1.grid(row=2, column=2, padx=5, pady=2, sticky="w")

lbl_vbd2      = ttk.Label(frame_telemetry, text="VBD2 Pos: N/A")
lbl_vbd2.grid(row=2, column=3, padx=5, pady=2, sticky="w")

lbl_pitchpos  = ttk.Label(frame_telemetry, text="Pitch Act Pos: N/A")
lbl_pitchpos.grid(row=3, column=0, padx=5, pady=2, sticky="w")

lbl_rollpos   = ttk.Label(frame_telemetry, text="Roll Act Pos: N/A")
lbl_rollpos.grid(row=3, column=1, padx=5, pady=2, sticky="w")

lbl_bms1      = ttk.Label(frame_telemetry, text="BMS1: N/A")
lbl_bms1.grid(row=4, column=0, padx=5, pady=2, sticky="w")

lbl_bms2      = ttk.Label(frame_telemetry, text="BMS2: N/A")
lbl_bms2.grid(row=4, column=1, padx=5, pady=2, sticky="w")

lbl_bms3      = ttk.Label(frame_telemetry, text="BMS3: N/A")
lbl_bms3.grid(row=4, column=2, padx=5, pady=2, sticky="w")

lbl_bms4      = ttk.Label(frame_telemetry, text="BMS4: N/A")
lbl_bms4.grid(row=4, column=3, padx=5, pady=2, sticky="w")

lbl_bms5      = ttk.Label(frame_telemetry, text="BMS5: N/A")
lbl_bms5.grid(row=5, column=0, padx=5, pady=2, sticky="w")


# --- Control Frame (Pitch Only / VBD) ---
frame_control = ttk.Frame(root, relief="ridge")
frame_control.grid(row=1, column=0, sticky="nsew", padx=10, pady=5)
frame_control.columnconfigure(0, weight=1)
frame_control.columnconfigure(1, weight=1)

lbl_control_header = ttk.Label(frame_control, text="Manual Controls", style="Header.TLabel")
lbl_control_header.grid(row=0, column=0, columnspan=2, pady=(5, 10))

ttk.Label(frame_control, text="Pitch (°):").grid(row=1, column=0, sticky="e", padx=5, pady=2)
entry_pitch = ttk.Entry(frame_control, width=10)
entry_pitch.grid(row=1, column=1, padx=5, pady=2)

btn_set_pitch = ttk.Button(
    frame_control,
    text="Set Pitch",
    command=send_pitch
)
btn_set_pitch.grid(row=2, column=0, columnspan=2, pady=8)

ttk.Label(frame_control, text="VBD State:").grid(row=3, column=0, sticky="e", padx=5, pady=2)
vbd_state = tk.StringVar(value="IN")
combo_vbd = ttk.Combobox(
    frame_control,
    textvariable=vbd_state,
    values=["IN", "MID", "OUT"],
    state="readonly",
    width=8
)
combo_vbd.grid(row=3, column=1, padx=5, pady=2)

btn_set_vbd = ttk.Button(
    frame_control,
    text="Set VBD",
    command=send_vbd
)
btn_set_vbd.grid(row=4, column=0, columnspan=2, pady=8)


# --- Mission Parameters Frame ---
frame_mission = ttk.Frame(root, relief="ridge")
frame_mission.grid(row=1, column=1, sticky="nsew", padx=10, pady=5)
for i in range(2):
    frame_mission.columnconfigure(i, weight=1)

lbl_mission_header = ttk.Label(frame_mission, text="Mission Parameters", style="Header.TLabel")
lbl_mission_header.grid(row=0, column=0, columnspan=2, pady=(5, 10))

ttk.Label(frame_mission, text="Depth Limit (m):").grid(row=1, column=0, sticky="e", padx=5, pady=2)
entry_depth_limit = ttk.Entry(frame_mission, width=10)
entry_depth_limit.grid(row=1, column=1, padx=5, pady=2)

ttk.Label(frame_mission, text="Floor Dist (m):").grid(row=2, column=0, sticky="e", padx=5, pady=2)
entry_floor_distance = ttk.Entry(frame_mission, width=10)
entry_floor_distance.grid(row=2, column=1, padx=5, pady=2)

ttk.Label(frame_mission, text="Number of Cycles:").grid(row=3, column=0, sticky="e", padx=5, pady=2)
entry_cycles = ttk.Entry(frame_mission, width=10)
entry_cycles.grid(row=3, column=1, padx=5, pady=2)

ttk.Label(frame_mission, text="Dive Angle (°):").grid(row=4, column=0, sticky="e", padx=5, pady=2)
entry_dive_angle = ttk.Entry(frame_mission, width=10)
entry_dive_angle.grid(row=4, column=1, padx=5, pady=2)

ttk.Label(frame_mission, text="Rise Angle (°):").grid(row=5, column=0, sticky="e", padx=5, pady=2)
entry_rise_angle = ttk.Entry(frame_mission, width=10)
entry_rise_angle.grid(row=5, column=1, padx=5, pady=2)

btn_start = ttk.Button(
    frame_mission,
    text="Start Mission",
    command=start_mission
)
btn_start.grid(row=6, column=0, columnspan=2, pady=(10, 5))

btn_stop = ttk.Button(
    frame_mission,
    text="Stop Mission",
    command=stop_mission
)
btn_stop.grid(row=7, column=0, columnspan=2, pady=5)

btn_emergency = ttk.Button(
    frame_mission,
    text="Emergency Surface",
    style="Danger.TButton",
    command=emergency_surface
)
btn_emergency.grid(row=8, column=0, columnspan=2, pady=(5, 10))


# -----------------------------------------------------------
# Update Telemetry Labels Periodically
# -----------------------------------------------------------
def update_telemetry_labels():
    with telemetry_lock:
        lbl_depth.config(text=f"Depth: {telemetry['depth']}")
        lbl_pitch_val.config(text=f"Pitch: {telemetry['pitch']}")
        lbl_roll_val.config(text=f"Roll: {telemetry['roll']}")
        lbl_yaw_val.config(text=f"Yaw: {telemetry['yaw']}")
        lbl_floor.config(text=f"Floor Dist: {telemetry['floor_dist']}")
        # If leak detected, show in red
        if telemetry['leak'] == "LEAK":
            lbl_leak.config(text="Leak: LEAK", foreground="#D9534F")
        else:
            lbl_leak.config(text="Leak: OK", foreground="#ECECEC")

        lbl_vbd1.config(text=f"VBD1 Pos: {telemetry['vbd1']}")
        lbl_vbd2.config(text=f"VBD2 Pos: {telemetry['vbd2']}")
        lbl_pitchpos.config(text=f"Pitch Act Pos: {telemetry['pitchPos']}")
        lbl_rollpos.config(text=f"Roll Act Pos: {telemetry['rollPos']}")

        lbl_bms1.config(text=f"BMS1: {telemetry['bms1']}")
        lbl_bms2.config(text=f"BMS2: {telemetry['bms2']}")
        lbl_bms3.config(text=f"BMS3: {telemetry['bms3']}")
        lbl_bms4.config(text=f"BMS4: {telemetry['bms4']}")
        lbl_bms5.config(text=f"BMS5: {telemetry['bms5']}")
    root.after(200, update_telemetry_labels)

# -----------------------------------------------------------
# Graceful Shutdown
# -----------------------------------------------------------
def on_closing():
    if messagebox.askokcancel("Quit", "Do you really want to quit?"):
        root.destroy()

root.protocol("WM_DELETE_WINDOW", on_closing)

# -----------------------------------------------------------
# Start Serial Listener Thread & GUI Loop
# -----------------------------------------------------------
if ser:
    threading.Thread(target=serial_listener, daemon=True).start()

root.after(200, update_telemetry_labels)
root.mainloop()
