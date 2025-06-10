import tkinter as tk
from tkinter import ttk, messagebox
import threading
import json
import serial
import time
from time import time as current_time  # High-resolution timestamp
import csv

# === Configuration ===
COM_PORT = 'COM18'    # CANoverSerial bridge port
BAUD_RATE = 9600      # Match Arduino/CAN bridge baud

# === Serial Setup (for both sending commands and receiving JSON telemetry) ===
try:
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  # Allow port to settle
except serial.SerialException as e:
    messagebox.showerror("Serial Port Error", f"Could not open {COM_PORT}:\n{e}")
    ser = None

# -----------------------------------------------------------
# Timing Control to Avoid Collisions
# -----------------------------------------------------------
last_telemetry_time = 0
TELEMETRY_GRACE_MS = 30  # 30 ms after telemetry during which commands wait

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
    'bms':         "N/A",  # Only show BMS2 as "bms"
}

# -----------------------------------------------------------
# CSV Logging Setup (in Downloads folder)
# -----------------------------------------------------------
csv_file = open(r'C:\Users\zhara\Downloads\telemetry_log.csv', mode='w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow([
    'timestamp', 'depth', 'pitch', 'rolssl', 'yaw',
    'floor_dist', 'leak', 'vbd1', 'vbd2',
    'pitchPos', 'rollPos', 'bms'
])


# -----------------------------------------------------------
# Helper: Send a text command over serial with delay logic
# -----------------------------------------------------------
def send_serial_command(cmd_str: str):
    """
    Send a command string over serial, ensuring at least TELEMETRY_GRACE_MS have passed
    since the last telemetry JSON was received.
    """
    global last_telemetry_time

    if not ser:
        return

    # Calculate elapsed time since last telemetry in milliseconds
    elapsed_ms = (current_time() - last_telemetry_time) * 1000
    if elapsed_ms < TELEMETRY_GRACE_MS:
        wait_time = (TELEMETRY_GRACE_MS - elapsed_ms) / 1000.0
        time.sleep(wait_time)

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
    cmd = f"PITCH,{pitch}"
    send_serial_command(cmd)

def send_vbd():
    """Send a VBD command: 'VBD,IN/MID/OUT'."""
    state = vbd_state.get()
    cmd = f"VBD,{state}"
    send_serial_command(cmd)

def send_vbd1():

    state = vbd1_state.get()
    cmd = f"VBD1,{state}"
    send_serial_command(cmd)

def send_vbd2():
    """Send a VBD2 command: 'VBD2,IN/MID/OUT'."""
    state = vbd2_state.get()
    cmd = f"VBD2,{state}"
    send_serial_command(cmd)


def stop_vbd():
    """Send a VBD STOP command: 'VBD,STOP'."""
    send_serial_command("VBD,STOP")

def zero_vbd():
    """Send a Zero VBD command: 'VBD,ZERO'."""
    confirm = messagebox.askyesno(
        title="Confirm Zero VBD",
        message="Are you sure you want to zero the VBD?"
    )
    if confirm:
        send_serial_command("VBD,ZERO")

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

    cmd = f"START,{depth_m:.2f},{floor_m:.2f},{cycles},{dive_angle},{rise_angle}"
    send_serial_command(cmd)

def stop_mission():
    """Send 'STOP'."""
    send_serial_command("STOP")

def emergency_surface():
    """Prompt confirmation and send 'EMERGENCY' if confirmed."""
    confirm = messagebox.askyesno(
        title="Confirm Emergency Surface",
        message="Are you sure you want to Emergency Surface?"
    )
    if confirm:
        send_serial_command("EMERGENCY")

def battery_off():
    """Prompt confirmation and send 'BATT_OFF' if confirmed."""
    confirm = messagebox.askyesno(
        title="Confirm Battery Off",
        message="Are you sure you want to turn Battery Off?"
    )
    if confirm:
        send_serial_command("BATT_OFF")

# -----------------------------------------------------------
# Serial Listener Thread: Parse JSON Telemetry
# -----------------------------------------------------------
def serial_listener():
    """
    Read lines from serial (each line is JSON),
    parse and update telemetry dict, and record timestamp.
    """
    global last_telemetry_time

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
                telemetry['floor_dist'] = (
                    f"{dist_bottom:.2f} m" if dist_bottom is not None else "N/A"
                )

                # Leak sensors
                leaks = data.get('sensors', {}).get('leakSensors', {})
                telemetry['leak'] = "LEAK" if any(leaks.values()) else "OK"

                # Actuator positions
                actuators = data.get('actuators', {})

                # vbd1 / vbd2 are nested under "vbd1" and "vbd2" keys
                vbd1_obj = actuators.get('vbd1', {})
                vbd2_obj = actuators.get('vbd2', {})
                telemetry['vbd1'] = f"{vbd1_obj.get('position', 0):.1f}"
                telemetry['vbd2'] = f"{vbd2_obj.get('position', 0):.1f}"

                # pitchPosition and rollPosition remain as direct fields
                telemetry['pitchPos'] = f"{actuators.get('pitchPosition', 0):.1f}"
                telemetry['rollPos']  = f"{actuators.get('rollPosition', 0):.1f}"

                # Only read BMS2 and store as 'bms'
                bms2_entry = data.get('bms', {}).get('bms2', {})
                if bms2_entry:
                    parts = [f"{fld}: {val}" for fld, val in bms2_entry.items()]
                    telemetry['bms'] = "; ".join(parts)
                else:
                    telemetry['bms'] = "N/A"

            # ← INSERT CSV LOGGING HERE →
            csv_writer.writerow([
                time.strftime("%Y-%m-%d %H:%M:%S"),  # Timestamp
                telemetry['depth'],
                telemetry['pitch'],
                telemetry['roll'],
                telemetry['yaw'],
                telemetry['floor_dist'],
                telemetry['leak'],
                telemetry['vbd1'],
                telemetry['vbd2'],
                telemetry['pitchPos'],
                telemetry['rollPos'],
                telemetry['bms']
            ])

            # Record the time telemetry was received
            last_telemetry_time = current_time()

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
for i in range(3):
    frame_telemetry.columnconfigure(i, weight=1)

# === TELEMETRY GROUPING (MODIFIED) ===
# Group 1: Depth and Floor Distance
frame_depth_floor = ttk.Frame(frame_telemetry)
frame_depth_floor.grid(row=1, column=0, padx=5, pady=2, sticky="w")
lbl_depth = ttk.Label(frame_depth_floor, text="Depth: N/A")
lbl_depth.pack(anchor="w")
lbl_floor = ttk.Label(frame_depth_floor, text="Floor Dist: N/A")
lbl_floor.pack(anchor="w")

# Group 2: Pitch, Roll, Yaw
frame_attitude = ttk.Frame(frame_telemetry)
frame_attitude.grid(row=1, column=1, padx=5, pady=2, sticky="w")
lbl_pitch_val = ttk.Label(frame_attitude, text="Pitch: N/A")
lbl_pitch_val.pack(anchor="w")
lbl_roll_val = ttk.Label(frame_attitude, text="Roll: N/A")
lbl_roll_val.pack(anchor="w")
lbl_yaw_val = ttk.Label(frame_attitude, text="Yaw: N/A")
lbl_yaw_val.pack(anchor="w")

# Group 3: VBD1, VBD2, Leak
frame_vbd_leak = ttk.Frame(frame_telemetry)
frame_vbd_leak.grid(row=1, column=2, padx=5, pady=2, sticky="w")
lbl_vbd1 = ttk.Label(frame_vbd_leak, text="VBD1 Pos: N/A")
lbl_vbd1.pack(anchor="w")
lbl_vbd2 = ttk.Label(frame_vbd_leak, text="VBD2 Pos: N/A")
lbl_vbd2.pack(anchor="w")
lbl_leak = ttk.Label(frame_vbd_leak, text="Leak: N/A")
lbl_leak.pack(anchor="w")

# Group 4: Pitch and Roll Actuator Positions
frame_actuators = ttk.Frame(frame_telemetry)
frame_actuators.grid(row=2, column=0, padx=5, pady=2, sticky="w")
lbl_pitchpos = ttk.Label(frame_actuators, text="Pitch Act Pos: N/A")
lbl_pitchpos.pack(anchor="w")
lbl_rollpos = ttk.Label(frame_actuators, text="Roll Act Pos: N/A")
lbl_rollpos.pack(anchor="w")

# Group 5: BMS Info
frame_bms = ttk.Frame(frame_telemetry)
frame_bms.grid(row=2, column=1, padx=5, pady=2, sticky="w")
lbl_bms = ttk.Label(frame_bms, text="BMS: N/A")
lbl_bms.pack(anchor="w")

# === Instruction Box ===
frame_instructions = ttk.Frame(root, relief="ridge")
frame_instructions.grid(row=2, column=0, columnspan=2, sticky="ew", padx=10, pady=(5,10))
tt_instruction = (
    "IMPORTANT: Before sending any commands or starting a mission,\n"
    "the VBDs must be zeroed using the 'Zero VBD' button.\n"
    "Failure to do so may cause incorrect depth regulation and instability."
)
lbl_instructions = ttk.Label(frame_instructions, text=tt_instruction, foreground="#FFD966", font=("Helvetica", 10, "italic"))
lbl_instructions.pack(padx=5, pady=5)

# --- Control Frame (Pitch Only / VBD / Zero VBD / Stop VBD) ---
frame_control = ttk.Frame(root, relief="ridge")
frame_control.grid(row=1, column=0, sticky="nsew", padx=10, pady=5)
frame_control.columnconfigure(0, weight=1)
frame_control.columnconfigure(1, weight=1)

lbl_control_header = ttk.Label(frame_control, text="Manual Controls", style="Header.TLabel")
lbl_control_header.grid(row=0, column=0, columnspan=2, pady=(5, 10))

# Pitch controls
ttk.Label(frame_control, text="Pitch (mm):").grid(row=1, column=0, sticky="e", padx=5, pady=2)
entry_pitch = ttk.Entry(frame_control, width=10)
entry_pitch.grid(row=1, column=1, padx=5, pady=2)

btn_set_pitch = ttk.Button(
    frame_control,
    text="Set Pitch (mm)",
    command=send_pitch
)
btn_set_pitch.grid(row=2, column=0, columnspan=2, pady=8)

# VBD controls
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

# Stop VBD button
btn_stop_vbd = ttk.Button(
    frame_control,
    text="Stop VBD",
    command=stop_vbd
)
btn_stop_vbd.grid(row=5, column=0, columnspan=2, pady=8)

# Zero VBD button
btn_zero_vbd = ttk.Button(
    frame_control,
    text="Zero VBD",
    style="Danger.TButton",
    command=zero_vbd
)
btn_zero_vbd.grid(row=6, column=0, columnspan=2, pady=(0, 10))

# Manual VBD1 control
ttk.Label(frame_control, text="VBD1 State:").grid(row=7, column=0, sticky="e", padx=5, pady=2)
vbd1_state = tk.StringVar(value="IN")
combo_vbd1 = ttk.Combobox(
    frame_control,
    textvariable=vbd1_state,
    values=["IN", "MID", "OUT"],
    state="readonly",
    width=8
)
combo_vbd1.grid(row=7, column=1, padx=5, pady=2)

btn_set_vbd1 = ttk.Button(
    frame_control,
    text="Set VBD1",
    command=send_vbd1
)
btn_set_vbd1.grid(row=8, column=0, columnspan=2, pady=5)

# Manual VBD2 control
ttk.Label(frame_control, text="VBD2 State:").grid(row=9, column=0, sticky="e", padx=5, pady=2)
vbd2_state = tk.StringVar(value="IN")
combo_vbd2 = ttk.Combobox(
    frame_control,
    textvariable=vbd2_state,
    values=["IN", "MID", "OUT"],
    state="readonly",
    width=8
)
combo_vbd2.grid(row=9, column=1, padx=5, pady=2)

btn_set_vbd2 = ttk.Button(
    frame_control,
    text="Set VBD2",
    command=send_vbd2
)
btn_set_vbd2.grid(row=10, column=0, columnspan=2, pady=5)

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
btn_emergency.grid(row=8, column=0, columnspan=2, pady=(5, 5))

btn_battery_off = ttk.Button(
    frame_mission,
    text="Battery Off",
    style="Danger.TButton",
    command=battery_off
)
btn_battery_off.grid(row=9, column=0, columnspan=2, pady=(0, 10))

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

        # Only one BMS label, showing BMS2 as "BMS"
        lbl_bms.config(text=f"BMS: {telemetry['bms']}")

    root.after(200, update_telemetry_labels)

# -----------------------------------------------------------
# Graceful Shutdown
# -----------------------------------------------------------
def on_closing():
    if messagebox.askokcancel("Quit", "Do you really want to quit?"):
        root.destroy()
        csv_file.close()    # ← CLOSE YOUR LOG HERE

root.protocol("WM_DELETE_WINDOW", on_closing)

# -----------------------------------------------------------
# Start Serial Listener Thread & GUI Loop
# -----------------------------------------------------------
if ser:
    threading.Thread(target=serial_listener, daemon=True).start()

root.after(200, update_telemetry_labels)
root.mainloop()
