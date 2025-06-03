import tkinter as tk
from tkinter import ttk, messagebox
import struct
import threading

# Ensure python-can is installed: pip install python-can
import can

# -----------------------------------------------------------
# CAN Bus Initialization
# -----------------------------------------------------------
try:
    # Adjust channel and bustype as needed for your platform
    bus = can.interface.Bus(channel='can0', bustype='socketcan')
except Exception as e:
    messagebox.showerror(
        "CAN Bus Error",
        f"Could not initialize CAN interface:\n{e}"
    )
    bus = None

# -----------------------------------------------------------
# Helper Functions to Pack and Send CAN Messages
# -----------------------------------------------------------
def send_can_message(arbitration_id: int, data: bytes):
    """
    Send a CAN message with the given ID and data payload.
    """
    if not bus:
        return
    try:
        msg = can.Message(
            arbitration_id=arbitration_id,
            data=data,
            is_extended_id=False
        )
        bus.send(msg)
    except Exception as e:
        print(f"CAN send error (ID 0x{arbitration_id:X}): {e}")


def send_pitch():
    """
    Read pitch from entry, pack into int16 little-endian with roll=0,
    and send under CAN ID 0x100.
    """
    try:
        pitch = int(entry_pitch.get())
    except ValueError:
        messagebox.showwarning("Invalid Input", "Pitch must be an integer.")
        return

    roll = 0  # No manual roll control; send 0
    data = struct.pack('<hh', pitch, roll)
    send_can_message(0x100, data)
    print(f"Sent PITCH -> pitch: {pitch}, roll: {roll} (fixed)")


def send_vbd():
    """
    Read VBD state (IN=0, MID=1, OUT=2) and send under CAN ID 0x101.
    """
    state_str = vbd_state.get()
    state_map = {'IN': 0, 'MID': 1, 'OUT': 2}
    state = state_map.get(state_str, 0)
    data = struct.pack('<B', state)
    send_can_message(0x101, data)
    print(f"Sent VBD -> state: {state_str}")


def start_mission():
    """
    Collect mission parameters, convert to required units, pack into bytes,
    and send under CAN ID 0x102.
    """
    try:
        depth_m = float(entry_depth_limit.get())
        floor_m = float(entry_floor_distance.get())
        cycles = int(entry_cycles.get())
        dive_angle = int(entry_dive_angle.get())
        rise_angle = int(entry_rise_angle.get())
    except ValueError:
        messagebox.showwarning(
            "Invalid Input",
            "Make sure Depth, Floor Distance are numbers and Cycles/Angles are integers."
        )
        return

    # Convert meters to centimeters and pack as signed int16
    depth_cm = int(depth_m * 100)
    floor_cm = int(floor_m * 100)

    # Pack: depth_cm (int16), floor_cm (int16), cycles (uint8), dive_angle (int8), rise_angle (int8)
    data = struct.pack('<hhBBBb',
                       depth_cm,
                       floor_cm,
                       cycles & 0xFF,
                       dive_angle & 0xFF,
                       rise_angle & 0xFF,
                       0  # padding byte to fill 8 bytes (ignored by firmware)
                       )
    send_can_message(0x102, data)
    print(f"Sent START_MISSION -> depth: {depth_m}m, floor: {floor_m}m, cycles: {cycles}, "
          f"dive_angle: {dive_angle}, rise_angle: {rise_angle}")


def stop_mission():
    """
    Send STOP under CAN ID 0x103 (no data payload).
    """
    send_can_message(0x103, bytes())
    print("Sent STOP_MISSION")


def emergency_surface():
    """
    Send EMERGENCY under CAN ID 0x104 (no data payload).
    """
    send_can_message(0x104, bytes())
    print("Sent EMERGENCY_SURFACE")


# -----------------------------------------------------------
# Telemetry Handling
# -----------------------------------------------------------
telemetry_lock = threading.Lock()

# Telemetry variables (updated by CAN listener)
telemetry = {
    'depth': "N/A",
    'pitch': "N/A",
    'roll': "N/A",
    'yaw': "N/A",
    'floor_dist': "N/A"
}


def process_can_message(msg):
    """
    Decode incoming CAN messages and update the telemetry dictionary.
    """
    global telemetry
    if msg.arbitration_id == 0x300 and len(msg.data) >= 2:
        # Depth: int16 cm -> meters
        depth_cm = struct.unpack('<h', msg.data[0:2])[0]
        depth_m = depth_cm / 100.0
        with telemetry_lock:
            telemetry['depth'] = f"{depth_m:.2f} m"

    elif msg.arbitration_id == 0x301 and len(msg.data) >= 6:
        # Orientation: pitch, roll, yaw as int16 each
        pitch = struct.unpack('<h', msg.data[0:2])[0]
        roll = struct.unpack('<h', msg.data[2:4])[0]
        yaw = struct.unpack('<h', msg.data[4:6])[0]
        with telemetry_lock:
            telemetry['pitch'] = f"{pitch}°"
            telemetry['roll'] = f"{roll}°"
            telemetry['yaw'] = f"{yaw}°"

    elif msg.arbitration_id == 0x302 and len(msg.data) >= 2:
        # Sonar floor distance: int16 cm -> meters
        floor_cm = struct.unpack('<h', msg.data[0:2])[0]
        floor_m = floor_cm / 100.0
        with telemetry_lock:
            telemetry['floor_dist'] = f"{floor_m:.2f} m"


def can_listener():
    """
    Continuously read from CAN and process messages.
    Runs in a background thread.
    """
    if not bus:
        return
    while True:
        try:
            msg = bus.recv(timeout=1.0)
            if msg:
                process_can_message(msg)
        except Exception:
            continue


# -----------------------------------------------------------
# Build the GUI
# -----------------------------------------------------------
root = tk.Tk()
root.title("Underwater Glider Control Panel")
root.configure(bg="#2F343F")  # dark background

# Use ttk styles for better visual appearance
style = ttk.Style(root)
style.theme_use("clam")
style.configure("TFrame", background="#2F343F")
style.configure("TLabel", background="#2F343F", foreground="#ECECEC", font=("Helvetica", 11))
style.configure("Header.TLabel", font=("Helvetica", 14, "bold"), foreground="#FFD966")
style.configure("TButton", font=("Helvetica", 11), padding=6)
style.configure("Danger.TButton", background="#D9534F", foreground="white", font=("Helvetica", 11, "bold"))

# Layout configuration: two columns
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

lbl_depth = ttk.Label(frame_telemetry, text="Depth: N/A")
lbl_depth.grid(row=1, column=0, padx=5, pady=2, sticky="w")
lbl_pitch = ttk.Label(frame_telemetry, text="Pitch: N/A")
lbl_pitch.grid(row=1, column=1, padx=5, pady=2, sticky="w")
lbl_roll = ttk.Label(frame_telemetry, text="Roll: N/A")
lbl_roll.grid(row=1, column=2, padx=5, pady=2, sticky="w")
lbl_yaw = ttk.Label(frame_telemetry, text="Yaw: N/A")
lbl_yaw.grid(row=1, column=3, padx=5, pady=2, sticky="w")

lbl_floor = ttk.Label(frame_telemetry, text="Floor Dist: N/A")
lbl_floor.grid(row=2, column=0, padx=5, pady=2, sticky="w")


# --- Control Frame (Pitch Only / VBD) ---
frame_control = ttk.Frame(root, relief="ridge")
frame_control.grid(row=1, column=0, sticky="nsew", padx=10, pady=5)
frame_control.columnconfigure(0, weight=1)
frame_control.columnconfigure(1, weight=1)

lbl_control_header = ttk.Label(frame_control, text="Manual Controls", style="Header.TLabel")
lbl_control_header.grid(row=0, column=0, columnspan=2, pady=(5, 10))

# Pitch
ttk.Label(frame_control, text="Pitch (°):").grid(row=1, column=0, sticky="e", padx=5, pady=2)
entry_pitch = ttk.Entry(frame_control, width=10)
entry_pitch.grid(row=1, column=1, padx=5, pady=2)

btn_set_pitch = ttk.Button(
    frame_control,
    text="Set Pitch",
    command=send_pitch
)
btn_set_pitch.grid(row=2, column=0, columnspan=2, pady=8)

# VBD State
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
# Periodic Telemetry Update in GUI
# -----------------------------------------------------------
def update_telemetry_labels():
    """
    Update the telemetry labels from the shared telemetry dictionary.
    Then schedule itself to run again after 200 ms.
    """
    with telemetry_lock:
        lbl_depth.config(text=f"Depth: {telemetry['depth']}")
        lbl_pitch.config(text=f"Pitch: {telemetry['pitch']}")
        lbl_roll.config(text=f"Roll: {telemetry['roll']}")
        lbl_yaw.config(text=f"Yaw: {telemetry['yaw']}")
        lbl_floor.config(text=f"Floor Dist: {telemetry['floor_dist']}")
    root.after(200, update_telemetry_labels)


# -----------------------------------------------------------
# Application Shutdown
# -----------------------------------------------------------
def on_closing():
    if messagebox.askokcancel("Quit", "Do you really want to quit?"):
        root.destroy()


root.protocol("WM_DELETE_WINDOW", on_closing)

# -----------------------------------------------------------
# Launch CAN Listener Thread and Start GUI Loop
# -----------------------------------------------------------
if bus:
    listener_thread = threading.Thread(target=can_listener, daemon=True)
    listener_thread.start()

# Kick off first telemetry update
root.after(200, update_telemetry_labels)
root.mainloop()
