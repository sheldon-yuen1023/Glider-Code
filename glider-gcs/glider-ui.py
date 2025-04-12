import tkinter as tk
from tkinter import messagebox
import serial
import time

#-----------------------------------------------------------
# Establish Serial Communication with Flight Controller
#-----------------------------------------------------------
try:
    # Replace 'COM5' and 115200 with your ESP32 serial port and baud rate
    ser = serial.Serial('COM5', 115200, timeout=1)
    time.sleep(2)  # give some time for connection to settle
except Exception as e:
    messagebox.showerror("Serial Port Error", f"Could not open serial port:\n{e}")
    ser = None

#-----------------------------------------------------------
# Functions to Handle Mission Commands and Config Updates
#-----------------------------------------------------------
def start_mission():
    """Collect mission parameters and send a START command with parameters."""
    dive_depth = entry_dive_depth.get()
    rise_depth = entry_rise_depth.get()
    dive_angle = entry_dive_angle.get()
    rise_angle = entry_rise_angle.get()
    turn_radius = entry_turn_radius.get()
    cycles = entry_cycles.get()
    # Construct the START command string.
    command = f"START,{dive_depth},{rise_depth},{dive_angle},{rise_angle},{turn_radius},{cycles}\n"
    if ser:
        ser.write(command.encode('utf-8'))
        print("Sent:", command.strip())

def end_mission():
    """Send the END command."""
    command = "END\n"
    if ser:
        ser.write(command.encode('utf-8'))
        print("Sent:", command.strip())

def emergency_surface():
    """Send the EMERGENCY command to trigger an immediate surface."""
    command = "EMERGENCY\n"
    if ser:
        ser.write(command.encode('utf-8'))
        print("Sent:", command.strip())

def update_config(event=None):
    """
    Send mission parameters as a config update to the flight controller.
    This function is bound to changes (focus-out) on input fields.
    """
    dive_depth = entry_dive_depth.get()
    rise_depth = entry_rise_depth.get()
    dive_angle = entry_dive_angle.get()
    rise_angle = entry_rise_angle.get()
    turn_radius = entry_turn_radius.get()
    cycles = entry_cycles.get()
    command = f"CONFIG,{dive_depth},{rise_depth},{dive_angle},{rise_angle},{turn_radius},{cycles}\n"
    if ser:
        ser.write(command.encode('utf-8'))
        print("Sent config:", command.strip())

def update_telemetry():
    """
    Continuously poll the serial port for telemetry data and update the GUI.
    The flight controller is assumed to send a comma-separated message of the form:
    "TELEMETRY,<pitch>,<roll>,<yaw>,<heading>,<depth>,<sonar>,<btemp>,<bvoltage>,<bcurrent>,<leak>"
    """
    if ser and ser.in_waiting:
        try:
            line = ser.readline().decode('utf-8').strip()
            # Check if this is a telemetry message and parse it
            if line.startswith("TELEMETRY"):
                parts = line.split(',')
                if len(parts) == 11:
                    # Update telemetry displays accordingly:
                    pitch_label.config(text=f"Pitch: {parts[1]}")
                    roll_label.config(text=f"Roll: {parts[2]}")
                    yaw_label.config(text=f"Yaw: {parts[3]}")
                    heading_label.config(text=f"Heading: {parts[4]}")
                    depth_label.config(text=f"Depth: {parts[5]}")
                    sonar_label.config(text=f"Sonar Dist.: {parts[6]}")
                    battery_label.config(text=f"Battery: Temp {parts[7]}, Volt {parts[8]}, Curr {parts[9]}")
                    leak_label.config(text=f"Leak: {parts[10]}")
        except Exception as e:
            print("Telemetry error:", e)
    # Schedule the next telemetry update after 100 ms
    root.after(100, update_telemetry)

def quit_app():
    """Properly close the serial connection and exit the application."""
    if ser:
        ser.close()
    root.destroy()

#-----------------------------------------------------------
# Build the GUI
#-----------------------------------------------------------
root = tk.Tk()
root.title("Underwater Glider Control Interface")

# --- Telemetry Frame ---
frame_telemetry = tk.Frame(root, bd=2, relief=tk.SUNKEN)
frame_telemetry.grid(row=0, column=0, columnspan=2, padx=5, pady=5, sticky="nsew")
tk.Label(frame_telemetry, text="Telemetry", font=("Arial", 14, "bold")).grid(row=0, column=0, columnspan=2)

pitch_label = tk.Label(frame_telemetry, text="Pitch: N/A", width=25, anchor="w")
pitch_label.grid(row=1, column=0, padx=2, pady=2)
roll_label = tk.Label(frame_telemetry, text="Roll: N/A", width=25, anchor="w")
roll_label.grid(row=1, column=1, padx=2, pady=2)
yaw_label = tk.Label(frame_telemetry, text="Yaw: N/A", width=25, anchor="w")
yaw_label.grid(row=2, column=0, padx=2, pady=2)
heading_label = tk.Label(frame_telemetry, text="Heading: N/A", width=25, anchor="w")
heading_label.grid(row=2, column=1, padx=2, pady=2)
depth_label = tk.Label(frame_telemetry, text="Depth: N/A", width=25, anchor="w")
depth_label.grid(row=3, column=0, padx=2, pady=2)
sonar_label = tk.Label(frame_telemetry, text="Sonar Dist.: N/A", width=25, anchor="w")
sonar_label.grid(row=3, column=1, padx=2, pady=2)
battery_label = tk.Label(frame_telemetry, text="Battery: N/A", width=25, anchor="w")
battery_label.grid(row=4, column=0, padx=2, pady=2)
leak_label = tk.Label(frame_telemetry, text="Leak: N/A", width=25, anchor="w")
leak_label.grid(row=4, column=1, padx=2, pady=2)

# --- Mission Parameters Frame ---
frame_params = tk.Frame(root, bd=2, relief=tk.SUNKEN)
frame_params.grid(row=1, column=0, padx=5, pady=5, sticky="nsew")
tk.Label(frame_params, text="Mission Parameters", font=("Arial", 14, "bold")).grid(row=0, column=0, columnspan=2, pady=(0,5))

tk.Label(frame_params, text="Dive Depth:").grid(row=1, column=0, sticky="e", padx=5, pady=2)
entry_dive_depth = tk.Entry(frame_params, width=10)
entry_dive_depth.grid(row=1, column=1, padx=5, pady=2)

tk.Label(frame_params, text="Rise Depth:").grid(row=2, column=0, sticky="e", padx=5, pady=2)
entry_rise_depth = tk.Entry(frame_params, width=10)
entry_rise_depth.grid(row=2, column=1, padx=5, pady=2)

tk.Label(frame_params, text="Dive Angle:").grid(row=3, column=0, sticky="e", padx=5, pady=2)
entry_dive_angle = tk.Entry(frame_params, width=10)
entry_dive_angle.grid(row=3, column=1, padx=5, pady=2)

tk.Label(frame_params, text="Rise Angle:").grid(row=4, column=0, sticky="e", padx=5, pady=2)
entry_rise_angle = tk.Entry(frame_params, width=10)
entry_rise_angle.grid(row=4, column=1, padx=5, pady=2)

tk.Label(frame_params, text="Turn Radius / Roll Angle:").grid(row=5, column=0, sticky="e", padx=5, pady=2)
entry_turn_radius = tk.Entry(frame_params, width=10)
entry_turn_radius.grid(row=5, column=1, padx=5, pady=2)

tk.Label(frame_params, text="Number of Cycles:").grid(row=6, column=0, sticky="e", padx=5, pady=2)
entry_cycles = tk.Entry(frame_params, width=10)
entry_cycles.grid(row=6, column=1, padx=5, pady=2)

# Bind the update_config function to trigger when an input field loses focus.
for widget in [entry_dive_depth, entry_rise_depth, entry_dive_angle, entry_rise_angle, entry_turn_radius, entry_cycles]:
    widget.bind("<FocusOut>", update_config)

# --- Command Buttons Frame ---
frame_buttons = tk.Frame(root, bd=2, relief=tk.SUNKEN)
frame_buttons.grid(row=1, column=1, padx=5, pady=5, sticky="nsew")

start_btn = tk.Button(frame_buttons, text="Start Mission", command=start_mission, width=20)
start_btn.grid(row=0, column=0, padx=5, pady=5)
end_btn = tk.Button(frame_buttons, text="End Mission", command=end_mission, width=20)
end_btn.grid(row=1, column=0, padx=5, pady=5)
emergency_btn = tk.Button(frame_buttons, text="Emergency Surface", command=emergency_surface, width=20, bg="red", fg="white")
emergency_btn.grid(row=2, column=0, padx=5, pady=5)
quit_btn = tk.Button(frame_buttons, text="Quit", command=quit_app, width=20)
quit_btn.grid(row=3, column=0, padx=5, pady=5)

# Start the periodic telemetry update loop.
root.after(100, update_telemetry)

# Start the GUI event loop
root.mainloop()
