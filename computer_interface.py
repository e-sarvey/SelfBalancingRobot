# Updated to dynamically adjust y-axis limits for all plots based on data in the buffer
import dearpygui.dearpygui as dpg
import numpy as np
import time
import paho.mqtt.client as mqtt
import json
import threading

# ---------------------------
# Configurable Variables
# ---------------------------

WINDOW_WIDTH = 800
WINDOW_HEIGHT = 600
PLOT_WIDTH = 350
PLOT_HEIGHT = 200
SLIDER_WIDTH = 200
BUFFER_SIZE = 1000  # Size of the data buffer

# MQTT Configurations
MQTT_BROKER = "10.243.82.33"
DATA_TOPIC = "ESP32/data"
GAINS_TOPIC = "ESP32/gains"

# Data buffers for angle, gyro, PWM, and corresponding time buffer
data_buffer = {
    'anglex': np.zeros(BUFFER_SIZE),  # Buffer for anglex
    'gyroX': np.zeros(BUFFER_SIZE),   # Buffer for GyroX
    'pwm': np.zeros(BUFFER_SIZE)      # Buffer for PWM
}
time_buffer = np.zeros(BUFFER_SIZE)  # Buffer to store time data for x-axis
buffer_index = 0  # Index to manage the buffer

# Gains variables and timers
balancing_gains = {"Kp": 0.0, "Kd": 0.0}
last_change_time = time.time()  # Initialize the last change time
gains_pending = False  # Flag to track if a change occurred and needs to be published
start_time = time.time()

# MQTT Setup
client = mqtt.Client()

# ---------------------------
# Helper Functions
# ---------------------------

# MQTT callback to process incoming messages from ESP32/data
def on_message(client, userdata, message):
    global buffer_index
    try:
        # Parse the incoming JSON data
        data = json.loads(message.payload.decode())
        print(f"Received Data: {data}")  # Debugging print to verify incoming data

        # Update data buffers
        time_buffer[buffer_index] = time.time() - start_time
        data_buffer['anglex'][buffer_index] = data.get('anglex', 0)
        data_buffer['gyroX'][buffer_index] = data.get('gyroX', 0)
        data_buffer['pwm'][buffer_index] = data.get('PWM', 0)

        # Increment buffer index and wrap around if it exceeds buffer size
        buffer_index = (buffer_index + 1) % BUFFER_SIZE

    except json.JSONDecodeError as e:
        print(f"Error decoding JSON: {e}")

# Function to handle publishing the new gains if there are no changes for 2 seconds
def publish_gains_if_stable():
    global last_change_time, gains_pending
    while True:
        if gains_pending and (time.time() - last_change_time >= 2):
            # Format the JSON payload with gains rounded to 3 decimal places
            payload = json.dumps({
                "balancing": f"[{balancing_gains['Kp']:.3f},{balancing_gains['Kd']:.3f}]"
            })
            # Publish the gains to the ESP32/gains topic
            client.publish(GAINS_TOPIC, payload)
            print(f"Published Gains: {payload}")
            gains_pending = False  # Reset the pending flag after publishing
        time.sleep(0.1)

# Start a background thread for publishing gains
threading.Thread(target=publish_gains_if_stable, daemon=True).start()

# Callback functions for the sliders and number inputs
def update_kp(sender, app_data):
    global last_change_time, gains_pending
    balancing_gains["Kp"] = app_data
    dpg.set_value("kp_label", f"Kp: {app_data:.3f}")
    dpg.set_value("kp_slider", app_data)  # Update the slider to reflect the number input
    last_change_time = time.time()
    gains_pending = True

def update_kp_slider(sender, app_data):
    update_kp(sender, app_data)  # Use the same update logic for sliders

def update_kd(sender, app_data):
    global last_change_time, gains_pending
    balancing_gains["Kd"] = app_data
    dpg.set_value("kd_label", f"Kd: {app_data:.3f}")
    dpg.set_value("kd_slider", app_data)
    last_change_time = time.time()
    gains_pending = True

def update_kd_slider(sender, app_data):
    update_kd(sender, app_data)

# ---------------------------
# MQTT Setup and Start
# ---------------------------

client.on_message = on_message  # Set the callback function for message reception
client.connect(MQTT_BROKER)
client.subscribe(DATA_TOPIC)  # Subscribe to the ESP32/data topic
client.loop_start()  # Start the MQTT loop in a separate thread

# ---------------------------
# GUI Setup
# ---------------------------

dpg.create_context()
dpg.create_viewport(title='Robot Data Viewer', width=WINDOW_WIDTH, height=WINDOW_HEIGHT)

# Create a theme for each plot line (Angle, GyroX, PWM)
with dpg.theme(tag="angle_theme"):
    with dpg.theme_component(dpg.mvLineSeries):
        dpg.add_theme_color(dpg.mvPlotCol_Line, (255, 0, 0), category=dpg.mvThemeCat_Plots)

with dpg.theme(tag="gyro_x_theme"):
    with dpg.theme_component(dpg.mvLineSeries):
        dpg.add_theme_color(dpg.mvPlotCol_Line, (0, 255, 0), category=dpg.mvThemeCat_Plots)

with dpg.theme(tag="pwm_theme"):
    with dpg.theme_component(dpg.mvLineSeries):
        dpg.add_theme_color(dpg.mvPlotCol_Line, (0, 0, 255), category=dpg.mvThemeCat_Plots)

# Main GUI layout
with dpg.window(label="Robot Data Viewer", width=WINDOW_WIDTH, height=WINDOW_HEIGHT):
    # First Row: Angle Plot (left) and PWM Plot (right)
    with dpg.group(horizontal=True):
        # Angle Plot (Top Left)
        with dpg.plot(label="Angle Plot", height=PLOT_HEIGHT, width=PLOT_WIDTH):
            dpg.add_plot_axis(dpg.mvXAxis, label="Time (s)", tag="xaxis_angle")
            y_axis = dpg.add_plot_axis(dpg.mvYAxis, label="Angle", tag="yaxis_angle")
            angle_series = dpg.add_line_series(time_buffer, data_buffer['anglex'], label="Angle Data", parent=y_axis)
            dpg.bind_item_theme(angle_series, "angle_theme")
        
        # PWM Plot (Top Right)
        with dpg.plot(label="PWM Plot", height=PLOT_HEIGHT, width=PLOT_WIDTH):
            dpg.add_plot_axis(dpg.mvXAxis, label="Time (s)", tag="xaxis_pwm")
            y_axis = dpg.add_plot_axis(dpg.mvYAxis, label="PWM", tag="yaxis_pwm")
            pwm_series = dpg.add_line_series(time_buffer, data_buffer['pwm'], label="PWM Data", parent=y_axis)
            dpg.bind_item_theme(pwm_series, "pwm_theme")

    # Second Row: Angular Velocity Plot (left) and Gains sliders (right)
    with dpg.group(horizontal=True):
        # Angular Velocity Plot (Bottom Left)
        with dpg.plot(label="Angular Velocity Plot", height=PLOT_HEIGHT, width=PLOT_WIDTH):
            x_axis = dpg.add_plot_axis(dpg.mvXAxis, label="Time (s)", tag="xaxis_gyro")
            y_axis = dpg.add_plot_axis(dpg.mvYAxis, label="GyroX", tag="yaxis_gyro")
            
            gyro_x_series = dpg.add_line_series(time_buffer, data_buffer['gyroX'], label="GyroX", parent=y_axis)
            dpg.bind_item_theme(gyro_x_series, "gyro_x_theme")

        # Gains Sliders (Bottom Right)
        with dpg.group(horizontal=False):
            dpg.add_text("Kp: 0.00", tag="kp_label")
            dpg.add_input_float(label="Kp Input", default_value=0, min_value=0, max_value=10, width=SLIDER_WIDTH, callback=update_kp)
            dpg.add_slider_float(label="Kp", tag="kp_slider", default_value=0, min_value=0, max_value=10, width=SLIDER_WIDTH, callback=update_kp_slider)
            
            dpg.add_text("Kd: 0.00", tag="kd_label")
            dpg.add_input_float(label="Kd Input", default_value=0, min_value=0, max_value=10, width=SLIDER_WIDTH, callback=update_kd)
            dpg.add_slider_float(label="Kd", tag="kd_slider", default_value=0, min_value=0, max_value=10, width=SLIDER_WIDTH, callback=update_kd_slider)

# ---------------------------
# Main Loop
# ---------------------------

dpg.setup_dearpygui()
dpg.show_viewport()

# Update the plots in a loop
try:
    while dpg.is_dearpygui_running():
        # Use only the last 10 seconds of data (e.g., current_time - 10 to current_time)
        current_time = time.time() - start_time
        mask = time_buffer >= max(0, current_time - 10)

        # Dynamically adjust y-axis limits based on data in the buffer
        dpg.set_axis_limits("yaxis_angle", np.min(data_buffer['anglex'][mask]), np.max(data_buffer['anglex'][mask]))  # For Angle Plot
        dpg.set_axis_limits("yaxis_gyro", np.min(data_buffer['gyroX'][mask]), np.max(data_buffer['gyroX'][mask]))  # For GyroX Plot
        dpg.set_axis_limits("yaxis_pwm", np.min(data_buffer['pwm'][mask]), np.max(data_buffer['pwm'][mask]))  # For PWM Plot

        # Update plots with new data
        dpg.set_value(angle_series, [time_buffer[mask], data_buffer['anglex'][mask]])
        dpg.set_value(gyro_x_series, [time_buffer[mask], data_buffer['gyroX'][mask]])
        dpg.set_value(pwm_series, [time_buffer[mask], data_buffer['pwm'][mask]])

        # Update the x-axis limits for all plots to reflect the moving time window
        dpg.set_axis_limits("xaxis_angle", max(0, current_time - 10), current_time)
        dpg.set_axis_limits("xaxis_gyro", max(0, current_time - 10), current_time)
        dpg.set_axis_limits("xaxis_pwm", max(0, current_time - 10), current_time)

        dpg.render_dearpygui_frame()  # Render the GUI frame
        time.sleep(0.01)  # Add a small delay to limit updates and improve performance
except KeyboardInterrupt:
    print("Window Closing")
    print("---------------------")
# Clean up Dear PyGui context
dpg.destroy_context()

# Disconnect MQTT client
client.disconnect()
