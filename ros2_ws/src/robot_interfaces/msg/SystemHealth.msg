# Overall system health status
bool arduino_connected
bool lidar_active
bool camera_usb_active
bool camera_picam1_active
bool camera_picam2_active
string active_camera        # "usb", "picam1", or "picam2"
bool movement_enabled       # Safety interlock
float32 battery_voltage     # Optional battery monitoring
