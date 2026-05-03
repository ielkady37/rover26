std_msgs/Header header

# Motor 1
uint8 m1_dir      # 0=forward  1=reverse
uint8 m1_brake    # 0=off      1=on
float32 m1_speed  # 0.0–1.0 duty cycle

# Motor 2
uint8 m2_dir      # 0=forward  1=reverse
uint8 m2_brake    # 0=off      1=on
float32 m2_speed  # 0.0–1.0 duty cycle

# Laser
uint8 laser       # 0=off  1=on

# Servo
float32 servo     # degrees
