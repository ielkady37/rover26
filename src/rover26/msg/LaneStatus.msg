float32 steering_angle
float32 steering_magnitude
int8    direction
bool    left_detected
bool    right_detected
uint8   confidence
float32 imu_yaw_rate
float32 imu_accel_x

# Lane polynomial coefficients [a, b, c] for x = a*y^2 + b*y + c
float32[3] left_coeffs
float32[3] right_coeffs