# Message For integration navigation, pose, velocity and so on.

############################## Header #############################################
Header header
###################################################################################

################################ GPS Week #########################################
uint32 gps_week_num
float64 gps_seconds
###################################################################################

################################### Position ######################################
# lla: latitude, longitude, altitude
# Latitude (degrees). Positive is north of equator; negative is south.
float64 latitude
# Longitude (degrees). Positive is east of prime meridian, negative west.
float64 longitude
# Altitude (meters). Positive is above reference (e.g., sea level).
float64 altitude
float64[9] position_covariance
###################################################################################

################################### Velocity ######################################
# Velocity, enu
float64 velocity_x
float64 velocity_y
float64 velocity_z
float64[9] velocity_covariance
###################################################################################

################################### Euler #########################################
# Heading in degrees [0,360), Pitch in degrees +- 90
float64 heading
float64 pitch
float64 roll
###################################################################################

################################### Quaternion ####################################
# quaternion
geometry_msgs/Quaternion quaternion
###################################################################################

################################### IMU Data ######################################
float32 gyro_x
float32 gyro_y
float32 gyro_z
float32 acc_x
float32 acc_y
float32 acc_z
###################################################################################

################################### Rover Status ##################################
uint16 status
uint8 satellites1
uint8 satellites2
float32 age
###################################################################################
