############################## Ros Custom Msg Header ##############################
# Message For integration navigation
Header header

NovatelMessageHeader novatel_msg_header

float64 gps_seconds1   # bestpos
float64 gps_seconds2   # bestxyz
float64 gps_seconds3   # heading
###################################################################################

############################### Position Msg(bestpos) #############################
# lla: latitude, longitude, altitude
# Latitude (degrees). Positive is north of equator; negative is south.
float64 latitude
# Longitude (degrees). Positive is east of prime meridian, negative west.
float64 longitude
# Altitude (meters). Positive is above reference (e.g., sea level).
float64 altitude
float32 undulation

# Position Standard Deviation (m)
float32 lat_sigma
float32 lon_sigma
float32 height_sigma

float64[9] position_covariance
###################################################################################

############################### Velocity Msg(bestxyz) #############################
# Velocity, ecef
float32 x_vel
float32 y_vel
float32 z_vel

float32 v_latency

# Velocity Standard Deviation (m/s)
float32 x_vel_sigma
float32 y_vel_sigma
float32 z_vel_sigma

float64[9] velocity_covariance
###################################################################################

################################### Heading Msg ###################################
# Heading in degrees [0,360), Pitch in degrees +- 90
float32 heading
float32 pitch

float32 heading_sigma
float32 pitch_sigma

float64 heading_covariance
float64 pitch_covariance
###################################################################################

################################### Rover Status ##################################
string solution_status   # bestpos
string position_type
string xyz_sol_status    # bestxyz
string xyz_type
string heading_type      # heading

uint8 num_satellites1
uint8 num_satellites2

string board_type
###################################################################################

################################### Radio Status ##################################
float32 diff_age
float32 solution_age
float32 baseline_length    # Baseline length (m)
###################################################################################

############################## Ros Custom Msg Tail ################################
NovatelExtendedSolutionStatus extended_solution_status
NovatelSignalMask signal_mask
###################################################################################
