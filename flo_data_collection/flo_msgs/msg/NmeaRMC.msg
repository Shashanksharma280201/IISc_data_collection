# NEMA-0183 RMC message
# Position, velocity, and time
# Example : $GNRMC,130420.80,A,1255.41032,N,07738.83880,E,0.279,,040923,,,A,V*1F
# Refer : https://receiverhelp.trimble.com/alloy-gnss/en-us/NMEA-0183messages_RMC.html

# constellation
string talker

float64 utc_timestamp

# Status A=active or V=void
string status

float64 latitude
string latitude_direction
float64 longitude
string longitude_direction

float64 ground_speed_knots
float64 true_track # in degrees

string date # format in ddMMYY

float64 magnetic_variation # in degrees
string magnetic_variation_direction

# A=Autonomous, D=DGPS, E=DR
string mode

# TODO : add source
string navigational_status

string checksum



