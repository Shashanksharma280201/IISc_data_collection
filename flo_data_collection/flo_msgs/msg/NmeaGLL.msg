# NEMA-0183 GLL message
# Example : $GNGLL,1255.41029,N,07738.83881,E,130420.60,A,A*79
# Refer : https://receiverhelp.trimble.com/alloy-gnss/en-us/NMEA-0183messages_GLL.html

# constellation
string talker

# Latitude in dd mm,mmmm format (0-7 decimal places)
float64 latitude

# Direction of latitude N: North S: South
string latitude_direction

# Longitude in ddd mm,mmmm format (0-7 decimal places)
float64 longitude

# Direction of longitude E: East W: West
string longitude_direction

# UTC of position in hhmmss.ss format
float64 utc_timestamp

# Status indicator:
# A: Data valid
# V: Data not valid

# This value is set to V (Data not valid) for all Mode Indicator values except A (Autonomous) and D (Differential)
string status_indicator

# Mode indicator:
# A: Autonomous mode
# D: Differential mode
# E: Estimated (dead reckoning) mode
# M: Manual input mode
# S: Simulator mode
# N: Data not valid
string faa_mode

string checksum