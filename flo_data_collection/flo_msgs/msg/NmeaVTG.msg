# NEMA-0183 VTG message
# Track made good and speed over ground
# Example : $GNVTG,,T,,M,0.413,N,0.765,K,A*3F
# Refer : https://receiverhelp.trimble.com/alloy-gnss/en-us/NMEA-0183messages_VTG.html

# constellation
string talker

# track made good is relative to true north
float64 true_track # degrees
string true_track_sym

# track made good is relative to magnetic north
float64 magnetic_track # degrees
string magnetic_track_sym

# ground speed - knots
float64 ground_speed_knots
string ground_speed_knots_sym

# ground speed - kmph
float64 ground_speed_kmph
string ground_speed_kmph_sym

# Mode indicator:
# A: Autonomous mode
# D: Differential mode
# E: Estimated (dead reckoning) mode
# M: Manual input mode
# S: Simulator mode
# N: Data not valid
string faa_mode

string checksum



