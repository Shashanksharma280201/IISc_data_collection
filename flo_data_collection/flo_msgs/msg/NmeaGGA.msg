# NEMA-0183 GGA message
# Example : $GNGGA,130420.80,1255.41032,N,07738.83880,E,1,12,0.71,891.6,M,-86.5,M,,*6C
# Refer : https://receiverhelp.trimble.com/alloy-gnss/en-us/NMEA-0183messages_GGA.html

# constellation
string talker
float64 utc_timestamp
float64 latitude
string latitude_direction
float64 longitude
string longitude_direction

# GPS Quality indicator:
# 0: Fix not valid
# 1: GPS fix
# 2: Differential GPS fix (DGNSS), SBAS, OmniSTAR VBS, Beacon, RTX in GVBS mode
# 3: Not applicable
# 4: RTK Fixed, xFill
# 5: RTK Float, OmniSTAR XP/HP, Location RTK, RTX
# 6: INS Dead reckoning
uint16 quality

uint16 number_of_satellites
float64 hdop
float64 altitude
string altitude_units

float64 geoid_separation
string geoid_separation_units

# Age of differential GPS data record, Type 1 or Type 9. Null field when DGPS is not used.
uint64 age_dgps_data

# Reference station ID, range 0000 to 4095. 
# A null field when any reference station ID is selected and no corrections are received.
uint16 reference_station_id

string checksum