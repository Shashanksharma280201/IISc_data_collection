# NEMA-0183 GSV message
# Example : $GBGSV,2,1,05,08,21,172,21,09,39,020,12,10,56,130,17,13,30,179,20,3*76
# Refer : https://receiverhelp.trimble.com/alloy-gnss/en-us/NMEA-0183messages_GSV.html

# constellation
string talker

# Total number of messages of this type in this cycle
uint32 num_messages

# Message number
uint32 msg_num

# number of visible satellites
uint32 num_satellites_in_view

# SV - Visible Satellite
# Visible Satellite - 1
# SV PRN number
int32 sv_prn_1

# Elevation, in degrees, 90° maximum
int32 elevation_1

# Azimuth, degrees from True North, 000° through 359°
int32 azimuth_1

# SNR, 00 through 99 dB (null when not tracking)
# -1 is considered null here
int32 snr_1

# satellite - 2
int32 sv_prn_2
int32 elevation_2
int32 azimuth_2
int32 snr_2

# satellite - 3
int32 sv_prn_3
int32 elevation_3
int32 azimuth_3
int32 snr_3

# satellite - 4
int32 sv_prn_4
int32 elevation_4
int32 azimuth_4
int32 snr_4

string checksum