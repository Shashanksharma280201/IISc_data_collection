# NEMA-0183 GSA message
# Example : $GNGSA,A,3,03,09,08,21,27,16,26,02,,,,,1.23,0.71,1.00,1*03
# Refer : https://receiverhelp.trimble.com/alloy-gnss/en-us/NMEA-0183messages_GSA.html

# constellation
string talker

# Mode 1:
# M = Manual
# A = Automatic
string mode

# Mode 2: Fix type:
uint8 NOT_AVAILABLE=1
uint8 FIX_2D=2
uint8 FIX_3D=3
uint8 mode_fix_type

# PRN Number
# 01 to 32 for GPS
# 33 to 64 for SBAS
# 64+ for GLONASS
# all the satellites used for estimating dop
uint16 sv_id01
uint16 sv_id02
uint16 sv_id03
uint16 sv_id04
uint16 sv_id05
uint16 sv_id06
uint16 sv_id07
uint16 sv_id08
uint16 sv_id09
uint16 sv_id10
uint16 sv_id11
uint16 sv_id12

# Dilution of precision
# 0.5 to 99.9
float64 pdop # position
float64 hdop # horizontal
float64 vdop # vertical

string checksum