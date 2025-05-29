# SIM7600 4g Modem by Qualcomm. 
# Ref : https://www.waveshare.com/w/upload/5/54/SIM7500_SIM7600_Series_AT_Command_Manual_V1.08.pdf
# AT+CPSI?
# +CPSI: <System Mode>,<Operation Mode>[,<MCC>-<MNC>,<
# TAC>,<SCellID>,<PCellID>,<Frequency Band>,<earfcn>,<dlb
# w>,<ulbw>,<RSRQ>,<RSRP>,<RSSI>,<RSSNR>]
# OK
# sample:
# +CPSI: LTE,Online,404-45,0x21F9,322642,453,EUTRAN-BAND40,39275,4,4,-168,-1126,-773,7

std_msgs/Header header
string system_mode
string operation_mode
string mcc # mobile country code
string mnc # mobile network code
string tac # tracing area code
uint32 scell_id # service cell id
uint32 pcell_id # physical cell id
string frequency_band
uint64 earfcn # E-UTRA absolute radio frequency channel number for searching LTE cells
int32 dlbw # downlink transmission bandwidth config
int32 ulbw # uplink transmission bandwidth config
int32 rsrq # Current Reference Signal Receive Quality as measured by L1.
int32 rsrp # Current Reference Signal Received Power in -1/10 dBm
int32 rssi # Received Signal Strength Indication 
int32 rssnr # Average Reference Signal Signal-to-Noise Ratio of the serving cel
