#!/usr/bin/env python3

import os
import math
from datetime import datetime
from threading import Lock

import rclpy
from rclpy.time import Time
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import pynmea2
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference
from geometry_msgs.msg import TwistStamped
from flo_msgs.msg import NmeaGGA, NmeaGLL, NmeaGSA, NmeaGSV, NmeaRMC, NmeaVTG
from flo_msgs.srv import StartLogging, StopLogging

from flo_utils.ros2_utils import spin


def safe_int(field, default=0):
    try:
        return int(field)
    except TypeError:
        return default
    except ValueError:
        return default


def safe_float(field, default="NaN"):
    try:
        return float(field)
    except TypeError:
        return float(default)
    except ValueError:
        return float(default)


def convert_knots_to_meter_per_secs(knots):
    return safe_float(knots) * 0.514444444444


class NmeaTopicDriver(Node):
    def __init__(self):
        super().__init__('nmea_topic_driver')
        self.declare_parameter('debug', True)
        self.declare_parameter('frame_id', 'gps')
        self.declare_parameter('time_ref_source', 'gps').value
        self.declare_parameter('useRMC', False).value
        self.declare_parameter('publish_fix', True)
        self.declare_parameter('publish_velocity', False)
        self.declare_parameter('publish_time_ref', True)
        self.declare_parameter('quality_fix', False) # publishes fix iff in quality 4
        self.frame_id = self.get_parameter(
            'frame_id').get_parameter_value().string_value
        self.time_ref_source = self.get_parameter(
            'time_ref_source').get_parameter_value().string_value
        self.use_rmc = self.get_parameter(
            'useRMC').get_parameter_value().bool_value
        self.publish_fix = self.get_parameter(
            'publish_fix').get_parameter_value().bool_value
        self.publish_velocity = self.get_parameter(
            'publish_velocity').get_parameter_value().bool_value
        self.publish_time_ref = self.get_parameter(
            'publish_time_ref').get_parameter_value().bool_value
        self.quality_fix = self.get_parameter(
            'quality_fix').get_parameter_value().bool_value

        self.create_subscription(
            String,
            "/nmea",
            self._nmea_cb,
            100,
            callback_group=ReentrantCallbackGroup()
        )

        if self.is_debug():
            self._create_nmea_msg_publishers()

        self.logging = False
        self.logging_gga_fd = None
        self.logging_gll_fd = None
        self.logging_gsa_fd = None
        self.logging_gsv_fd = None
        self.logging_rmc_fd = None
        self.logging_vtg_fd = None
        self.logging_lock = Lock()
        self.start_logging = self.create_service(
            StartLogging,
            'nmea_topic_driver/start_logging',
            self.start_logging_cb
        )
        self.stop_logging = self.create_service(
            StopLogging,
            'nmea_topic_driver/stop_logging',
            self.stop_logging_cb
        )

        if self.publish_fix:
            self._fix_pub = self.create_publisher(
                NavSatFix,
                "/fix",
                10
            )

        if self.publish_velocity:
            self._velocity_pub = self.create_publisher(
                TwistStamped,
                "/velocity",
                10
            )

        if self.publish_time_ref:
            self._time_ref_pub = self.create_publisher(
                TimeReference,
                "/time_reference",
                10
            )

        # synced using RMC sentences
        self._date = None

        # epe = estimated position error
        self.default_epe_quality0 = self.declare_parameter(
            'epe_quality0', 1000000).value
        self.default_epe_quality1 = self.declare_parameter(
            'epe_quality1', 4.0).value
        self.default_epe_quality2 = self.declare_parameter(
            'epe_quality2', 0.1).value
        self.default_epe_quality4 = self.declare_parameter(
            'epe_quality4', 0.02).value
        self.default_epe_quality5 = self.declare_parameter(
            'epe_quality5', 4.0).value
        self.default_epe_quality9 = self.declare_parameter(
            'epe_quality9', 3.0).value

        self.using_receiver_epe = False

        self.lon_std_dev = float("nan")
        self.lat_std_dev = float("nan")
        self.alt_std_dev = float("nan")

        self.gps_qualities = {
            # Unknown
            -1: [
                self.default_epe_quality0,
                NavSatStatus.STATUS_NO_FIX,
                NavSatFix.COVARIANCE_TYPE_UNKNOWN
            ],
            # Invalid
            0: [
                self.default_epe_quality0,
                NavSatStatus.STATUS_NO_FIX,
                NavSatFix.COVARIANCE_TYPE_UNKNOWN
            ],
            # SPS
            1: [
                self.default_epe_quality1,
                NavSatStatus.STATUS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # DGPS
            2: [
                self.default_epe_quality2,
                NavSatStatus.STATUS_SBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # RTK Fix
            4: [
                self.default_epe_quality4,
                NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # RTK Float
            5: [
                self.default_epe_quality5,
                NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # WAAS
            9: [
                self.default_epe_quality9,
                NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ]
        }

    def _create_nmea_msg_publishers(self):
        self._gga_pub = self.create_publisher(
            NmeaGGA,
            "/nmea/gga",
            10
        )

        self._gll_pub = self.create_publisher(
            NmeaGLL,
            "/nmea/gll",
            10
        )

        self._gsa_pub = self.create_publisher(
            NmeaGSA,
            "/nmea/gsa",
            10
        )

        self._gsv_pub = self.create_publisher(
            NmeaGSV,
            "/nmea/gsv",
            10
        )

        self._rmc_pub = self.create_publisher(
            NmeaRMC,
            "/nmea/rmc",
            10
        )

        self._vtg_pub = self.create_publisher(
            NmeaVTG,
            "/nmea/vtg",
            10
        )

    def is_debug(self):
        return self.get_parameter('debug').get_parameter_value().bool_value

    def talker_to_navsat_status(self, talker: str):
        talker = talker.rstrip()
        if talker == "GP":
            return NavSatStatus.SERVICE_GPS
        elif talker == "GN":
            return NavSatStatus.SERVICE_GLONASS
        elif talker == "GL":
            return NavSatStatus.SERVICE_GALILEO
        else:
            return NavSatStatus.SERVICE_COMPASS

    def _nmea_cb(self, msg: String):
        # 1. obtain NMEA-0183 sentence
        # 2. parse msg
        # 3. get type and publish if debug
        # 4. populate fix
        # 5. populate ground speed
        # 6. populate time ref
        try:
            self._parse_nmea_msg(msg)
        except Exception:
            return

    def start_logging_cb(self, req, res):
        if not self.logging:
            self.logging_gga_fd = open(
                os.path.join(req.path_to_data_dir, "GGA.csv"), "w"
            )
            self.logging_gga_fd.write(
                "timestamp,talker,utc_timestamp,latitude,latitude_direction,"+\
                "longitude,longitude_direction,quality,number_of_satellites,"+\
                "hdop,altitude,altitude_units,geoid_separation,geoid_separation_units,"+\
                "age_dgps_data,reference_station_id,checksum\n"
            )
            self.logging_gll_fd = open(
                os.path.join(req.path_to_data_dir, "GLL.csv"), "w"
            )
            self.logging_gll_fd.write(
                "timestamp,talker,latitude,latitude_direction,"+\
                "longitude,longitude_direction,utc_timestamp,"+\
                "status_indicator,faa_mode,checksum\n"
            )
            self.logging_gsa_fd = open(
                os.path.join(req.path_to_data_dir, "GSA.csv"), "w"
            )
            self.logging_gsa_fd.write(
                "timestamp,talker,mode,mode_fix_type,"+\
                "sv_id01,sv_id02,sv_id03,sv_id04,sv_id05,sv_id06,sv_id07,sv_id08,sv_id09,"+\
                "sv_id010,sv_id011,sv_id012,pdop,hdop,vdop,checksum\n"
            )
            self.logging_gsv_fd = open(
                os.path.join(req.path_to_data_dir, "GSV.csv"), "w"
            )
            self.logging_gsv_fd.write(
                "timestamp,talker,num_messages,msg_num,num_satellites_in_view,"+\
                "sv_prn_1,elevation_1,azimuth_1,snr_1,"+\
                "sv_prn_2,elevation_2,azimuth_2,snr_2,"+\
                "sv_prn_3,elevation_3,azimuth_3,snr_3,"+\
                "sv_prn_4,elevation_4,azimuth_4,snr_4,checksum\n"
            )
            self.logging_rmc_fd = open(
                os.path.join(req.path_to_data_dir, "RMC.csv"), "w"
            )
            self.logging_rmc_fd.write(
                "timestamp,talker,utc_timestamp,status,latitude,latitude_direction,"+\
                "longitude,longitude_direction,ground_speed_knots,true_track,"+\
                "date,magnetic_variation,magnetic_variation_direction,"+\
                "mode,navigational_status,checksum\n"
            )
            self.logging_vtg_fd = open(
                os.path.join(req.path_to_data_dir, "VTG.csv"), "w"
            )
            self.logging_vtg_fd.write(
                "timestamp,talker,true_track,true_track_sym,magnetic_track,magnetic_track_sym"+\
                "ground_speed_knots,ground_speed_knots_sym"+\
                "ground_speed_kmph,ground_speed_kmph_sym"+\
                "faa_mode,checksum\n"
            )
            self.logging = True
            res.success = True
        else:
            res.success = True
            res.message = "Already logging!"
        return res

    def stop_logging_cb(self, req, res):
        if self.logging:
            self.logging_lock.acquire()
            self.logging_gga_fd.close()
            self.logging_gll_fd.close()
            self.logging_gsa_fd.close()
            self.logging_gsv_fd.close()
            self.logging_rmc_fd.close()
            self.logging_vtg_fd.close()
            self.logging = False
            self.logging_lock.release()
            res.success = True
        else:
            res.success = True
            res.message = "Already not logging!"
        return res

    def _parse_nmea_msg(self, msg: String):
        nmea_sentence = msg.data.rstrip()
        parsed = pynmea2.parse(nmea_sentence)
        parsed_split = parsed.data
        talker = parsed.talker
        checksum = nmea_sentence.split("*")[-1]

        current_time = self.get_clock().now().to_msg()

        fix_msg = NavSatFix()
        fix_msg.header.stamp = current_time
        fix_msg.header.frame_id = self.frame_id

        current_time_ref = TimeReference()
        current_time_ref.header.stamp = current_time
        current_time_ref.header.frame_id = self.frame_id
        if self.time_ref_source:
            current_time_ref.source = self.time_ref_source
        else:
            current_time_ref.source = self.frame_id

        if parsed.sentence_type == 'GGA':
            gga_msg = NmeaGGA()
            gga_msg.talker = talker
            gga_msg.checksum = checksum
            gga_msg.utc_timestamp = safe_float(parsed_split[0])
            gga_msg.latitude = safe_float(parsed_split[1])
            gga_msg.latitude_direction = parsed_split[2]
            gga_msg.longitude = safe_float(parsed_split[3])
            gga_msg.longitude_direction = parsed_split[4]
            gga_msg.quality = safe_int(parsed_split[5])
            gga_msg.number_of_satellites = safe_int(parsed_split[6])
            gga_msg.hdop = safe_float(parsed_split[7])
            gga_msg.altitude = safe_float(parsed_split[8])
            gga_msg.altitude_units = parsed_split[9]
            gga_msg.geoid_separation = safe_float(parsed_split[10])
            gga_msg.geoid_separation_units = parsed_split[11]
            gga_msg.age_dgps_data = safe_int(parsed_split[12])
            gga_msg.reference_station_id = safe_int(parsed_split[13])

            self._gga_pub.publish(gga_msg)

            self.logging_lock.acquire()
            if self.logging:
                self.logging_gga_fd.write(
                    f"{self.get_clock().now().nanoseconds/1e9},"+\
                    f"{gga_msg.talker},"+\
                    f"{gga_msg.utc_timestamp},"+\
                    f"{gga_msg.latitude},"+\
                    f"{gga_msg.latitude_direction},"+\
                    f"{gga_msg.longitude},"+\
                    f"{gga_msg.longitude_direction},"+\
                    f"{gga_msg.quality},"+\
                    f"{gga_msg.number_of_satellites},"+\
                    f"{gga_msg.hdop},"+\
                    f"{gga_msg.altitude},"+\
                    f"{gga_msg.altitude_units},"+\
                    f"{gga_msg.geoid_separation},"+\
                    f"{gga_msg.geoid_separation_units},"+\
                    f"{gga_msg.age_dgps_data},"+\
                    f"{gga_msg.reference_station_id},"+\
                    f"{gga_msg.checksum}\n"
                )
            self.logging_lock.release()

            fix_type = gga_msg.quality
            gps_qual = self.gps_qualities[fix_type]
            default_epe = gps_qual[0]
            fix_msg.status.status = gps_qual[1]
            fix_msg.status.service = self.talker_to_navsat_status(
                gga_msg.talker)
            fix_msg.position_covariance_type = gps_qual[2]

            lat = parsed_split[1]
            lat = float(lat[0:2]) + (float(lat[2:]) / 60.0)
            parsed_latitude = lat * (1 if parsed_split[2] == 'N' else -1)

            lon = parsed_split[3]
            lon_split = lon.split(".")
            lon_degrees = lon_split[0][:-2]
            lon_seconds = f"{lon_split[0][-2:]}.{lon_split[1]}"
            lon = float(lon_degrees) + (float(lon_seconds) / 60.0)
            parsed_longitude = lon * (1 if parsed_split[4] == 'E' else -1)

            fix_msg.latitude = round(parsed_latitude, 7)
            fix_msg.longitude = round(parsed_longitude, 7)

            # Altitude is above ellipsoid, so adjust for mean-sea-level
            altitude = gga_msg.altitude + gga_msg.geoid_separation
            fix_msg.altitude = altitude

            # use default epe std_dev unless we've received a GST sentence with epes
            if not self.using_receiver_epe or math.isnan(self.lon_std_dev):
                self.lon_std_dev = default_epe
            if not self.using_receiver_epe or math.isnan(self.lat_std_dev):
                self.lat_std_dev = default_epe
            if not self.using_receiver_epe or math.isnan(self.alt_std_dev):
                self.alt_std_dev = default_epe * 2

            hdop = gga_msg.hdop
            fix_msg.position_covariance[0] = (hdop * self.lon_std_dev) ** 2
            fix_msg.position_covariance[4] = (hdop * self.lat_std_dev) ** 2
            fix_msg.position_covariance[8] = \
                (2 * hdop * self.alt_std_dev) ** 2  # FIXME ??

            if self.publish_fix and not self.use_rmc:
                if self.quality_fix:
                    if gga_msg.quality == 4:
                        self._fix_pub.publish(fix_msg)
                else:
                    self._fix_pub.publish(fix_msg)


            if not self.publish_time_ref:
                return

            stamp = datetime.combine(datetime.now(), parsed.timestamp)
            epoch_time = stamp.timestamp()

            seconds = int(epoch_time)
            nseconds = int((epoch_time - seconds) * 1e9)

            if not math.isnan(gga_msg.utc_timestamp):
                current_time_ref.time_ref = Time(
                    seconds=seconds,
                    nanoseconds=nseconds
                ).to_msg()
                self._time_ref_pub.publish(current_time_ref)

        elif parsed.sentence_type == 'GLL':
            gll_msg = NmeaGLL()
            gll_msg.talker = talker
            gll_msg.checksum = checksum
            gll_msg.latitude = safe_float(parsed_split[0])
            gll_msg.latitude_direction = parsed_split[1]
            gll_msg.longitude = safe_float(parsed_split[2])
            gll_msg.longitude_direction = parsed_split[3]
            gll_msg.utc_timestamp = safe_float(parsed_split[4])
            gll_msg.status_indicator = parsed_split[5]
            gll_msg.faa_mode = parsed_split[6]
            if self.is_debug():
                self._gll_pub.publish(gll_msg)

            self.logging_lock.acquire()
            if self.logging:
                self.logging_gll_fd.write(
                    f"{self.get_clock().now().nanoseconds/1e9},"+\
                    f"{gll_msg.talker},"+\
                    f"{gll_msg.latitude},"+\
                    f"{gll_msg.latitude_direction},"+\
                    f"{gll_msg.longitude},"+\
                    f"{gll_msg.longitude_direction},"+\
                    f"{gll_msg.utc_timestamp},"+\
                    f"{gll_msg.status_indicator},"+\
                    f"{gll_msg.faa_mode},"+\
                    f"{gll_msg.checksum}\n"
                )
            self.logging_lock.release()

        elif parsed.sentence_type == 'GSA':
            gsa_msg = NmeaGSA()
            gsa_msg.talker = talker
            gsa_msg.mode = parsed.mode
            fix_type = int(parsed.mode_fix_type)
            if fix_type == NmeaGSA.FIX_2D:
                gsa_msg.mode_fix_type = NmeaGSA.FIX_2D
            elif fix_type == NmeaGSA.FIX_3D:
                gsa_msg.mode_fix_type = NmeaGSA.FIX_3D
            else:
                gsa_msg.mode_fix_type = NmeaGSA.NOT_AVAILABLE
            gsa_msg.sv_id01 = safe_int(parsed.sv_id01)
            gsa_msg.sv_id02 = safe_int(parsed.sv_id02)
            gsa_msg.sv_id03 = safe_int(parsed.sv_id03)
            gsa_msg.sv_id04 = safe_int(parsed.sv_id04)
            gsa_msg.sv_id05 = safe_int(parsed.sv_id05)
            gsa_msg.sv_id06 = safe_int(parsed.sv_id06)
            gsa_msg.sv_id07 = safe_int(parsed.sv_id07)
            gsa_msg.sv_id08 = safe_int(parsed.sv_id08)
            gsa_msg.sv_id09 = safe_int(parsed.sv_id09)
            gsa_msg.sv_id10 = safe_int(parsed.sv_id10)
            gsa_msg.sv_id11 = safe_int(parsed.sv_id11)
            gsa_msg.sv_id12 = safe_int(parsed.sv_id12)

            gsa_msg.pdop = safe_float(parsed.pdop)
            gsa_msg.hdop = safe_float(parsed.hdop)
            gsa_msg.vdop = safe_float(parsed.vdop)

            if self.is_debug():
                self._gsa_pub.publish(gsa_msg)

            self.logging_lock.acquire()
            if self.logging:
                self.logging_gsa_fd.write(
                    f"{self.get_clock().now().nanoseconds/1e9},"+\
                    f"{gsa_msg.talker},"+\
                    f"{gsa_msg.mode},"+\
                    f"{gsa_msg.mode_fix_type},"+\
                    f"{gsa_msg.sv_id01},"+\
                    f"{gsa_msg.sv_id02},"+\
                    f"{gsa_msg.sv_id03},"+\
                    f"{gsa_msg.sv_id04},"+\
                    f"{gsa_msg.sv_id05},"+\
                    f"{gsa_msg.sv_id06},"+\
                    f"{gsa_msg.sv_id07},"+\
                    f"{gsa_msg.sv_id08},"+\
                    f"{gsa_msg.sv_id09},"+\
                    f"{gsa_msg.sv_id10},"+\
                    f"{gsa_msg.sv_id11},"+\
                    f"{gsa_msg.sv_id12},"+\
                    f"{gsa_msg.pdop},"+\
                    f"{gsa_msg.hdop},"+\
                    f"{gsa_msg.vdop},"+\
                    f"{gsa_msg.checksum}\n"
                )
            self.logging_lock.release()

        elif parsed.sentence_type == 'GSV':
            gsv_msg = NmeaGSV()
            gsv_msg.talker = talker
            gsv_msg.checksum = checksum
            gsv_msg.num_messages = safe_int(parsed.num_messages)
            gsv_msg.msg_num = safe_int(parsed.msg_num)
            gsv_msg.num_satellites_in_view = safe_int(parsed.num_sv_in_view)
            gsv_msg.sv_prn_1 = safe_int(parsed.sv_prn_num_1)
            gsv_msg.elevation_1 = safe_int(parsed.elevation_deg_1)
            gsv_msg.azimuth_1 = safe_int(parsed.azimuth_1)
            gsv_msg.snr_1 = safe_int(parsed.snr_1)

            gsv_msg.sv_prn_2 = safe_int(parsed.sv_prn_num_2)
            gsv_msg.elevation_2 = safe_int(parsed.elevation_deg_2)
            gsv_msg.azimuth_2 = safe_int(parsed.azimuth_2)
            gsv_msg.snr_2 = safe_int(parsed.snr_2)

            gsv_msg.sv_prn_3 = safe_int(parsed.sv_prn_num_3)
            gsv_msg.elevation_3 = safe_int(parsed.elevation_deg_3)
            gsv_msg.azimuth_3 = safe_int(parsed.azimuth_3)
            gsv_msg.snr_3 = safe_int(parsed.snr_3)

            gsv_msg.sv_prn_4 = safe_int(parsed.sv_prn_num_4)
            gsv_msg.elevation_4 = safe_int(parsed.elevation_deg_4)
            gsv_msg.azimuth_4 = safe_int(parsed.azimuth_4)
            gsv_msg.snr_4 = safe_int(parsed.snr_4)

            if self.is_debug():
                self._gsv_pub.publish(gsv_msg)

            self.logging_lock.acquire()
            if self.logging:
                self.logging_gsv_fd.write(
                    f"{self.get_clock().now().nanoseconds/1e9},"+\
                    f"{gsv_msg.talker},"+\
                    f"{gsv_msg.num_messages},"+\
                    f"{gsv_msg.msg_num},"+\
                    f"{gsv_msg.num_satellites_in_view},"+\
                    f"{gsv_msg.sv_prn_1},"+\
                    f"{gsv_msg.elevation_1},"+\
                    f"{gsv_msg.azimuth_1},"+\
                    f"{gsv_msg.snr_1},"+\
                    f"{gsv_msg.sv_prn_2},"+\
                    f"{gsv_msg.elevation_2},"+\
                    f"{gsv_msg.azimuth_2},"+\
                    f"{gsv_msg.snr_2},"+\
                    f"{gsv_msg.sv_prn_3},"+\
                    f"{gsv_msg.elevation_3},"+\
                    f"{gsv_msg.azimuth_3},"+\
                    f"{gsv_msg.snr_3},"+\
                    f"{gsv_msg.sv_prn_4},"+\
                    f"{gsv_msg.elevation_4},"+\
                    f"{gsv_msg.azimuth_4},"+\
                    f"{gsv_msg.snr_4},"+\
                    f"{gsv_msg.checksum}\n"
                )
            self.logging_lock.release()

        elif parsed.sentence_type == 'RMC':
            rmc_msg = NmeaRMC()
            rmc_msg.talker = talker
            rmc_msg.checksum = checksum
            rmc_msg.utc_timestamp = safe_float(parsed_split[0])
            rmc_msg.status = parsed.status
            rmc_msg.latitude = safe_float(parsed.lat)
            rmc_msg.latitude_direction = parsed_split[3]
            rmc_msg.longitude = safe_float(parsed.lon)
            rmc_msg.longitude_direction = parsed_split[5]
            rmc_msg.ground_speed_knots = safe_float(parsed.spd_over_grnd)
            rmc_msg.true_track = safe_float(parsed.true_course)
            rmc_msg.date = parsed_split[8]
            rmc_msg.magnetic_variation = safe_float(parsed.mag_variation)
            rmc_msg.magnetic_variation_direction = parsed.mag_var_dir
            rmc_msg.mode = parsed.mode_indicator
            rmc_msg.navigational_status = parsed.nav_status

            if self.is_debug():
                self._rmc_pub.publish(rmc_msg)

            self.logging_lock.acquire()
            if self.logging:
                self.logging_rmc_fd.write(
                    f"{self.get_clock().now().nanoseconds/1e9},"+\
                    f"{rmc_msg.talker},"+\
                    f"{rmc_msg.utc_timestamp},"+\
                    f"{rmc_msg.status},"+\
                    f"{rmc_msg.latitude},"+\
                    f"{rmc_msg.latitude_direction},"+\
                    f"{rmc_msg.longitude},"+\
                    f"{rmc_msg.longitude_direction},"+\
                    f"{rmc_msg.ground_speed_knots},"+\
                    f"{rmc_msg.true_track},"+\
                    f"{rmc_msg.date},"+\
                    f"{rmc_msg.magnetic_variation},"+\
                    f"{rmc_msg.magnetic_variation_direction},"+\
                    f"{rmc_msg.mode},"+\
                    f"{rmc_msg.navigational_status},"+\
                    f"{rmc_msg.checksum}\n"
                )
            self.logging_lock.release()

            if rmc_msg.status == 'A':
                fix_msg.status.status = NavSatStatus.STATUS_FIX
            else:
                fix_msg.status.status = NavSatStatus.STATUS_NO_FIX
            fix_msg.status.service = self.talker_to_navsat_status(talker)
            fix_msg.latitude = parsed.latitude
            fix_msg.longitude = parsed.longitude
            fix_msg.altitude = float('NaN')
            fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

            if self.publish_fix and self.use_rmc:
                self._fix_pub.publish(fix_msg)

            stamp = datetime.combine(parsed.datestamp, parsed.timestamp)
            epoch_time = stamp.timestamp()

            seconds = int(epoch_time)
            nseconds = int((epoch_time - seconds) * 1e9)

            if self.publish_time_ref:
                current_time_ref.time_ref = Time(
                    seconds=seconds,
                    nanoseconds=nseconds
                ).to_msg()
                self._time_ref_pub.publish(current_time_ref)

            current_time = self.get_clock().now().to_msg()
            speed_in_meter_per_sec = convert_knots_to_meter_per_secs(
                rmc_msg.ground_speed_knots)

            current_vel = TwistStamped()
            current_vel.header.stamp = current_time
            current_vel.header.frame_id = self.frame_id
            current_vel.twist.linear.x = speed_in_meter_per_sec * \
                math.sin(math.radians(rmc_msg.true_track))
            current_vel.twist.linear.y = speed_in_meter_per_sec * \
                math.cos(math.radians(rmc_msg.true_track))
            current_vel.twist.linear

            if self.publish_velocity:
                self._velocity_pub.publish(current_vel)

        elif parsed.sentence_type == 'VTG':
            vtg_msg = NmeaVTG()
            vtg_msg.talker = talker
            vtg_msg.checksum = checksum
            vtg_msg.true_track = safe_float(parsed.true_track)
            vtg_msg.true_track_sym = parsed.true_track_sym
            vtg_msg.magnetic_track = safe_float(parsed.mag_track)
            vtg_msg.magnetic_track_sym = parsed.mag_track_sym
            vtg_msg.ground_speed_knots = safe_float(parsed.spd_over_grnd_kts)
            vtg_msg.ground_speed_knots_sym = parsed.spd_over_grnd_kts_sym
            vtg_msg.ground_speed_kmph = safe_float(parsed.spd_over_grnd_kmph)
            vtg_msg.ground_speed_kmph_sym = parsed.spd_over_grnd_kmph_sym
            vtg_msg.faa_mode = parsed.faa_mode

            if self.is_debug():
                self._vtg_pub.publish(vtg_msg)

            self.logging_lock.acquire()
            if self.logging:
                self.logging_vtg_fd.write(
                    f"{self.get_clock().now().nanoseconds/1e9},"+\
                    f"{vtg_msg.talker},"+\
                    f"{vtg_msg.true_track},"+\
                    f"{vtg_msg.true_track_sym},"+\
                    f"{vtg_msg.magnetic_track},"+\
                    f"{vtg_msg.magnetic_track_sym},"+\
                    f"{vtg_msg.ground_speed_knots},"+\
                    f"{vtg_msg.ground_speed_knots_sym},"+\
                    f"{vtg_msg.ground_speed_kmph},"+\
                    f"{vtg_msg.ground_speed_kmph_sym},"+\
                    f"{vtg_msg.faa_mode},"+\
                    f"{vtg_msg.checksum}\n"
                )
            self.logging_lock.release()

            if not self.publish_velocity:
                return

            current_time = self.get_clock().now().to_msg()
            speed_in_meter_per_sec = convert_knots_to_meter_per_secs(
                vtg_msg.ground_speed_knots)

            current_vel = TwistStamped()
            current_vel.header.stamp = current_time
            current_vel.header.frame_id = self.frame_id
            current_vel.twist.linear.x = speed_in_meter_per_sec * \
                math.sin(math.radians(vtg_msg.true_track))
            current_vel.twist.linear.y = speed_in_meter_per_sec * \
                math.cos(math.radians(vtg_msg.true_track))
            current_vel.twist.linear

            self._velocity_pub.publish(current_vel)


def main(args=None):
    rclpy.init(args=args)
    nmea_topic_driver = NmeaTopicDriver()
    multi_thread_executor = MultiThreadedExecutor()
    multi_thread_executor.add_node(nmea_topic_driver)
    try:
        spin(nmea_topic_driver, executor=multi_thread_executor, hz=400)
    except KeyboardInterrupt:
        print("Killing nmea topic driver ...")
        nmea_topic_driver.stop_logging_cb(StopLogging.Request(), StopLogging.Response())
        nmea_topic_driver.destroy_node()


if __name__ == "__main__":
    main()
