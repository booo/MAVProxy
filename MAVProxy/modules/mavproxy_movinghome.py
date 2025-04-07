#!/usr/bin/env python
'''
movinghome module
André Kjellstrup, Norce

This module can update the home position of the ArduPilot vehicle to the position of a moving GCS.
requires package python-nmea2

'''

import os
import os.path
import sys
from pymavlink import mavutil
import errno
import time
import math

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings

from MAVProxy.modules.mavproxy_map.mp_elevation import ElevationModel

class movinghome(mp_module.MPModule):
    def __init__(self, mpstate):
        # Initialise module
        super(movinghome, self).__init__(mpstate, "movinghome", "")
        # last/set home coordinates
        self.lath = 0
        self.lonh = 0
        self.alth = 0
        # misc settings
        self.updates_enabled = False
        self.radius = 15 # min travelled distance (m) before update
        self.check_interval = 3 # seconds
        self.last_check = time.time()
        self.fresh = True # fresh start/first movement
        self.dist = 0
        # home position ack settings
        self.home_position_ack_timeout = 3 # seconds
        self.last_home_position_ack = None # last acknowledged home position 
        self.time_last_home_sent = None # time of last home position sent
        self.mpstate.time_last_home_ack = None # time of last home position ack - shared variable from mavproxy_link module
        self.qgc_connection = None # to send messages to QGC
        self.debug_ack = False
        # elevation model
        self.EleModel = ElevationModel(database='srtm', offline=1, debug=False)
        self.add_command('movinghome', self.cmd_movinghome, "movinghome module")



    def cmd_movinghome(self, args):
        '''control behaviour of the module'''

        usage = "Usage: movinghome <status|on|off|radius|device|baud>\n On/Off: enable position update.\n Radius defines the threshold in meters (2D) from last position, when exceeded, home position is updated."

        if len(args) == 0:
            print(usage)
        elif args[0] == "status":
            self.status()
        elif args[0] == "on":
            self.movinghome_on()
        elif args[0] == "off":
            self.movinghome_off()
        elif args[0] == "radius":
            if len(args) < 2:
                print("Usage: moving base minimum travel radius <RADIUS>")
                return
            self.radius=float(args[1])
        else:
            print(usage)

    def status(self):
        # Returns information about module'''
        if self.updates_enabled == True:
            print("Last known GCS position lat %(lat)f lon=%(lon)f  max radius=%(max).1fm" %
                   {"lat": self.lath,
                    "lon": self.lonh,
                    "max": self.radius,
                   })
        else:
            print("Home position updates not enabled")
        print("Radius is %sm \nInterval is %ss \n" % (self.radius, self.check_interval))


    def movinghome_on(self):
        self.updates_enabled = True
        self.lath = 0 # ensure push of current home.
        print("Home position will be updated if GCS moves > %(max).1fm" %
               {"max": self.radius,
               })

    def movinghome_off(self):
        self.updates_enabled = False
        print("Home position will not be updated.")

    def check_home_position_ack(self):
        time_last_home_ack=self.mpstate.time_last_home_ack
        if time_last_home_ack is None:
            self.alert_mavproxy_and_qgc("Never received a Home position ACK! Dont take off!", mavutil.mavlink.MAV_SEVERITY_CRITICAL)
        elif time_last_home_ack and self.time_last_home_sent:
            time_since_last_ack =  time_last_home_ack - self.time_last_home_sent
            if self.debug_ack:
                self.alert_mavproxy_and_qgc("Home pos time_since_last_ack: %s s" % round(time_since_last_ack,4),mavutil.mavlink.MAV_SEVERITY_CRITICAL)
            if time_since_last_ack > self.home_position_ack_timeout: # Timeout
                self.alert_mavproxy_and_qgc("Home position ACK timed out: %s s" % round(time_since_last_ack,4),mavutil.mavlink.MAV_SEVERITY_CRITICAL)
                if self.last_home_position_ack:
                    self.alert_mavproxy_and_qgc("Last successful sent home position",mavutil.mavlink.MAV_SEVERITY_CRITICAL)
                    self.alert_mavproxy_and_qgc("lat:%s lon:%s" % (round(self.lath,4), round(self.lonh,4)),mavutil.mavlink.MAV_SEVERITY_CRITICAL) # ~11m accuracy
            else: # save last acknowledged home position 
                self.last_home_position_ack = {"lat": self.lath, "lon": self.lonh, "alt": self.alth}

    def alert_mavproxy_and_qgc(self, msg, severity=mavutil.mavlink.MAV_SEVERITY_NOTICE):
        # Send alert to mavproxy console
        self.console.writeln(
            msg,
        )
        # Send alert to QGC
        if self.qgc_connection is None:    
            self.connect_to_gqc()
        if self.qgc_connection is not None:
            self.qgc_connection.mav.statustext_send(
                severity,
                msg.encode("utf-8")
            )
            
    def connect_to_gqc(self):
        target_system=self.mpstate.settings.target_system
        if target_system and target_system != 0:
            self.qgc_connection = mavutil.mavlink_connection('udpout:0.0.0.0:14550', source_system=target_system, source_component=220)

    def idle_task(self):

        # Called frequently by mavproxy
        if not self.updates_enabled:
            return

        position = self.mpstate.position

        time_now = time.time()
        # TODO maybe add a check on position.timestamp here
        # e.g. if not position or (position && (time.time() - position.timestamp > some falue)
        # print error and return


        # check if we have a position and the position contains actual data from
        # a GGA message, if there is a latitude the rest exists as well
        if position and position.latitude:
            if position.num_sats and position.num_sats > 5 and (time_now - self.last_check > self.check_interval):
                self.check_home_position_ack()

                self.last_check = time_now
                # check if we moved enough
                self.dist = self.haversine(
                        position.longitude,
                        position.latitude,
                        position.altitude,
                        self.lonh,
                        self.lath,
                        self.alth
                        )

                if self.dist > self.radius:
                    if self.fresh == True:
                        self.say("GCS position set as home")
                        self.fresh = False
                    else:
                        message = "GCS moved %.0f" % self.dist + "meters"
                        self.say("%s: %s" % (self.name, message))
                        self.master.mav.statustext_send(
                                mavutil.mavlink.MAV_SEVERITY_NOTICE,
                                message.encode("utf-8")
                                )

                    srtm_alt = self.EleModel.GetElevation(position.latitude, position.longitude, timeout=0) # returns 0 at sea - timeout==0 means try to download once
                    if srtm_alt is None: # just in case...
                        srtm_alt=0
                    self.console.writeln("SRTM-Altitude: {}m".format(srtm_alt))

                    self.master.mav.command_long_send(
                        self.settings.target_system,
                        self.settings.target_component,
                        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                        0, # confirmation
                        0, # (0, set location specified in msg)
                        0, # param2
                        0, # param3
                        0, # param4
                        position.latitude, # param5
                        position.longitude, # param6
                        srtm_alt # param7,
                        # Elevation models report 0 too at sea
                    )
                    self.time_last_home_sent = time.time()

                    self.lath = position.latitude
                    self.lonh = position.longitude
                    self.alth = position.altitude
                    # self.console.writeln("%s: %s %s GNSS Quality %s Sats %s " % (self.name, self.lat, self.lon, msg.gps_qual, msg.num_sats))

                    self.console.writeln("Home position update sent...")


    def haversine(self, lon1, lat1, alt1, lon2, lat2, alt2):
        r_earth = 6371000
        lon1, lat1, lon2, lat2 = map(math.radians, [lon1, lat1, lon2, lat2])
        dlon = lon2 - lon1
        dlat = lat2 - lat1
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.asin(math.sqrt(a))
        d = c*r_earth
        return math.sqrt(d**2+(alt1 - alt2)**2)


def init(mpstate):
    return movinghome(mpstate)
