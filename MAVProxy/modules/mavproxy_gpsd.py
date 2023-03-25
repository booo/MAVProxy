#!/usr/bin/env python
'''
GPSD GPS connector
connect to a gpsd and provide this as location position
'''

import sys, os, time
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib import mp_util

try:
    from gpsdclient import GPSDClient
except ImportError as e:
    print('please install gpsdclient package with "python -m pip install gpsdclient"')

class GPSDModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(GPSDModule, self).__init__(mpstate, "GPSD", "GPSD input")
        self.gpsd_settings = mp_settings.MPSettings([
            ("host", str, "127.0.0.1"),
            ("port", str, 2947),
            ])
        self.add_completion_function('(GPSDSETTING)',
                                     self.gpsd_settings.completion)
        self.add_command('gpsd', self.cmd_gpsd, "gpsd GPS input control",
                         ["<status|connect|disconnect>", "set (GPSDSETTING)"])
        self.client = None
        self.position = mp_util.mp_position()

    def cmd_gpsd(self, args):
        '''gpsd commands'''
        usage = "gpsd <set|connect|disconnect|status>"
        if len(args) == 0:
            print(usage)
            return
        if args[0] == "set":
            self.gpsd_settings.command(args[1:])
        elif args[0] == "connect":
            self.cmd_connect()
        elif args[0] == "disconnect":
            self.cmd_disconnect()
        elif args[0] == "status":
            self.cmd_status()
        else:
            print(usage)

    def cmd_connect(self):
        '''connect to GPS'''
        try:
            self.client = GPSDClient(
                    host=self.gpsd_settings.host,
                    port=self.gpsd_settings.port
            )
        except Exception as ex:
            print("Failed to open %s:%s - %s" % (self.gpsd_settings.host, self.gpsd_settings.port, ex))
        self.stream = self.client.dict_stream(
                convert_datetime=True,
                filter=["TPV", "SKY"]
                )
    def cmd_disconnect(self):
        '''disconnect from GPS'''
        if self.client is not None:
            self.client.close()
            self.client = None
        else:
            print("GPS not connected")

    def cmd_status(self):
        '''status'''
        if self.client is None:
            print("GPS not connected")
            return
        if self.position.timestamp is None:
            print("No position")
            return
        print(self.position)

    def idle_task(self):
        '''check for new data'''
        if self.client is None:
            return
        try:
            result = next(self.stream)
        except StopIteration as ex:
            return
        self.position.num_sats = result.get("uSat", self.position.num_sats)
        self.position.latitude = result.get("lat", self.position.latitude)
        self.position.longitude = result.get("lon", self.position.longitude)
        self.position.altitude = result.get("altMSL", self.position.altitude)
        self.position.timestamp = time.time() # should maybe be result.get("time", None)
        self.position.ground_course = result.get(
                "track",
                self.position.ground_course
                )
        self.position.ground_speed = result.get(
                "speed",
                self.position.ground_speed
                )
        self.mpstate.position = self.position

def init(mpstate):
    '''initialise module'''
    return GPSDModule(mpstate)
