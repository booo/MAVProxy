#!/usr/bin/env python
'''module template'''
import time, math, struct
from pymavlink import mavutil
from pymavlink.dialects.v10 import common as commonMavlinkDialect
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib.mp_settings import MPSetting
from datetime import datetime

# ASSUMPTION: There is only one system in operation and we receive messages only
# from one system (id).

def is_armed(msg):
  return bool(msg.base_mode & commonMavlinkDialect.MAV_MODE_FLAG_SAFETY_ARMED)

class SearchWingLogModule(mp_module.MPModule):
  def __init__(self, mpstate):
    super(SearchWingLogModule, self).__init__(mpstate, "searchwing", "searchwing log module")
    '''initialisation code'''
    self.armed = None
    self.log_file = None
    print("SearchWing log module initialized!")

  def mavlink_packet(self, m):
    '''handle a mavlink packet'''
    if m.get_type() == 'HEARTBEAT':
        # TODO integrate system_id into log name/directory
        system_id = m.get_srcSystem()
        armed = is_armed(m)
        if self.log_file and armed:
            # log file open and plane is armed, continue logging
            pass
        elif self.log_file and not armed:
            # log file still open but plane is disarmed, close log file
            # write last heartbeat msg otherwise we loose it
            usec = int(time.time() * 1.0e6)
            self.log_file.write(bytearray(struct.pack('>Q', usec) + m.get_msgbuf()))
            self.log_file.close()
            self.log_file = None
        elif not self.log_file and armed:
            # no log file open but plane is armed, open new log file
            now = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
            # TODO add an command to change the log directory
            file_name = f'/tmp/logs/{now}.tlog'
            self.log_file = open(file_name, "wb")
        elif not self.log_file and not armed:
            # no log file open but plane is disarmed, do not open log file
            pass

    if self.log_file:
        usec = int(time.time() * 1.0e6)
        self.log_file.write(bytearray(struct.pack('>Q', usec) + m.get_msgbuf()))
        self.log_file.flush()


def init(mpstate):
  '''initialise module'''
  return SearchWingLogModule(mpstate)
