import os
import signal
from vpython import *
import re
import threading
import time as timelib
import shared_buffer
from multiwii import FCInterfaceBase
import time as timelib
from multiwii.imu_stream import IMUMessage, IMUStream

from .quadrotor import Quadrotor


class LilSim(FCInterfaceBase):
    """Simulation class.

    Derived from the FCInterfaceBase class so that interface is identical to MultiWiiInterface base
    """

    def __init__(self, sb, headless):
        print('LilSim init')
        self._get_dt = 1.0
        self._main_dt = 0.1
        super(LilSim, self).__init__(sb)
        self._main_last = self._loop_last
        self._setpoint_stream = None
        self._last_setpoint = timelib.time()
        self._headless = headless
        self._sim_speed = 1
        self._loop_dt = self._main_dt * 0.1

        self._quadrotor = Quadrotor()

    def main(self):
        # this is the main simulation loop
        # TODO compute force and moment from states and setpoint
        # TODO kinematics from force and moment
        self._quadrotor.propagate(self._sim_speed * self._main_dt)

        if not self._headless:
            self.show()

    def get(self):
        # after we implement main(), grab actual accels and gyros i suppose (rates)
        print('LilSim get')
        acceleration = [0, 0, 9.8]
        rate = [0, 0, 0]
        msg = IMUMessage(timelib.time(), acceleration, rate)
        self._imu_stream.addOne(msg)
    
    def set(self):
        # TODO instead of printing, store setpoint
        print('LilSim set')

        if self._setpoint_stream is None:
            print('Connecting to Setpoint stream')
            self._setpoint_stream = self._sb.connect("Setpoint")
        
        if self._setpoint_stream is not None:
            setpoint_messages = self._setpoint_stream.getAll(self._last_setpoint)
            print(f'Found {len(setpoint_messages)} new setpoint message(s): ')
            for msg in setpoint_messages:
                if msg._tov > self._last_setpoint:
                    self._last_setpoint = msg._tov
                    self._quadrotor.set_rpyt(msg._roll, msg._pitch, msg._yaw, msg._thrust)

    def show(self):
        # quad visualizer
        # x_ang, y_ang, z_ang = self._quadrotor.get_rpy()
        
        # set up visualization model
        # IMPORTANT: this doesn't work in a thread! gotta move it to main thread, and get published imu values
        # ptr = box(up=vector(0,1,0), color=color.blue, length=3, height=0.4, width=3)

        # try:
    #        entry = imu_log.readline()
    #        entry_list = [i for i in re.findall(r'[-+]?\d+(?:\.\d+)?', entry) if i]

    #        print(f'x: {entry_list[0]} y: {entry_list[1]} z: {entry_list[2]}')

            # entry_list = [0, 0, 0, 0, 0, 0]
            # x_ang = int(entry_list[3])# - x_val
            # y_ang = int(entry_list[4])# - y_val
            # z_ang = int(entry_list[5])# - z_val

            # x_ang, y_ang, z_ang = self._quadrotor.get_rpy()

            # ptr.rotate(angle=(x_ang / 1800), axis=vector(0,0,1))
            # ptr.rotate(angle=(y_ang / 1800), axis=vector(1,0,0))
            # ptr.rotate(angle=(z_ang / 1800), axis=vector(0,1,0))

            #x_val = x_ang
            #y_val = y_ang
            #z_val = z_ang
            #timelib.sleep(0.009)

        # except IndexError as err:
        #     print(err)
        #     print('end of file?')
        #     os.kill(os.getpid(), signal.SIGINT)
        pass


    def loop(self):
        while True:
            with self._lock:
                if self._running:
                    self._loop_last = timelib.time()

                    if self._loop_last > (self._set_last + self._set_dt):
                        self._set_last = self._loop_last
                        self.set()

                    if self._loop_last > (self._main_last + self._main_dt):
                        self._main_last = self._loop_last
                        self.main()

                    if self._loop_last > (self._get_last + self._get_dt):
                        self._get_last = self._loop_last
                        self.get()

                    if timelib.time() < (self._loop_last + self._loop_dt):
                        timelib.sleep(self._loop_last + self._loop_dt - timelib.time())
                    else:
                        print('LilSim loop took too long')
                else:
                    print('Exiting LilSim loop')
                    break
