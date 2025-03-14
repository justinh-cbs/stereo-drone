
import os
import signal
import threading
import time as timelib
from vpython import *
from lilsim import LilSim

class LilVizBase:
    _lock = threading.Lock()
    _running = False
    _loop_dt = 0.1 #0.05
    _update_dt = 0.25 #0.25
    _loop_last = timelib.time()
    _update_last = _loop_last
    _sb = None

    def __init__(self, sb):
        print('Initializing LilVizBase')
        self._sb = sb.getInstance()

    def update(self):
        print('LilVizBase.update()')
        x, y, z = self.lilsim.show()
        try:
            self.model.rotate(angle=(x / 1800), axis=vector(0,0,1))
            self.model.rotate(angle=(y / 1800), axis=vector(1,0,0))
            self.model.rotate(angle=(z / 1800), axis=vector(0,1,0))
        except IndexError as err:
            print(err)
            print('bad imu values?')
            os.kill(os.getpid(), signal.SIGINT)

    def loop(self):
        print('LilVizBase.loop()')
    
    def start(self):
        with self._lock:
            print('LilVizBase Start Running')
            self.lilsim = LilSim()
            self.model = box(up=vector(0,1,0), color=color.blue, length=3, height=0.4, width=3)
            x_ang, y_ang, z_ang = self._quadrotor.get_rpy()
            self._running = True
    
    def stop(self):
        with self._lock:
            print('LilVizBase Stop Running')
            self._running = False
    
    def loop(self):
        while True:
            with self._lock:
                if self._running:
                    self._loop_last = timelib.time()

                    if self._loop_last > (self._update_last + self._update_dt):
                        self._update_last = self._loop_last
                        self.update()

                    if timelib.time() < (self._loop_last + self._loop_dt):
                        timelib.sleep(self._loop_last + self._loop_dt - timelib.time())
                    else:
                        print('LilVizBase loop took too long')
                else:
                    print('Exiting LilVizBase loop')
                    break

class LilViz(LilVizBase):
    def __init__(self, sb):
        print('Initializing Visualization...')
        super(LilViz, self).__init__(sb)


    # model = box(up=vector(0,1,0), color=color.blue, length=3, height=0.4, width=3)

        # try:
    #        entry = imu_log.readline()
    #        entry_list = [i for i in re.findall(r'[-+]?\d+(?:\.\d+)?', entry) if i]

    #        print(f'x: {entry_list[0]} y: {entry_list[1]} z: {entry_list[2]}')

            # pete help me with maths plz i don't know how :(
            # entry_list = [0, 0, 0, 0, 0, 0]
            # x_ang = int(entry_list[3])# - x_val
            # y_ang = int(entry_list[4])# - y_val
            # z_ang = int(entry_list[5])# - z_val

            # x_ang, y_ang, z_ang = self._quadrotor.get_rpy()

            # model.rotate(angle=(x_ang / 1800), axis=vector(0,0,1))
            # model.rotate(angle=(y_ang / 1800), axis=vector(1,0,0))
            # model.rotate(angle=(z_ang / 1800), axis=vector(0,1,0))

            #x_val = x_ang
            #y_val = y_ang
            #z_val = z_ang
            #timelib.sleep(0.009)

        # except IndexError as err:
        #     print(err)
        #     print('end of file?')
        #     os.kill(os.getpid(), signal.SIGINT)