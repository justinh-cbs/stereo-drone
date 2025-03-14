from .setpoint_stream import SetpointStream, SetpointMessage
import threading
import time as timelib
import shared_buffer

class PlanningBase:
    _lock = threading.Lock()
    _running = False
    _loop_dt = 0.01 #0.005
    _replan_dt = 0.1
    _loop_last = timelib.time()
    _replan_last = _loop_last
    _sb = None
    
    def __init__(self, sb):
        print('Initializing PlanningBase')
        self._sb = sb.getInstance()
        self._setpoint_stream = SetpointStream()
        self._sb.register(self._setpoint_stream, "Setpoint")
    
    def replan(self):
        #print('PlanningBase.update()')
        pitch = 1500
        roll = 1500
        yaw = 1500
        thrust = 1600
        msg = SetpointMessage(timelib.time(), roll, pitch, yaw, thrust)
        self._setpoint_stream.addOne(msg)

    def loop(self):
        #print('PlanningBase.loop()')
        pass
    
    def start(self):
        with self._lock:
            print('PlanningBase Start Running')
            self._running = True
    
    def stop(self):
        with self._lock:
            print('PlanningBase Stop Running')
            self._running = False
    
    def loop(self):
        while True:
            with self._lock:
                if self._running:
                    self._loop_last = timelib.time()

                    if self._loop_last > (self._replan_last + self._replan_dt):
                        self._replan_last = self._loop_last
                        self.replan()
                    
                    if timelib.time() < (self._loop_last + self._loop_dt):
                        timelib.sleep(self._loop_last + self._loop_dt - timelib.time())
                    else:
                        print('PlanningBase loop took too long')
                else:
                    print('Exiting PlanningBase loop')
                    break

class Planning(PlanningBase):
    def __init__(self, sb):
        print('Initializing Planning')
        super(Planning, self).__init__(sb)