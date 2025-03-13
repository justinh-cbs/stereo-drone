
import threading
import time as timelib

class StateEstimatorBase:
    _lock = threading.Lock()
    _running = False
    _loop_dt = 0.01 #0.005
    _propagate_dt = 0.1 #0.01
    _update_dt = 3 #0.1
    _loop_last = timelib.time()
    _propagate_last = _loop_last
    _update_last = _loop_last
    _sb = None
    _imu_stream = None
    _last_imu = None
    _feature_stream = None
    _last_feature = None

    def __init__(self, sb):
        print('Initializing StateEstimatorBase')
        self._sb = sb.getInstance()
        self._last_imu = timelib.time()

    def propagate(self):
        #print('StateEstimatorBase.propagate()')
        pass

    def update(self):
        #print('StateEstimatorBase.update()')
        pass

    def loop(self):
        #print('StateEstimatorBase.loop()')
        pass

    def start(self):
        with self._lock:
            print('StateEstimatorBase Start Running')
            self._running = True

    def stop(self):
        with self._lock:
            print('StateEstimatorBase Stop Running')
            self._running = False

class StateEstimator(StateEstimatorBase):
    def __init__(self, sb):
        print('Initializing StateEstimator')
        super(StateEstimator, self).__init__(sb)

    def propagate(self):
        #print('StateEstimator.propagate()')
        # if we haven't already, connect to IMU stream
        if self._imu_stream is None:
            print('Connecting to IMU stream')
            self._imu_stream = self._sb.connect("IMU")
        
        if self._imu_stream is not None:
            imu_messages = self._imu_stream.getAll(self._last_imu)
            print(f'Found {len(imu_messages)} new imu messages')
            for msg in imu_messages:
                if msg._tov > self._last_imu:
                    self._last_imu = msg._tov 

    def update(self):
        if self._feature_stream is None:
            print('Connecting to Feature stream')
            self._feature_stream = self._sb.connect("Feature")

        if self._feature_stream is not None:
            feature_messages = self._feature_stream.getAll(self._last_feature)
            print(f'Found {len(feature_messages)} new feature messages')
            for msg in feature_messages:
                if msg._tov > self._last_feature:
                    self._last_feature = msg._tov
            # if feature messages is not empty, this is where we
            # put update into Kalman filter


    def loop(self):
        while True:
            with self._lock:
                if self._running:
                    self._loop_last = timelib.time()

                    if self._loop_last > (self._propagate_last + self._propagate_dt):
                        self._propagate_last = self._loop_last
                        self.propagate()

                    if self._loop_last > (self._update_last + self._update_dt):
                        self._update_last = self._loop_last
                        self.update()

                    if timelib.time() < (self._loop_last + self._loop_dt):
                        timelib.sleep(self._loop_last + self._loop_dt - timelib.time())
                    else:
                        print('StateEstimatorBase loop took too long')
                else:
                    print('Exiting StateEstimatorBase loop')
                    break