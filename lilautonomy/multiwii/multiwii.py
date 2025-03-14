from yamspy import MSPy
import threading
import time as timelib
import shared_buffer
from .imu_stream import IMUMessage, IMUStream

class FCInterfaceBase:
    _lock = threading.Lock()
    _running = False
    _loop_dt = 0.1 #0.005
    _get_dt = 5 #0.01
    _set_dt = 3 #0.01
    _loop_last = timelib.time()
    _get_last = _loop_last
    _set_last = _loop_last
    _sb = None
    _imu_stream = None

    def __init__(self, sb):
        print('Initializing FCInterfaceBase')
        self._sb = sb.getInstance()
        self._imu_stream = IMUStream()
        self._sb.register(self._imu_stream, "IMU")

    def get(self):
        print('FCInterfaceBase.get()')
    
    def set(self):
        print('FCInterfaceBase.set()')
    
    def start(self):
        with self._lock:
            print('FCInterface Start Running')
            self._running = True
    
    def stop(self):
        with self._lock:
            print('FCInterface Stop Running')
            self._running = False
    
    def loop(self):
        while True:
            with self._lock:
                if self._running:
                    self._loop_last = timelib.time()

                    if self._loop_last > (self._get_last + self._get_dt):
                        self._get_last = self._loop_last
                        self.get()
                    
                    if self._loop_last > (self._set_last + self._set_dt):
                        self._set_last = self._loop_last
                        self.set()
                    
                    if timelib.time() < (self._loop_last + self._loop_dt):
                        timelib.sleep(self._loop_last + self._loop_dt - timelib.time())
                    else:
                        print('FCInterfaceBase loop took too long')
                else:
                    print('Exiting FCInterfaceBase loop')
                    break

class MultiWiiInterface(FCInterfaceBase):
    board = None

    def __init__(self, sb):
        print('MultiWiiInterface init')
        self.board = MSPy(device="/dev/ttyAMA1", loglevel='WARNING', baudrate=115200)
        self.board.connect()
        self._loop_dt = 0.01 #0.005
        self._get_dt = 0.01 #0.01
        self._set_dt = 10 #0.01
        super(MultiWiiInterface, self).__init__(sb)

    def stop(self):
        with self._lock:
            print('FCInterface Stop Running')
            self.board.__exit__(0, 0, 0)
            self._running = False

    def get(self):
        #with self.board as board:
            #board.fast_read_imu()
        self.board.fast_read_imu()
        gyroscope = self.board.SENSOR_DATA['gyroscope']
        accelerometer = self.board.SENSOR_DATA['accelerometer']
        msg = IMUMessage(timelib.time(), accelerometer, gyroscope)
        self._imu_stream.addOne(msg)

        #print(f"accelerometer: {accelerometer}   gyroscope: {gyroscope}")

            #ARMED = board.bit_check(board.CONFIG['mode'],0)
            #print("ARMED: {}".format(ARMED))
            #print("armingDisableFlags: {}".format(board.process_armingDisableFlags(board.CONFIG['armingDisableFlags'])))
            #print("cpuload: {}".format(board.CONFIG['cpuload']))
            #print("cycleTime: {}".format(board.CONFIG['cycleTime']))
            #print("mode: {}".format(board.CONFIG['mode']))
            #print("Flight Mode: {}".format(board.process_mode(board.CONFIG['mode'])))

            #accelerometer = board.SENSOR_DATA['accelerometer']
            #gyroscope = board.SENSOR_DATA['gyroscope']

            #print(f"accelerometer: {accelerometer}   gyroscope: {gyroscope}")

    def set(self, armed):
        print('MultiWiiInterface.set placeholder')
        # TODO copy this from lilsim, try it on real quad
        CMDS = {
                'roll':     1500,
                'pitch':    1500,
                'throttle': 900,
                'yaw':      1500,
                'aux1':     1000,
                'aux2':     1000
                }
        CMDS_ORDER = ['roll', 'pitch', 'throttle', 'yaw', 'aux1', 'aux2']

        #with self.board as board:

        # disarm
        if armed:
            CMDS['aux1'] = 1800
        else:
            CMDS['aux1'] = 1000

        # set throttle
        CMDS['throttle'] = 988



        # mode
        CMDS['aux2'] <= 1300 # Horizon mode
        1700 > CMDS['aux2'] > 1300 # Flip Mode
        CMDS['aux2'] >= 1700 # Angle Mode

        # roll
        CMDS['roll'] = 1500

        # pitch
        CMDS['pitch'] = 1500
        self.board.send_RAW_RC([CMDS[ki] for ki in CMDS_ORDER])


    # def start(self):  -- using base class
    # def stop(self):   -- using base class
    
    def loop(self):
        while True:
            with self._lock:
                if self._running:
                    self._loop_last = timelib.time()

                    if self._loop_last > (self._get_last + self._get_dt):
                        self._get_last = self._loop_last
                        self.get()
                    
                    if self._loop_last > (self._set_last + self._set_dt):
                        self._set_last = self._loop_last
                        self.set(armed=False)
                    
                    current_time = timelib.time()
                    if current_time < (self._loop_last + self._loop_dt):
                        timelib.sleep(self._loop_last + self._loop_dt - current_time)
                    else:
                        print('MultiWiiInterface loop took too long')
                else:
                    print('Exiting MultiWiiInterface loop')
                    break
