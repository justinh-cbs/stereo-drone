
import threading
import time as timelib

class MappingBase:
    _lock = threading.Lock()
    _running = False
    _loop_dt = 0.1 #0.05
    _update_dt = 3 #0.25
    _loop_last = timelib.time()
    _update_last = _loop_last
    _sb = None

    def __init__(self, sb):
        print('Initializing MappingBase')
        self._sb = sb.getInstance()

    def update(self):
        #print('MappingBase.update()')
        pass

    def loop(self):
        #print('MappingBase.loop()')
        pass
    
    def start(self):
        with self._lock:
            print('MappingBase Start Running')
            self._running = True
    
    def stop(self):
        with self._lock:
            print('MappingBase Stop Running')
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
                        print('MappingBase loop took too long')
                else:
                    print('Exiting MappingBase loop')
                    break

class Mapping(MappingBase):
    def __init__(self, sb):
        print('Initializing Mapping')
        super(Mapping, self).__init__(sb)