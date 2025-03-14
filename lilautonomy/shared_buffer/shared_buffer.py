
import threading
import time as timelib
from . import message_base
from . import stream_base

# we should try using queue

class SharedBuffer:
    """SharedBuffer class.

    Singleton class for sharing data between threads.  All publishers will be managed by the
    SharedBuffer class, and threads will access publishers through it.
    """

    _instance = None
    _registry = dict()

    @staticmethod 
    def getInstance():
        """ Static access method. """
        if SharedBuffer._instance == None:
            SharedBuffer()
        return SharedBuffer._instance

    def __init__(self):
        """ Virtually private constructor. """
        print('Init SharedBuffer')
        if SharedBuffer._instance != None:
            raise Exception("This class is a singleton!")
        else:
            SharedBuffer._instance = self
    
    def register(self, pub, name):
        self._registry[name] = pub
    
    def connect(self, name):
        try:
            return self._registry[name]
        except KeyError:
            print(f'SharedBuffer registry does not contain {name}')
            return None
        
