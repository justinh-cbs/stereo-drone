#!/usr/bin/python

import logging
import threading
import time as timelib

from multiwii import MultiWiiInterface 
from SE import StateEstimator
from mapping import Mapping
from planning import Planning
from shared_buffer import SharedBuffer
from lil_rs_tracker import RSTracker

# some settings
simulation = True
main_loop_dt = 1

# create the shared buffer
sb = SharedBuffer()

# some setup
if simulation:
    from lilsim import LilSim
    from vpython import *
    FCInterface = LilSim(sb, headless=True)
else:
    FCInterface = MultiWiiInterface(sb)

FCInterfaceThread = threading.Thread(target=FCInterface.loop) # could probably move thread into the Interface class
FCInterface.start()
FCInterfaceThread.start()

SE = StateEstimator(sb)
SEThread = threading.Thread(target=SE.loop) # could probably move thread into the StateEstimator class
SE.start()
SEThread.start()

Map = Mapping(sb)
MapThread = threading.Thread(target=Map.loop) # could probably move thread into the Mapping class
Map.start()
MapThread.start()

RST = RSTracker(sb)
RSTThread = threading.Thread(target=RST.loop)
RST.start()
RSTThread.start()

Planner = Planning(sb)
PlannerThread = threading.Thread(target=Planner.loop) # could probably move thread into the Planning class
Planner.start()
PlannerThread.start()

start_time = timelib.time()
time = start_time
while time < start_time + 10:
    print(f'time = {time}')
    print('')

    # create a vpython box object above
    # publish vehicle states in lilsim.py
    # add a subscriber to get them here
    # update box states here with box.rotate() or something

    timelib.sleep(time + main_loop_dt - timelib.time())

    time = time + main_loop_dt

print("stopping FC Interface...")
FCInterface.stop()
FCInterfaceThread.join()

print("stopping SE...")
SE.stop()
SEThread.join()

print("stopping map...")
Map.stop()
MapThread.join()

print("stopping RST...")
RST.stop()
RSTThread.join()

print("stopping planner...")
Planner.stop()
PlannerThread.join()

print("done!")