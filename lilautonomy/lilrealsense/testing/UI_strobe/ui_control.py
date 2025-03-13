import struct
import time
import curses
from collections import deque
from itertools import cycle

import multiprocessing
from queue import Empty
import lilrealsense.testing.UI_strobe.realsense_depth as realsense_depth
from yamspy import MSPy

# Max periods for:
CTRL_LOOP_TIME = 1/100
SLOW_MSGS_LOOP_TIME = 1/5 # these messages take a lot of time slowing down the loop...

NO_OF_CYCLES_AVERAGE_GUI_TIME = 10

SERIAL_PORT = "/dev/ttyAMA1"

def realsense(out_q):
    realsense_depth.main(out_q, testing=False)

def ir_stream(out_q):
    ir.main(out_q, testing=False)

def stop(pool):
    pool.terminate()
    pool.join()
    exit(0)

def altitude(board):
    if board.send_RAW_msg(MSPy.MSPCodes['MSP_SONAR'], data=[]):
        # 2. Response msg from the flight controller is received
        dataHandler = board.receive_msg()
        # 3. The msg is parsed
        board.process_recv_data(dataHandler)
        # 4. After the parser, the instance is populated.
        # In this example, SENSOR_DATA has its altitude value updated.
        return board.SENSOR_DATA['sonar']

def fast_read_altitude(board):
    # Request altitude
    if board.send_RAW_msg(MSPy.MSPCodes['MSP_SONAR'], data=[]):
        # $ + M + < + data_length + msg_code + data + msg_crc
        # 6 bytes + data_length
        data_length = 4
        msg = board.receive_raw_msg(size = (6+data_length))[5:]
        converted_msg = struct.unpack('<i', msg[:-1])[0]
        return round((converted_msg / 100.0), 2) # correct scale factor

def run_curses(external_function):
    result = 1

    try:
        # get the curses screen window
        screen = curses.initscr()

        # turn off input echoing
        curses.noecho()

        # respond to keys immediately (don't wait for enter)
        curses.cbreak()

        # non-blocking
        screen.timeout(0)

        # map arrow keys to special values
        screen.keypad(True)

        screen.addstr(1, 0, "Press 'q' to quit, 'r' to reboot, 'm' to change mode, 'a' to arm, 'd' to disarm, 'n' for autonomous, 't' for takeoff, 'l' to land", curses.A_BOLD)

        result = external_function(screen)

    finally:
        # shut down cleanly
        curses.nocbreak(); screen.keypad(0); curses.echo()
        curses.endwin()
        if result==1:
            print("An error occurred... probably the serial port is not available ;)")

def keyboard_controller(screen):

    CMDS = {
            'roll':     1500,
            'pitch':    1500,
            'throttle': 900,
            'yaw':      1500,
            'aux1':     1000,
            'aux2':     1000,
            'aux3':     1000,
            'loop':     0
            }

    try:
    # Create the shared queue and launch both threads
        manager = multiprocessing.Manager()
        # make a queue
        q = manager.Queue()
        # make another queue??
        q2 = manager.Queue()
        pool = multiprocessing.Pool()
        t1 = pool.apply_async(realsense, [q])
        t2 = pool.apply_async(ir_stream, [q2])
        time.sleep(5)

    except KeyboardInterrupt:
        # item = "land"
        # q.put(item)
        # time.sleep(5)
        pool.terminate()
        pool.join()

    # This order is the important bit: it will depend on how your flight controller is configured.
    # Below it is assuming the flight controller is set to use AETR.
    # In the documentation, iNAV uses CH5, CH6, etc while Betaflight goes AUX1, AUX2...
    CMDS_ORDER = ['roll', 'pitch', 'throttle', 'yaw', 'aux1', 'aux2', 'aux3', 'loop']

    # "print" doesn't work with curses, use addstr instead
    try:
        screen.addstr(15, 0, "Connecting to the FC...")

        with MSPy(device=SERIAL_PORT, loglevel='WARNING', baudrate=115200) as board:
            if board == 1: # an error occurred...
                return 1

            screen.addstr(15, 0, "Connecting to the FC... connected!")
            screen.clrtoeol()
            screen.move(1,0)

            average_cycle = deque([0]*NO_OF_CYCLES_AVERAGE_GUI_TIME)

            # It's necessary to send some messages or the RX failsafe will be activated
            # and it will not be possible to arm.
            command_list = ['MSP_API_VERSION', 'MSP_FC_VARIANT', 'MSP_FC_VERSION', 'MSP_BUILD_INFO', 
                            'MSP_BOARD_INFO', 'MSP_UID', 'MSP_ACC_TRIM', 'MSP_NAME', 'MSP_STATUS', 'MSP_STATUS_EX',
                            'MSP_BATTERY_CONFIG', 'MSP_BATTERY_STATE', 'MSP_BOXNAMES']

            if board.INAV:
                command_list.append('MSPV2_INAV_ANALOG')
                command_list.append('MSP_VOLTAGE_METER_CONFIG')

            for msg in command_list: 
                if board.send_RAW_msg(MSPy.MSPCodes[msg], data=[]):
                    dataHandler = board.receive_msg()
                    board.process_recv_data(dataHandler)
            if board.INAV:
                cellCount = board.BATTERY_STATE['cellCount']
            else:
                cellCount = 0 # MSPV2_INAV_ANALOG is necessary
            min_voltage = board.BATTERY_CONFIG['vbatmincellvoltage']*cellCount
            warn_voltage = board.BATTERY_CONFIG['vbatwarningcellvoltage']*cellCount
            max_voltage = board.BATTERY_CONFIG['vbatmaxcellvoltage']*cellCount

            screen.addstr(15, 0, "apiVersion: {}".format(board.CONFIG['apiVersion']))
            screen.clrtoeol()
            screen.addstr(15, 50, "flightControllerIdentifier: {}".format(board.CONFIG['flightControllerIdentifier']))
            screen.addstr(16, 0, "flightControllerVersion: {}".format(board.CONFIG['flightControllerVersion']))
            screen.addstr(16, 50, "boardIdentifier: {}".format(board.CONFIG['boardIdentifier']))
            screen.addstr(17, 0, "boardName: {}".format(board.CONFIG['boardName']))
            screen.addstr(17, 50, "name: {}".format(board.CONFIG['name']))


            slow_msgs = cycle(['MSP_ANALOG', 'MSP_STATUS_EX', 'MSP_MOTOR', 'MSP_RC'])

            cursor_msg = ""
            last_loop_time = last_slow_msg_time = last_cycleTime = time.time()

            # bool to trigger realsense_control
            autonomous = False
            altitude = 0
            takeoff = False
            land = False

            while True:
                start_time = time.time()

                char = screen.getch() # get keypress
                curses.flushinp() # flushes buffer

                # Key input processing

                # KEYS (NO DELAYS)
                if char == ord('q') or char == ord('Q'):
                    break

                elif char == ord('d') or char == ord('D'):
                    cursor_msg = 'Sending Disarm command...'
                    CMDS['aux1'] = 1000

                elif char == ord('r') or char == ord('R'):
                    screen.addstr(3, 0, 'Sending Reboot command...')
                    screen.clrtoeol()
                    board.reboot()
                    time.sleep(0.5)
                    break

                elif char == ord('a') or char == ord('A'):
                    cursor_msg = 'Sending Arm command...'
                    CMDS['aux1'] = 1800

                # The code below is expecting the drone to have the
                # modes set accordingly since everything is hardcoded.

                elif char == ord('m') or char == ord('M'):
                    if CMDS['aux2'] <= 1300:
                        cursor_msg = 'Horizon Mode 1...'
                        CMDS['aux2'] = 1500
                    elif 1700 > CMDS['aux1'] > 1300:
                        cursor_msg = 'Horizon Mode 2...'
                        CMDS['aux2'] = 1800
                    elif CMDS['aux1'] >= 1700:
                        cursor_msg = 'Horizon Mode 3...'
                        CMDS['aux2'] = 1300

                elif char == ord('b') or char == ord('B'):
                    autonomous = False
                    CMDS['aux3'] = 1000

                elif char == ord('w') or char == ord('W'):
                    CMDS['throttle'] = CMDS['throttle'] + 10 if CMDS['throttle'] + 10 <= 2000 else CMDS['throttle']
                    save_throttle = save_throttle + 10
                    cursor_msg = 'W Key - throttle(+):{}'.format(CMDS['throttle'])

                elif char == ord('e') or char == ord('E'):
                    CMDS['throttle'] = CMDS['throttle'] - 10 if CMDS['throttle'] - 10 >= 1000 else CMDS['throttle']
                    save_throttle = save_throttle - 10
                    cursor_msg = 'E Key - throttle(-):{}'.format(CMDS['throttle'])

                elif char == curses.KEY_RIGHT:
                    CMDS['roll'] = CMDS['roll'] + 10 if CMDS['roll'] + 10 <= 2000 else CMDS['roll']
                    cursor_msg = 'Right Key - roll(-):{}'.format(CMDS['roll'])

                elif char == curses.KEY_LEFT:
                    CMDS['roll'] = CMDS['roll'] - 10 if CMDS['roll'] - 10 >= 1000 else CMDS['roll']
                    cursor_msg = 'Left Key - roll(+):{}'.format(CMDS['roll'])

                elif char == curses.KEY_UP:
                    CMDS['pitch'] = CMDS['pitch'] + 10 if CMDS['pitch'] + 10 <= 2000 else CMDS['pitch']
                    cursor_msg = 'Up Key - pitch(+):{}'.format(CMDS['pitch'])

                elif char == curses.KEY_DOWN:
                    CMDS['pitch'] = CMDS['pitch'] - 10 if CMDS['pitch'] - 10 >= 1000 else CMDS['pitch']
                    cursor_msg = 'Down Key - pitch(-):{}'.format(CMDS['pitch'])

                elif char == ord('t') or char == ord('T'):
                    takeoff = True
                    land = False
                    cursor_msg = 'TAKEOFF'

                elif char == ord('l') or char == ord('L'):
                    land = True
                    takeoff = False
                    cursor_msg = 'LAND'

                elif char == ord('n'):
                    autonomous = True
                    CMDS['aux3'] = 2000
                    cursor_msg = 'Autonomous mode - Active'

                # save throttle value to overwrite command from queue until it's reliable
                save_throttle = CMDS['throttle']

                if takeoff and altitude < 0.5 and altitude != -0.01 and save_throttle < 1500:
                    save_throttle = save_throttle + 1
                    CMDS['throttle'] = save_throttle
                    CMDS['pitch'] = 1500

                elif altitude > 0.5:
                    if save_throttle > 1300:
                        save_throttle = save_throttle - 1
                        CMDS['throttle'] = save_throttle

                if land and save_throttle > 900:
                    save_throttle = save_throttle - 1
                    if save_throttle <= 1000:
                        save_throttle = 1000
                    CMDS['throttle'] = save_throttle
                    CMDS['pitch'] = 1500
                    CMDS['yaw'] = 1500
                #
                # IMPORTANT MESSAGES (CTRL_LOOP_TIME based)
                #
                if (time.time()-last_loop_time) >= CTRL_LOOP_TIME:
                    last_loop_time = time.time()
                    # Send the RC channel values to the FC
                    if board.send_RAW_RC([CMDS[ki] for ki in CMDS_ORDER]):
                        dataHandler = board.receive_msg()
                        board.process_recv_data(dataHandler)

                        # get imu data
                        board.fast_read_imu()
                        accelerometer = board.SENSOR_DATA['accelerometer']
                        gyroscope = board.SENSOR_DATA['gyroscope']

                #
                # SLOW MSG processing (user GUI)
                #
                if (time.time()-last_slow_msg_time) >= SLOW_MSGS_LOOP_TIME:
                    last_slow_msg_time = time.time()

                    next_msg = next(slow_msgs) # circular list

                    # Read info from the FC
                    if board.send_RAW_msg(MSPy.MSPCodes[next_msg], data=[]):
                        dataHandler = board.receive_msg()
                        board.process_recv_data(dataHandler)
                        
                    if next_msg == 'MSP_ANALOG':
                        voltage = board.ANALOG['voltage']
                        voltage_msg = ""
                        if min_voltage < voltage <= warn_voltage:
                            voltage_msg = "LOW BATT WARNING"
                        elif voltage <= min_voltage:
                            voltage_msg = "ULTRA LOW BATT!!!"
                        elif voltage >= max_voltage:
                            voltage_msg = "VOLTAGE TOO HIGH"

                        screen.addstr(8, 0, "Battery Voltage: {:2.2f}V".format(board.ANALOG['voltage']))
                        screen.clrtoeol()
                        screen.addstr(8, 24, voltage_msg, curses.A_BOLD + curses.A_BLINK)
                        screen.clrtoeol()

                        # show accel and gyro on screen
                        screen.addstr(8, 50, "Accel: {}, Gyro: {}".format(accelerometer, gyroscope))
                        screen.clrtoeol()

                    elif next_msg == 'MSP_STATUS_EX':
                        ARMED = board.bit_check(board.CONFIG['mode'],0)
                        screen.addstr(5, 0, "ARMED: {}".format(ARMED), curses.A_BOLD)
                        screen.clrtoeol()

                        screen.addstr(5, 50, "armingDisableFlags: {}".format(board.process_armingDisableFlags(board.CONFIG['armingDisableFlags'])))
                        screen.clrtoeol()

                        # screen.addstr(6, 0, "cpuload: {}".format(board.CONFIG['cpuload']))
                        # screen.clrtoeol()
                        altitude = fast_read_altitude(board)
                        screen.addstr(6, 0, "lidar altitude: {}".format(altitude))
                        screen.clrtoeol()

                        screen.addstr(6, 50, "cycleTime: {}".format(board.CONFIG['cycleTime']))
                        screen.clrtoeol()

                        screen.addstr(7, 0, "mode: {}".format(board.CONFIG['mode']))
                        screen.clrtoeol()

                        screen.addstr(7, 50, "Flight Mode: {}".format(board.process_mode(board.CONFIG['mode'])))
                        screen.clrtoeol()

                    elif next_msg == 'MSP_MOTOR':
                        screen.addstr(9, 0, "Motor Values: {}".format(board.MOTOR_DATA))
                        screen.clrtoeol()

                    elif next_msg == 'MSP_RC':
                        screen.addstr(10, 0, "RC Channels Values: {}".format(board.RC['channels']))
                        screen.clrtoeol()

                    screen.addstr(11, 0, "GUI cycleTime: {0:2.2f}ms (average {1:2.2f}Hz)".format((last_cycleTime)*1000,
                                                                                                1/(sum(average_cycle)/len(average_cycle))))
                    screen.clrtoeol()

                    if autonomous:
                        if q.qsize() > 10:
                            try:
                                CMDS = q.get()
                                CMDS['throttle'] = save_throttle
                                cursor_msg = 'Autonomous - yaw: {}, pitch: {}, loop number: {}'.format(CMDS['yaw'], CMDS['pitch'], CMDS['loop'])
                                if altitude < 0.3:
                                    CMDS['pitch'] = 1500
                            except q.Empty:
                                cursor_msg = 'No messages found in queue! Loop number: {}'.format(CMDS['loop'])
                        else:
                            cursor_msg = 'Queue too short, waiting for it to fill...'

                    screen.addstr(3, 0, cursor_msg)
                    screen.clrtoeol()
                    

                end_time = time.time()
                last_cycleTime = end_time-start_time
                if (end_time-start_time)<CTRL_LOOP_TIME:
                    time.sleep(CTRL_LOOP_TIME-(end_time-start_time))
                    
                average_cycle.append(end_time-start_time)
                average_cycle.popleft()

    finally:
        screen.addstr(5, 0, "Disconnected from the FC!")
        screen.clrtoeol()
        stop(pool)

if __name__ == "__main__":
    run_curses(keyboard_controller)
