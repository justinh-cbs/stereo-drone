import time
from itertools import cycle
import queue

from yamspy import MSPy


# serial port for flight controller
SERIAL_PORT = "/dev/ttyAMA1"


def disarmed(board, CMDS, CMDS_ORDER, cycles):
    print('Disarmed state...')
    CMDS['aux1'] = 1000
    CMDS['aux2'] = 1000
    for i in range(cycles):
        ARMED = board.bit_check(board.CONFIG['mode'],0)
        print(i, "ARMED: {}".format(ARMED))
        if board.send_RAW_RC([CMDS[ki] for ki in CMDS_ORDER]):
            dataHandler = board.receive_msg()
            board.process_recv_data(dataHandler)

def armed(board, CMDS, CMDS_ORDER, cycles):
    print('armed state...')
    CMDS['aux1'] = 1800
    CMDS['aux2'] = 1000
    for i in range(cycles):
        ARMED = board.bit_check(board.CONFIG['mode'],0)
        print(i, "ARMED: {}".format(ARMED))
        if board.send_RAW_RC([CMDS[ki] for ki in CMDS_ORDER]):
            dataHandler = board.receive_msg()
            board.process_recv_data(dataHandler)

def throttle_up(board, CMDS, CMDS_ORDER):
    # set mode to nav hold
    CMDS['aux2'] = 1800
    # test throttling up and down
    CMDS['aux1'] = 1800
    for i in range(988, 1500, 1):
        CMDS['throttle'] = i
        print('throttle up: {}'.format(CMDS['throttle']))
        if board.send_RAW_RC([CMDS[ki] for ki in CMDS_ORDER]):
            dataHandler = board.receive_msg()
            board.process_recv_data(dataHandler)

def hover(board, CMDS, CMDS_ORDER, cycles):
    # set mode to nav hold
    CMDS['aux2'] = 1800
    CMDS['aux1'] = 1800
    CMDS['throttle'] = 1550
    for i in range(cycles):
        ARMED = board.bit_check(board.CONFIG['mode'],0)
        print(i, "ARMED: {}".format(ARMED))
        if board.send_RAW_RC([CMDS[ki] for ki in CMDS_ORDER]):
            dataHandler = board.receive_msg()
            board.process_recv_data(dataHandler)


def throttle_down(board, CMDS, CMDS_ORDER):
    # set mode to nav hold
    CMDS['aux2'] = 1800
    # test throttling up and down
    CMDS['aux1'] = 1800
    for i in range(1300, 988, -1):
        CMDS['throttle'] = i
        print('throttle down: {}'.format(CMDS['throttle']))
        if board.send_RAW_RC([CMDS[ki] for ki in CMDS_ORDER]):
            dataHandler = board.receive_msg()
            board.process_recv_data(dataHandler)

def cmd_loop(board, CMDS, CMDS_ORDER):
    ARMED = board.bit_check(board.CONFIG['mode'],0)
    print("ARMED: {}".format(ARMED))
    print(CMDS)
    if board.send_RAW_RC([CMDS[ki] for ki in CMDS_ORDER]):
        dataHandler = board.receive_msg()
        board.process_recv_data(dataHandler)


def test_thing(queue, testing):

    # initial command values
    CMDS = {
        'roll':     1500,
        'pitch':    1500,
        'throttle': 900,
        'yaw':      1500,
        'aux1':     1000,
        'aux2':     1000
        }

    # this order depends on how the flight controller is configured, this order is for AETR
    CMDS_ORDER = ['roll', 'pitch', 'throttle', 'yaw', 'aux1', 'aux2']

    # It's necessary to send some messages or the RX failsafe will be activated
    # and it will not be possible to arm.
    command_list = ['MSP_API_VERSION', 'MSP_FC_VARIANT', 'MSP_FC_VERSION', 'MSP_BUILD_INFO', 
                    'MSP_BOARD_INFO', 'MSP_UID', 'MSP_ACC_TRIM', 'MSP_NAME', 'MSP_STATUS', 'MSP_STATUS_EX',
                    'MSP_BATTERY_CONFIG', 'MSP_BATTERY_STATE', 'MSP_BOXNAMES']
    try:
        if testing:
            print("testing started, getting commands from queue...")
            commands_recv = 0
            loop_count = 0
            while True:
                loop_count += 1
                if queue.empty():
                    print("queue is empty, skipping...")

                else:
                    CMDS = queue.get()
                
                if CMDS == "land":
                    print("Recieved landing command - landing")
                    break

                else:
                    CMDS = queue.get()
                    commands_recv = commands_recv + 1
                    print("commands received: ", CMDS)
                print("commands loop count: ", loop_count)

        else:
            print("Connecting to the FC...")
            with MSPy(device=SERIAL_PORT, loglevel='WARNING', baudrate=115200) as board:

                if board.INAV:
                    command_list.append('MSPV2_INAV_ANALOG')
                    command_list.append('MSP_VOLTAGE_METER_CONFIG')

                if board == 1: # an error occurred...
                    print("Returned 1, unable to connect to FC.")
                    return

                else:
                    for msg in command_list: 
                        if board.send_RAW_msg(MSPy.MSPCodes[msg], data=[]):
                            dataHandler = board.receive_msg()
                            board.process_recv_data(dataHandler)


                ARMED = board.bit_check(board.CONFIG['mode'],0)
                print("ARMED: {}".format(ARMED))
                print("armingDisableFlags: {}".format(board.process_armingDisableFlags(board.CONFIG['armingDisableFlags'])))
                print("cpuload: {}".format(board.CONFIG['cpuload']))
                print("cycleTime: {}".format(board.CONFIG['cycleTime']))
                print("mode: {}".format(board.CONFIG['mode']))
                print("Flight Mode: {}".format(board.process_mode(board.CONFIG['mode'])))

                # takeoff sequence
                # wait for realsense to start
                wait_for_realsense = True

                while wait_for_realsense:
                    if queue.empty():
                        print("queue is empty, waiting...")

                    else:
                        wait_for_realsense = False

                print("starting takeoff sequence...")
                disarmed(board, CMDS, CMDS_ORDER, cycles=200)
                armed(board, CMDS, CMDS_ORDER, cycles=300)
                throttle_up(board, CMDS, CMDS_ORDER)
                hover(board, CMDS, CMDS_ORDER, cycles=200)
                print("takeoff sequence complete!")

                # fly forward with realsense!

                try:
                    loop_count = 0
                    while True:
                        loop_count += 1
                        if queue.empty():
                            print("queue is empty, skipping...")

                        else:
                            CMDS = queue.get()
                        
                        if CMDS == "land":
                            print("Recieved landing command - landing")
                            throttle_down(board, CMDS, CMDS_ORDER)
                            armed(board, CMDS, CMDS_ORDER, cycles=100)
                            disarmed(board, CMDS, CMDS_ORDER, cycles=100)
                            break

                        print("commands received: ", CMDS)
                        print("controls loop count: ", loop_count)
                        cmd_loop(board, CMDS, CMDS_ORDER)
                    # hover loop
                    # disarmed(board, CMDS, CMDS_ORDER, cycles=200)
                    # armed(board, CMDS, CMDS_ORDER, cycles=300)
                    # throttle_up(board, CMDS, CMDS_ORDER)
                    # hover(board, CMDS, CMDS_ORDER, cycles=2000)
                    # throttle_down(board, CMDS, CMDS_ORDER)
                    # disarmed(board, CMDS, CMDS_ORDER, cycles=100)

                except KeyboardInterrupt:
                    print("KeyboardInterrupt - landing")
                    throttle_down(board, CMDS, CMDS_ORDER)
                    disarmed(board, CMDS, CMDS_ORDER, cycles=100)

    finally:
        disarmed(board, CMDS, CMDS_ORDER, cycles=100)
        print("test over!")
        exit(0)

if __name__ == "__main__":
    test_thing()

