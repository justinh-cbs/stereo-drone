import multiprocessing
import time
import ui_realsense_control
import simpleUI_realsense

# Threading control for our adapted simpleUI

def release(key):
    return True

def producer(out_q):
    ui_realsense_control.main(out_q, testing=False)

def simple_ui(in_q):
    simpleUI_realsense.run_curses(simpleUI_realsense.keyboard_controller, in_q)

def stop(pool):
    pool.terminate()
    pool.join()
    exit(0)

try:
    # Create the shared queue and launch both threads
    manager = multiprocessing.Manager()
    q = manager.Queue()
    pool = multiprocessing.Pool()
    t1 = pool.apply_async(simple_ui, [q])
    t2 = pool.apply_async(producer, [q])

    # time.sleep(15)
    # item = "land"
    # q.put(item)


except KeyboardInterrupt:
    item = "land"
    q.put(item)
    time.sleep(5)
    pool.terminate()
    pool.join()


# pool.terminate()
# pool.join()
# exit(0)