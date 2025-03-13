import multiprocessing
import time
import testing
import realsense_control
import keyboard
from sshkeyboard import listen_keyboard, stop_listening


def press(key):
    if key == "q":
        print("Q PRESSED, EXITING")
        exit_called = True

def release(key):
    return True

def producer(out_q):
    realsense_control.main(out_q, testing=False)

def consumer(in_q):
    testing.test_thing(in_q, testing=True)

try:
    # Create the shared queue and launch both threads
    manager = multiprocessing.Manager()
    q = manager.Queue()
    pool = multiprocessing.Pool()
    t1 = pool.apply_async(consumer, [q])
    t2 = pool.apply_async(producer, [q])

    time.sleep(15)
    item = "land"
    q.put(item)

    while True:
        #print(exit_called)
        if exit_called:
            item = "land"
            q.put(item)
            time.sleep(10)
            pool.terminate()
            pool.join()


except KeyboardInterrupt:
    item = "land"
    q.put(item)
    time.sleep(5)
    pool.terminate()
    pool.join()


pool.terminate()
pool.join()
exit(0)