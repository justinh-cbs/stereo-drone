import time
import multiprocessing
import queue
import cv2
import numpy as np

import rs_stream as rs_streamer
import depth_display as depth_display
import ir_display as ir_display

def rs_stream(stream_queue):
    rs_streamer.main(stream_queue)

def depth(stream_queue):
    depth_display.main(stream_queue)

def ir_stream(stream_queue):
    ir_display.main(stream_queue)

def stop(pool):
    pool.terminate()
    pool.join()
    exit(0)


manager = multiprocessing.Manager()

# make a stream_queue
stream_queue = manager.Queue()

print("starting threads...")
pool = multiprocessing.Pool()
t1 = pool.apply_async(rs_stream, [stream_queue])
time.sleep(2)
t2 = pool.apply_async(ir_stream, [stream_queue])
t3 = pool.apply_async(depth, [stream_queue])

# while True:
#     try:
#         item = stream_queue.get()
#         if not isinstance(item, np.ndarray):
#             if item == 1:
#                 print(item)
#                 depth_image = stream_queue.get()
#                 scaled_depth = cv2.convertScaleAbs(depth_image, alpha=0.08)
#                 print("got depth")
#                 depth_colormap = cv2.applyColorMap(scaled_depth, cv2.COLORMAP_JET)
#                 cv2.imshow('RealSense', depth_colormap)
#             elif item == 0:
#                 print(item)
#                 image = stream_queue.get()
#                 print("got IR image")
#                 cv2.namedWindow('IR Example', cv2.WINDOW_AUTOSIZE)
#                 cv2.imshow('IR Example', image)
#         else:
#             pass
#     except queue.Empty:
#         pass

#     key = cv2.waitKey(1)

#     # Press esc or 'q' to close the image window
#     if key & 0xFF == ord('q') or key == 27:
#         cv2.destroyAllWindows()
#         break

time.sleep(20)

item = "STOP"
stream_queue.put(item)

stop(pool)