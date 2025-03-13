import cv2
import numpy as np
import queue

def main(stream_queue):
    print("starting depth viz...")
    while True:
        try:
            item = stream_queue.get()
            if not isinstance(item, np.ndarray):
                if item == 1:
                    print(item)
                    depth_image = stream_queue.get()
                    scaled_depth = cv2.convertScaleAbs(depth_image, alpha=0.08)
                    print("got depth")
                    depth_colormap = cv2.applyColorMap(scaled_depth, cv2.COLORMAP_JET)
                    cv2.imshow('RealSense', depth_colormap)
                elif item == 0:
                    print("putting back: ", item)
                    stream_queue.put(item)
                    image = stream_queue.get()
                    stream_queue.put(image)
            else:
                pass
        except queue.Empty:
            pass

        key = cv2.waitKey(1)

        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
