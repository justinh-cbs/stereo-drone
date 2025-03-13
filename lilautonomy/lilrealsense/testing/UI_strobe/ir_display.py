import cv2
import numpy as np
import queue

def main(stream_queue):
    print("starting IR viz...")
    while True:
        try:
            item = stream_queue.get()
            if not isinstance(item, np.ndarray):
                if item == 1:
                    print("putting back: ", item)
                    stream_queue.put(item)
                    image = stream_queue.get()
                    stream_queue.put(image)                    
                elif item == 0:
                    print(item)
                    image = stream_queue.get()
                    print("got IR image")
                    cv2.namedWindow('IR Example', cv2.WINDOW_AUTOSIZE)
                    cv2.imshow('IR Example', image)
            else:
                pass
        except queue.Empty:
            pass

        key = cv2.waitKey(1)

        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
