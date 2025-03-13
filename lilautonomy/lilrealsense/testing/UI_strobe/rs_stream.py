import numpy as np
import cv2
import pyrealsense2.pyrealsense2 as rs


def main(queue):
    # start streaming
    print("starting rs stream...")
    pipeline = rs.pipeline()

    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)

    pipeline_profile = pipeline.start(config)
    device = pipeline_profile.get_device()

    depth_sensor = device.query_sensors()[0]
    depth_sensor.set_option(rs.option.emitter_on_off, 1)

    try:
        while True:
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()

            frame_number = frames.get_frame_number()
            print(frame_number)
            emitter = rs.depth_frame.get_frame_metadata(depth_frame, rs.frame_metadata_value.frame_laser_power_mode)

            if emitter:
                depth_image = np.asanyarray(depth_frame.get_data())
                queue.put(emitter)
                queue.put(depth_image)

            else:
                ir1_frame = frames.get_infrared_frame(1) # Left IR Camera, it allows 1, 2 or no input
                ir_image = np.asanyarray(ir1_frame.get_data())
                queue.put(emitter)
                queue.put(ir_image)

            if queue.qsize() > 20:
                try:
                    item = queue.get()
                    if item == "STOP":
                        print("stopping rs stream")
                        break
                except queue.Empty:
                    pass


    finally:
        pipeline.stop()