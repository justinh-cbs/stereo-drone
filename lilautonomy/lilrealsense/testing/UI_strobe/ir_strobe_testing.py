import numpy as np
import cv2
import pyrealsense2.pyrealsense2 as rs




pipeline = rs.pipeline()

config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)

pipeline_profile = pipeline.start(config)
device = pipeline_profile.get_device()

depth_sensor = device.query_sensors()[0]
emitter = depth_sensor.get_option(rs.option.emitter_enabled)

depth_sensor.set_option(rs.option.emitter_on_off, 1)

try:
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()

        frame_number = frames.get_frame_number()
        print(frame_number)
        emitter = rs.depth_frame.get_frame_metadata(depth_frame, rs.frame_metadata_value.frame_laser_power_mode)
        print("Emitter : ", emitter)
        laser_power = depth_sensor.get_option(rs.option.laser_power)
        print("laser_power: ", laser_power)

        if emitter:
            depth = frames.get_depth_frame()
            if not depth:
                continue

            depth_image = np.asanyarray(depth.get_data())
            scaled_depth = cv2.convertScaleAbs(depth_image, alpha=0.08)

            depth_colormap = cv2.applyColorMap(scaled_depth, cv2.COLORMAP_JET)
            cv2.imshow('RealSense', depth_colormap)

        else:
            # set_laser = 10
            # depth_sensor.set_option(rs.option.laser_power, set_laser)
            # emitter = depth_sensor.get_option(rs.option.laser_power)
            # print("laser power level = ", emitter)
            print("showing IR")

            ir1_frame = frames.get_infrared_frame(1) # Left IR Camera, it allows 1, 2 or no input
            image = np.asanyarray(ir1_frame.get_data())
            cv2.namedWindow('IR Example', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('IR Example', image)


        key = cv2.waitKey(1)

        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break

finally:
    pipeline.stop()
