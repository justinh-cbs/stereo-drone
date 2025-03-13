
import threading
import time as timelib

import numpy as np
import cv2
from scipy import optimize
import pyrealsense2.pyrealsense2 as rs

from .feature_stream import FeatureMessage, FeatureStream


# what you will do:
        # two image test will get split into init and process image
        # enable_stream will be in init or start
        # anything we expect to see in the loop needs to be a member in this init
        # define last_image and ir_np etc. in init, will be used in process image
        # next time... FLY

# TODO put data from here into ring buffer

class RSTracker:
    _lock = threading.Lock()
    _running = False
    _loop_dt = 0.2 #0.005
    _image_dt = 1 #0.1
    _loop_last = timelib.time()
    _image_last = _loop_last
    _sb = None
    min_tracked_features = 10

    def __init__(self, sb):
        print('Initializing RSTracker')
        self.headless = False
        self._sb = sb.getInstance()
        self._feature_stream = FeatureStream()
        self._sb.register(self._feature_stream, "Feature")
        # look at multiwii.py for example of how to publish IMU to buffer

    def start(self):
        with self._lock:
            print('RSTracker Start Running')
            # RealSense stream setup
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)
            pipeline_profile = self.pipeline.start(config)
            device = pipeline_profile.get_device()
            depth_sensor = device.query_sensors()[0]
            depth_sensor.set_option(rs.option.emitter_on_off, 1)

            # params for ShiTomasi corner detection
            self.feature_params = dict(maxCorners = 60,
                            qualityLevel = 0.5,
                            minDistance = 40,
                            blockSize = 7)
            # Parameters for lucas kanade optical flow
            self.lk_params = dict(winSize = (5, 5),
                            maxLevel = 5,
                            criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.3),)

            # initialize numpy arrays
            self.last_image = np.array([])
            self.ir_np = np.array([])
            self.scaled_depth = np.array([])
            self.p0 = []
            self.h_x = 0
            self.h_y = 0
            self.h_z = 0
            self.p0_depth = np.array([])
            self.p0_ids = np.array([])
            self.pts_diff = np.array([])
            self.ids = np.array([])

            self._running = True

    # def assign_row_numbers(df1, df2, start_number):
    #     """
    #     Function to assign row numbers to each point dataframe
    #     """
    #     last_number = df1[-1, -1] if df1.shape[1] > 1 else start_number - 1
    #     df1 = np.hstack((df1, np.arange(start_number, start_number + len(df1)).reshape(-1, 1)))
    #     df2 = np.hstack((df2, np.arange(last_number + 1, last_number + 1 + len(df2)).reshape(-1, 1)))
    #     combined_df = np.vstack((df1, df2))
    #     combined_df[:, -1] -= start_number
    #     last_added_number = combined_df[-1, -1]
    #     return combined_df, last_added_number

    def track_features(p0, p1):

        if p0 is None:
            p0 = np.zeros((13, 4))
            p0[:, 0] = np.arange(13)

        if p1 is not None:
            p0_tracked = np.zeros_like(p0)
            p0_tracked[:, :3] = p0[:, :3]
            p0_tracked[p1[:, 0].astype(int), 1:] = p1[:, 1:]
            p0 = p0_tracked

        p0[np.isnan(p0[:, 1]), 1:] = -1

        return p0


    def optical_flow(self, new_image, old_image, depth, p0):
        frame = np.array([])

        # ones_like!
        mask = 255*np.ones_like(old_image)

        if len(p0) <= self.min_tracked_features:
            for point in p0:
                mask = cv2.circle(mask, (int(point[0][0]), int(point[0][1])), 5, 0, -1)

            new_features = cv2.goodFeaturesToTrack(old_image, mask=mask, **self.feature_params)

            if type(new_features) == None:
                new_features = p0
            if len(p0) == 0:
                p0 = new_features
            else:
                try:
                    p0 = np.concatenate((p0, new_features), axis=0)
                except ValueError as err:
                    print(err)
                    p0 = new_features

            # give each new feature an id
            # increment ids starting at 0

        p1, st, err = cv2.calcOpticalFlowPyrLK(old_image, new_image, p0, None, **self.lk_params)
        # where None is we can put in next pts, get possible positions of new point from imu
        # use imu, do maths on tracked points vs imu accel, next pts will know where to look

        if p1 is not None:
            good_new = p1[st==1]
            good_old = p0[st==1]

            #new_ids = old_ids[st==1]

            # we want a dataframe with points and ids for each point
            # send that to buffer for SE

            # example: first frame, 13 features. Name them 0-12
            # return to send to SE: names, and position of each one (x, y, z)
            # make SE take either (x, y) or (x, y, z), add special value for z if it's not useful
            # second frame: 13 features to try to track so we don't add more
            # successfully track 8: [df p1 with a column for ids 0-7]
            # send to SE on second frame: the ids 0-7 and their tracked positions [p1]
            # p1 becomes p0
            # third frame: start with 8 features, have to detect more
            # detect 7 more, give them ids 13-19, add that to p0. now has 15 features
            # ids are 0-7 and 13-19
            # it tracks all but the first two, 0-1
            # p1 will have 13 features, with ids 2-7, 13-19
            # send p1 ids and position to SE (can look at imu)
            # this is missing z! have to add at a later step

        mask = np.zeros_like(old_image)
        color = np.random.randint(254, 255, (100, 3))

        for i, (new, old) in enumerate(zip(good_new, good_old)):
            # end coordinates x, y, z
            a, b = new.ravel()
            if a < 640 and b < 480:
                e = depth[int(b - 1), int(a - 1)]
                e_disp = 2*round(0.5*e)
            else:
                e = 0
            # start coordinates x, y
            c, d = old.ravel()

            if c < 640 and d < 480:
                f = depth[int(d - 1), int(c - 1)]
                f_disp = 2*round(0.5*f)
            else:
                f = 0

            self.p0_depth = np.append(self.p0_depth, (a, b, e))

            dim1 = (int(a) - int(c))
            dim2 = (int(b) - int(d))
            dim3 = (int(e) - int(f))
            self.pts_diff = np.append(self.pts_diff, (dim1, dim2, dim3))

            if not self.headless:
                mask = cv2.line(mask, (int(a), int(b)), (int(c), int(d)), color[i].tolist(), 2)
                frame = cv2.circle(old_image, (int(c), int(d)), 5, color[i].tolist(), -1)

                # write some depths at each point we're tracking
                font = cv2.FONT_HERSHEY_SIMPLEX
                coordText = (int(c), int(d))
                fontScale = 0.5
                fontColor = (255,255,255)
                thickness = 1
                lineType = 2

                cv2.putText(frame, str(e_disp),
                    coordText,
                    font,
                    fontScale,
                    fontColor,
                    thickness,
                    lineType)

            if frame.any():
                img = cv2.add(frame, mask)
                cv2.imshow('frame', img)

            k = cv2.waitKey(30) & 0xff
            if k == 27:
                return

        try:
            # Now update the previous frame and previous points
            self.p0 = good_new.reshape(-1, 1, 2)
            print("x diff array: ", self.pts_diff[0])
            avg_0 = np.average(self.pts_diff[0])
            avg_1 = np.average(self.pts_diff[1])
            avg_2 = np.average(self.pts_diff[2])

            pts_diff_avg = [avg_0, avg_1, avg_2]
            print("avg diff xyz: ", pts_diff_avg)
            return self.p0, pts_diff_avg, self.p0_depth

        except UnboundLocalError as err:
            dims = [0, 0, 0]
            pts_diff_avg = dims
            print("no dims found, setting to zero: ", dims)
            return self.p0, pts_diff_avg, self.p0_depth

        except IndexError as err:
            dims = [0, 0, 0]
            pts_diff_avg = dims
            print("pts_diff empty! setting to zero: ", dims)
            return self.p0, pts_diff_avg, self.p0_depth

    def stop(self):
        with self._lock:
            print('RSTracker Stop Running')
            self.pipeline.stop()
            self._running = False

    def process_image(self):
        try:
            # get frames from RealSense pipeline
            frames = self.pipeline.wait_for_frames()
            emitter = rs.depth_frame.get_frame_metadata(
                frames,
                rs.frame_metadata_value.frame_laser_power_mode,
                )

            # grab the next ones immediately and sort
            # depth needs emitter, IR needs no emitter
            frames_2 = self.pipeline.wait_for_frames()
            #ir_frame_2 = frames_2.get_infrared_frame(1)

            if not self.headless:
                frame_number = frames.get_frame_number()
                print("frame: ", frame_number, "emitter: ", emitter)
                frame_number_2 = frames_2.get_frame_number()
                print("** IR frame: ", frame_number_2)

            if emitter:
                depth = frames.get_depth_frame()
                if not depth:
                    return

                ir_frame = frames_2.get_infrared_frame(1)
                self.last_image = self.ir_np
                self.ir_np = np.asanyarray(ir_frame.get_data())

            else:
                ir_frame = frames.get_infrared_frame(1)
                self.last_image = self.ir_np
                depth = frames_2.get_depth_frame()
                self.ir_np = np.asanyarray(ir_frame.get_data())

            depth_image = np.asanyarray(depth.get_data())
            self.scaled_depth = cv2.convertScaleAbs(depth_image, alpha=0.08)

            if np.any(self.last_image) and np.any(self.scaled_depth):
                self.last_image = np.asanyarray(self.last_image)
                self.scaled_depth = np.asanyarray(self.scaled_depth)
                self.p0, pts_diff_avg, self.p0_depth = self.optical_flow(
                    self.ir_np,
                    self.last_image,
                    self.scaled_depth,
                    self.p0,
                    )

                pts_df = np.concatenate((self.p0, self.p0_depth, self.p0_ids), 1)
                msg = FeatureMessage(timelib.time(), pts_df)
                self._feature_stream.addOne(msg)
                # above is where we should publish to buffer
                # make a new dataframe with columns p0, p0_depth, ids

                # will need tov in relation to imu
                # subscribe to imu, whenever we get imu we update a local tov
                # from the imu_messages tov
                # when we process, use that tov for image

                # add flow dims to show heading
                self.h_x = (self.h_x + pts_diff_avg[0])
                self.h_y = (self.h_y + pts_diff_avg[1])
                self.h_z = (self.h_z + pts_diff_avg[2])
                headings = [self.h_x, self.h_y, self.h_z]

                # TODO not printing, make everything self.?
                if not self.headless:
                    print("opt_flow: ", pts_diff_avg)
                    print("headings xyz: ", headings)

        finally:
            if not self._running:
                self.pipeline.stop()


    def loop(self):
        while True:
            with self._lock:
                if self._running:
                    self._loop_last = timelib.time()

                    if self._loop_last > (self._image_last + self._image_dt):
                        self._image_last = self._loop_last
                        self.process_image()

                    if timelib.time() < (self._loop_last + self._loop_dt):
                        timelib.sleep(self._loop_last + self._loop_dt - timelib.time())
                    else:
                        print('RSTracker loop took too long')
                else:
                    print('Exiting RSTracker loop')
                    break
