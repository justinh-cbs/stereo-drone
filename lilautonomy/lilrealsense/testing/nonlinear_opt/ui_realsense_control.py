import time
import pyrealsense2.pyrealsense2 as rs
import numpy as np
import cv2
import argparse

# set up command line arguments
parser = argparse.ArgumentParser()
# parser.add_argument("--display_image", help="stream realsense images to display",
#                     action="store_true")
parser.add_argument("--drive", help="engage motion",
                    action="store_true")
args = parser.parse_args()

# display_image = args.display_image
display_image = True
drive = args.drive

CMDS = {
        "roll":     1500,
        "pitch":    1500,
        "throttle": 900,
        "yaw":      1500,
        "aux1":     1000,
        "aux2":     1000
        }

def nothing(args):
    pass

def detectBlobs(mask):
    # Set up the SimpleBlobdetector with default parameters.
    params = cv2.SimpleBlobDetector_Params()
         
    # Change thresholds
    params.minThreshold = 1
    params.maxThreshold = 255
    
    # Filter by Area.
    params.filterByArea = True
    params.maxArea = 2000000
    params.minArea = 1000
    
    # Filter by Circularity
    params.filterByCircularity = False
    params.minCircularity = 0.01
    
    # Filter by Convexity
    params.filterByConvexity = False
    params.minConvexity = 0.05
    
    # Filter by Inertia
    params.filterByInertia = False
    params.minInertiaRatio = 0.01

    detector = cv2.SimpleBlobDetector_create(params)
 
    # Detect blobs.
        # make a border so we detect blobs stuck on the side of the image
    mask = cv2.copyMakeBorder(mask, top=1, bottom=1, left=1, right=1, borderType=cv2.BORDER_CONSTANT, value=[255,255,255])
    reversemask = cv2.bitwise_not(mask)
    keypoints = detector.detect(reversemask)
    pts = [key_point.pt for key_point in keypoints]
    im_with_keypoints = cv2.drawKeypoints(mask, keypoints, np.array([]),
            (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    return im_with_keypoints, pts

def fourSectionAvgColor(image):
    rows, cols, ch = image.shape
    colsMid = int(cols/2)
    colsQuart = int(cols/4)
    cols3Quart = int((cols/4) * 3)

    # sections go from L to R
    section0 = image[0:rows, 0:colsQuart]
    section1 = image[0:rows, colsQuart:colsMid]
    section2 = image[0:rows, colsMid:cols3Quart]
    section3 = image[0:rows, cols3Quart:cols]
    sectionsList = [section0, section1, section2, section3]

    sectionAvgColorList = []
    for i in sectionsList:
        pixelSum = 0
        yRows, xCols, chs = i.shape
        pixelCount = yRows*xCols
        totRed = 0
        totBlue = 0
        totGreen = 0
        for x in range(xCols):
            for y in range(yRows):
                bgr = i[y,x]
                b = bgr[0]
                g = bgr[1]
                r = bgr[2]
                totBlue = totBlue+b
                totGreen = totGreen+g
                totRed = totRed+r

        avgBlue = int(totBlue/pixelCount)
        avgGreen = int(totGreen/pixelCount)
        avgRed = int(totRed/pixelCount)
        avgPixel = (avgBlue, avgGreen, avgRed)
        sectionAvgColorList.append(avgPixel)
    return sectionAvgColorList

def thresholdDepth(depth):
    depth[depth==0] = 255 #set all invalid depth pixels to 255
    threshold_value = cv2.getTrackbarPos('Threshold','Truncated Depth')
    ret,truncated_depth = cv2.threshold(depth, threshold_value, 255, cv2.THRESH_BINARY_INV) # Zero if dist>TH
    return truncated_depth

if display_image:
    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow('Truncated Depth', cv2.WINDOW_AUTOSIZE)
    cv2.createTrackbar('Threshold','Truncated Depth',40,255,nothing)


def main(queue, testing):
    if testing:
        CMDS = {
        "roll":     1500,
        "pitch":    1500,
        "throttle": 1500,
        "yaw":      1500,
        "aux1":     1800,
        "aux2":     1800
        }

        roll_cmd = 1000

        while True:
            if CMDS["roll"] < 1900:
                roll_cmd = roll_cmd + 1
            else:
                roll_cmd = roll_cmd - 1

            CMDS["roll"] = roll_cmd

            queue.put(CMDS)
            #print("commmands sent: ", CMDS)
            time.sleep(0.1)

    else:
        try:
            # Create a context object. This object owns the handles to all connected realsense devices
            #print("creating RS pipeline...")
            pipeline = rs.pipeline()

            # Configure streams
            #print("configure streams...")
            config = rs.config()
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            # try fast stream, narrow vertical FOV
            #print("enable fast stream...")
            # config.enable_stream(rs.stream.depth, 0, 848, 100, rs.format.z16, 300)
            #config.enable_stream(rs.stream.depth)

            # Start streaming
            #print("start streaming...")
            pipeline.start(config)

            # y-values to detect distances between
            detection_lower = 430
            detection_upper = 800

            det_left = 200
            det_right = 400

            loop_number = 0

            while True:
                # This call waits until a new coherent set of frames is available on a device
                # Calls to get_frame_data(...) and get_frame_timestamp(...) on a device will return stable values until wait_for_frames(...) is called
                loop_number = loop_number + 1
                #print("realsense loop number: ", loop_number)

                if loop_number >= 1000:
                    break

                #print("getting frame data...")
                frames = pipeline.wait_for_frames()
                depth = frames.get_depth_frame()

                if not depth: continue

                depth_image = np.asanyarray(depth.get_data())
                scaled_depth = cv2.convertScaleAbs(depth_image, alpha=0.08)

                truncated_depth = thresholdDepth(scaled_depth)
                truncated_depth, points = detectBlobs(truncated_depth)

                # drive by avoiding objects logic
                # example detections: [(489.6097106933594, 442.31201171875), (364.44451904296875, 405.34393310546875), (376.8372497558594, 278.6385803222656), (132.40438842773438, 347.2413024902344)]
                # (x, y)
                detections = fourSectionAvgColor(truncated_depth)

                if display_image:
                    #print("showing image...")
                    depth_colormap = cv2.applyColorMap(scaled_depth, cv2.COLORMAP_JET)
                    cv2.imshow('RealSense', depth_colormap)
                    cv2.imshow('Truncated Depth', truncated_depth)

                if (sum(detections[1]) <= 20 and sum(detections[2]) <= 20):
                    # drive_func(drive, "0")
                    mode = "MODE: FORWARD"
                    CMDS = {
                            "roll":     1500,
                            "pitch":    1550,
                            "throttle": 1500,
                            "yaw":      1500,
                            "aux1":     1800,
                            "aux2":     1800,
                            "aux3":     1900,
                            "loop":     0
                            }

                elif sum(detections[1]) >= 20:
                    #drive_func(drive, "1")
                    mode = "MODE: FLY RIGHT"
                    CMDS = {
                            "roll":     1500,
                            "pitch":    1500,
                            "throttle": 1500,
                            "yaw":      1550,
                            "aux1":     1800,
                            "aux2":     1800,
                            "aux3":     1900,
                            "loop":     0
                            }

                elif sum(detections[2]) >= 20:
                    #drive_func(drive, "2")
                    mode = "MODE: FLY LEFT"
                    CMDS = {
                            "roll":     1500,
                            "pitch":    1500,
                            "throttle": 1500,
                            "yaw":      1450,
                            "aux1":     1800,
                            "aux2":     1800,
                            "aux3":     1900,
                            "loop":     0
                            }

                elif (sum(detections[1]) >= 20 and sum(detections[2]) >= 20):
                    #drive_func(drive, "4")
                    mode = "MODE: STOPPED"
                    CMDS = {
                            "roll":     1500,
                            "pitch":    1500,
                            "throttle": 1500,
                            "yaw":      1500,
                            "aux1":     1800,
                            "aux2":     1800,
                            "aux3":     1900,
                            "loop":     0
                            }
                
                CMDS["loop"] = loop_number

                queue.put(CMDS)

                if queue.qsize() > 20:
                    try:
                        CMDS = queue.get()
                    except queue.Empty:
                        pass

                #print("commmands sent: ", CMDS)
                #print(mode)

                k = cv2.waitKey(1) & 0xFF
                if k == 27:
                    break

            # done with loop 
            #print("finished loop, landing!")
            CMDS = "land"
            mode = "MODE: STOPPED"
            #print("commmands sent: ", CMDS)
            #print(mode)

            queue.put(CMDS)
            pipeline.stop()
            exit(0)

        except KeyboardInterrupt:
            CMDS = "land"

            queue.put(CMDS)
            pipeline.stop()
            exit(0)

        except Exception as e:
            #print(e)
            pass

if __name__ =='__main__':
    main()