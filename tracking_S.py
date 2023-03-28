import cv2
from operator import itemgetter
import numpy as np
from Settings import *
import matplotlib.pyplot as plt
import math
import glob
import pymmcore
import time
import serial


""" DETECTION AND TRACKING """


# Find the indices of the top n values from a list or array quickly
def find_top_n_indices(data, top):
    indexed = enumerate(data)  # create pairs [(0, v1), (1, v2)...]
    sorted_data = sorted(indexed,
                         key=itemgetter(1),   # sort pairs by value
                         reverse=True)       # in reversed order
    return [d[0] for d in sorted_data[:top]]  # take first N indices


# Find n largest clusters using thresholding, canny edge detection and contour finding from OpenCV
def find_clusters(image, amount_of_clusters, verbose=True):
    """
    Detect clusters based on blur, thresholding and canny edge detection
    :param image:               Working image
    :param amount_of_clusters:  Number of clusters to detect (algorithm detects the #amount_of_clusters biggest ones)
    :param verbose:             Plotting True or False
    :return:                    Centroids, areas, bboxes of clusters
    """
    # Exception handling
    if not image.any():
        raise ValueError('Empty image received')

    # Check if image is grayscale
    # assert len(image.shape) == 2, "Image must be grayscale"

    # Using cv2.blur() method
    cleared_image = cv2.blur(cv2.threshold(image, 20, 255, cv2.THRESH_BINARY)[1], (2, 2))  # TODO --> Automatic threshold settings

    # Separate clusters from background and convert background to black
    canny = cv2.Canny(cleared_image, threshold1=0, threshold2=0)

    # Find contours
    contours, _ = cv2.findContours(canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Exception handling
    if not contours:
        raise ValueError('No contours detected')

    # Locate n biggest contours
    biggest_contours = find_top_n_indices([cv2.contourArea(con) for con in contours],
                                          top=amount_of_clusters)

    # Locate the centroid of each contour
    centroids = []
    areas = []
    bboxes = []

    # Find the features of contours
    for n in biggest_contours:

        # Calculate centroid moment and area
        M = cv2.moments(contours[n])
        cX = int(M["m10"] / (M["m00"] + 1e-8))
        cY = int(M["m01"] / (M["m00"] + 1e-8))
        area = cv2.contourArea(contours[n])

        if area <= 1:
            continue

        # Add features to respective lists
        centroids.append((cX, cY))
        areas.append(area)
        squared_area = np.sqrt(area)
        bboxes.append([int(cX-squared_area),
                       int(cY-squared_area),
                       int(2*squared_area),
                       int(2*squared_area)])
    # Draw results
    if verbose:

        img = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)

        for n in range(len(centroids)):
            cv2.drawContours(img, contours, n, (0, 0, 255), 1)
            cv2.circle(img, centroids[n], 0, (255, 0, 0), 5)
            cv2.rectangle(img,
                          pt1=(int(bboxes[n][0]), int(bboxes[n][1])),
                          pt2=(int(bboxes[n][0] + bboxes[n][2]), int(bboxes[n][1] + bboxes[n][3])),
                          color=(255, 255, 0))

        # Display result
        cv2.imshow("Tracking", img)
        # Exit if ESC pressed
        k = cv2.waitKey(1) & 0xff
        if k == 27:
            return

    return centroids, areas, bboxes

class TrackClusters:

    def __init__(self, bbox=None):
        self.bbox = bbox

    def reset(self, img):
        """
        Initialize tracker based on first image and initial swarm coordinate
        :param img: Working image
        :return:    Center and size of cluster
        """
        # Check if we specified a bounding box to start with, otherwise select largest cluster
        if not self.bbox:
            _, _, self.bbox = find_clusters(image=img, amount_of_clusters=1, verbose=False)
        if not self.bbox:
            self.bbox = (0, 0, IMG_SIZE, IMG_SIZE)

        # Define tracker and initialise
        self.tracker = cv2.TrackerCSRT_create()  # Very accurate, dynamic sizing, not the fastest, still okay
        # self.tracker = cv2.legacy_TrackerMedianFlow.create()  # Very fast, dynamic sizing, medium accuracy

        self.ok = self.tracker.init(img, self.bbox)

        # Calculate center of bounding box
        self.center = [int(self.bbox[0] + 0.5 * self.bbox[2]), int(self.bbox[1] + 0.5 * self.bbox[3])]

        return self.center, np.mean((self.bbox[2], self.bbox[3]))

    def update(self, img, verbose: bool=False):
        """
        Track cluster based on previous and current position
        :param img:     Working image
        :param target:  Target point (for verbose purposes)
        :param action:  Piezo actuation (for verbose purposes)
        :param verbose: Plotting
        :return:        Center and size of cluster
        """
        # Perform tracker update and calculate new center
        try:
            self.ok, self.bbox = self.tracker.update(img)
        except:
            raise ValueError('Swarm could not be tracked')
        self.bbox = list(self.bbox)
        self.center = [int(self.bbox[0] + 0.5 * self.bbox[2]), int(self.bbox[1] + 0.5 * self.bbox[3])]


        # Draw results
        if verbose:

            img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)

            # Tracking success
            p1 = (int(self.bbox[0]), int(self.bbox[1]))
            p2 = (int(self.bbox[0] + self.bbox[2]), int(self.bbox[1] + self.bbox[3]))
            cv2.rectangle(img, p1, p2, (255, 102, 102))
            # cv2.circle(img, target, 0, (178, 255, 102), 5)

            """   
            # Draw green line on the side which the piezo was actuated
            if action == 0:
                cv2.line(img, (IMG_SIZE - 2, IMG_SIZE), (298, 0), (153, 153, 255), 4)
            elif action == 2:
                cv2.line(img, (2, IMG_SIZE), (2, 0), (153, 153, 255), 4)
            elif action == 1:
                cv2.line(img, (0, IMG_SIZE - 2), (IMG_SIZE, 298), (153, 153, 255), 4)
            elif action == 3:
                cv2.line(img, (0, 2), (IMG_SIZE, 2), (153, 153, 255), 4)
            """

            # Display image
            cv2.imshow("Tracking", img)

            # Exit if ESC pressed
            k = cv2.waitKey(1) & 0xff
            if k == 27:
                return

        return self.center, np.mean((self.bbox[2], self.bbox[3]))


""" CAMERA ACQUISITION """


class VideoStreamHammamatsu:

    def __init__(self):

        # Initialiye core object
        self.core = pymmcore.CMMCore()

        # Find camera config --> get camera
        self.core.setDeviceAdapterSearchPaths(["C:/Program Files/Micro-Manager-2.0gamma"])
        self.core.loadSystemConfiguration('C:/Program Files/Micro-Manager-2.0gamma/mmHamamatsu.cfg')
        self.label = self.core.getCameraDevice()

        # Set exposure time
        self.core.setExposure(EXPOSURE_TIME)

        # Prepare acquisition
        self.core.prepareSequenceAcquisition(self.label)
        self.core.startContinuousSequenceAcquisition(1)
        self.core.initializeCircularBuffer()
        time.sleep(1)

    def snap(self, f_name, size=(IMG_SIZE, IMG_SIZE)):

        # Error handling (sometimes the video buffer is empty if we take super fast images)
        img = None
        while not np.any(img):
            try:
                # Get image
                img = self.core.getLastImage()
            except:
                pass

        # Resize image
        img = (cv2.resize(img, size) / 256).astype("uint8")

        # Save image
        cv2.imwrite(f_name, img)

        # Return image
        return img


""" MICROCONTROLLER """


class ActuatorPiezos:

    def __init__(self):

        # Initiate contact with arduino
        self.arduino = serial.Serial(port=SERIAL_PORT_ARDUINO, baudrate=BAUDRATE_ARDUINO)
        print(f"Arduino: {self.arduino.readline().decode()}")
        time.sleep(1)  # Give serial communication time to establish

        # Initiate status of 4 piëzo transducers
        self.arduino.write(b"9")  # Turn all outputs to LOW

    def move(self, action: int):

        if action == -1:
            return

        self.arduino.write(b"9")  # Turn old piezo off
        self.arduino.write(f"{action}".encode())  # Turn new piezo on

    def close(self):

        self.arduino.write(b"9")



""" IMAGE PROCESSING """

EXPOSURE_TIME = 33 # Exposure time Hammamatsu
IMG_SIZE = 300
size = (IMG_SIZE, IMG_SIZE)
counter = 1
# Initialize core object
mmc = pymmcore.CMMCore()
i = 0
# Find camera config --> get camera
mmc.setDeviceAdapterSearchPaths(["C:/Program Files/Micro-Manager-2.0gamma"])
mmc.loadSystemConfiguration('C:/Program Files/Micro-Manager-2.0gamma/mmHamamatsu.cfg')
label = mmc.getCameraDevice()
mmc.setExposure(EXPOSURE_TIME)
mmc.getLoadedDevices()
mmc.snapImage()
img = mmc.getImage()  # img - it's just numpy array
# cv2.namedWindow('Video')

mmc.startContinuousSequenceAcquisition(1)
img = (cv2.resize(img, size) / 256).astype("uint8")

print(np.shape(img))
state_buf = []
size_buf = []

# Data from Video1

# path="C:/Users/ARSL/Desktop/temp/*"
# cap = cv2.VideoCapture("C:/Users/ARSL/Desktop/Temp/Video1.AVI")
# ret, img = cap.read()
# img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Initialize devices (other way using classes)

# source = VideoStreamHammamatsu()  # Camera
# actuator = ActuatorPiezos()  # Piezo's

# Set env steps to 0
step = 0
# Get time
now = round(time.time(), 3)
# Define file name
filename = SNAPSHOTS_SAVE_DIR + f"{now}-reset.png"
# Snap a frame from the video stream and save
# img = source.snap(f_name=filename)

_, _, bbox = find_clusters(image=img, amount_of_clusters=1, verbose=False)
bbox = bbox[0]

# Initialize tracking algorithm
tracker = TrackClusters(bbox = bbox)

# Get the centroid of (biggest) swarm
state, size_swarm = tracker.reset(img=img)
state_buf.append(state)
size_buf.append(size_swarm)

counter = 1

while True:

    if mmc.getRemainingImageCount() > 0:
        img = mmc.getLastImage()
        img = (cv2.resize(img, size) / 256).astype("uint8")
        cv2.imshow('Video', img)

    if not counter % 50:
        _, _, bbox = find_clusters(image=img, amount_of_clusters=1, verbose=False)
        bbox = bbox[0]

        # Initialize tracking algorithm
        tracker = TrackClusters(bbox=bbox)
        state, size_swarm = tracker.reset(img=img)

    # img = source.snap(f_name=filename)
    # img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Data from Video1
    # _, img = cap.read()
    # img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    state, size_swarm =tracker.update(img=img, verbose=True)
    state_buf.append(state)
    size_buf.append(size_swarm)

    counter += 1

    key = cv2.waitKey(1)
    if key == 27:
        break
























"""
# frame_rate=50
class EuclideanDistTracker:
    def __init__(self):
        # Store the center positions of the objects
        self.center_points = {}
        # Keep the count of the IDs
        # each time a new object id detected, the count will increase by one
        self.id_count = 0

    def update(self, objects_rect):
        # Objects boxes and ids
        objects_bbs_ids = []

        # Get center point of new object
        for rect in objects_rect:
            x, y, w, h = rect
            cx = (x + x + w) // 2
            cy = (y + y + h) // 2
            # print(hello)
            # Find out if that object was detected already
            same_object_detected = False
            for id, pt in self.center_points.items():
                dist = math.hypot(cx - pt[0], cy - pt[1])
                # v=dis/(count/frame_rate)
                # v_x=v*cos(theta)
                # v_=v*sin(theta)
                # print(hello)
                print("the distance is",dist)
                if dist <45:
                    self.center_points[id] = (cx, cy)
                    print(self.center_points)
                    objects_bbs_ids.append([x, y, w, h, id])
                    same_object_detected = True
                    break

            # New object is detected we assign the ID to that object
            if same_object_detected is False:
                self.center_points[self.id_count] = (cx, cy)
                objects_bbs_ids.append([x, y, w, h, self.id_count])
                self.id_count += 1

        # Clean the dictionary by center points to remove IDS not used anymore
        new_center_points = {}
        for obj_bb_id in objects_bbs_ids:
            _, _, _, _, object_id = obj_bb_id
            center = self.center_points[object_id]
            new_center_points[object_id] = center

        # Update dictionary with IDs not used removed
        self.center_points = new_center_points.copy()
        return objects_bbs_ids

# Create tracker object
tracker = EuclideanDistTracker()

# path="C:/Users/ARSL/Desktop/temp/*"
cap = cv2.VideoCapture("C:/Users/ARSL/Desktop/temp/Video1.AVI")
for file in glob.glob(path):

    frame = cv2.imread(file)

    # 1. Object Detection
    mask = object_detector.apply(frame)
    _, mask = cv2.threshold(mask, 230, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    detections = []
    center_points_cur_frame = []

    for cnt in contours:

        area = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(area)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        detections.append([x, y, w, h])

    # 2. Object Tracking
    boxes_ids = tracker.update(detections)
    for box_id in boxes_ids:
        x, y, w, h, id = box_id
        cv2.putText(frame, str(id), (x, y - 15), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
        cx = int((x + x + w) / 2)
        cy = int((y + y + h) / 2)
        center_points_cur_frame.append((cx, cy))
        cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

    key = cv2.waitKey(1)
    if key == 27:
        break
"""