import cv2
from operator import itemgetter
import numpy as np
from settings import *
import matplotlib.pyplot as plt
import math
import glob
import pymmcore
import time
import serial
import tektronix_func_gen as tfg
from A_Star import Astar, generate_bitmap, generate_lines
from cluster_detection_and_tracking import find_top_n_indices, find_clusters, TrackClusters
from model import update_q_values, calc_action
from environment_pipeline_S import VideoStreamHammamatsu, ActuatorPiezos, FunctionGenerator


""" IMAGE PROCESSING """


EXPOSURE_TIME = 30 # Exposure time Hammamatsu
IMG_SIZE = 300
size = (IMG_SIZE, IMG_SIZE)
counter = 1
mmc = pymmcore.CMMCore() # Initialize core object
i = 0
function_generator = FunctionGenerator()  # Function generator

# Find camera config --> get camera
mmc.setDeviceAdapterSearchPaths(["C:/Program Files/Micro-Manager-2.0gamma"])
mmc.loadSystemConfiguration('C:/Program Files/Micro-Manager-2.0gamma/mmHamamatsu.cfg')
label = mmc.getCameraDevice()
mmc.setExposure(EXPOSURE_TIME)
mmc.getLoadedDevices()
mmc.snapImage()
img = mmc.getImage()  # img - it's just numpy array

mmc.startContinuousSequenceAcquisition(1)
img = (cv2.resize(img, size) / 256).astype("uint8")
# cv2.imshow('sd', img)
# cv2.waitKey(0)

step = 1
now = round(time.time(), 3)
# filename = SNAPSHOTS_SAVE_DIR + f"{now}-reset.png"

state_buf = []
size_buf = []
_, _, bbox = find_clusters(image=img, amount_of_clusters=1, verbose=False)
print(bbox)
bbox = bbox[0]

# Initialize tracking algorithm
tracker = TrackClusters(bbox= bbox)

# Initialize Vpp and frequency
vpp = 7
frequency = 2000  # kHz
action = -1
function_generator.reset(vpp=vpp, frequency=frequency)

# Get the centroid of (biggest) swarm
state, size_swarm = tracker.reset(img=img)
print(state)
state_buf.append(state)
size_buf.append(size_swarm)

# A-STAR
actuator = ActuatorPiezos()
astar = Astar()

start = tuple(state_buf[0])
print(start)
goal = tuple([50, 50])
bitmap = generate_bitmap(300, 1)
lines_dict = generate_lines(bitmap, 30, start, goal)

path_A = astar.path(start, goal, lines_dict=lines_dict)

if path_A is not None:
    path_img = bitmap.copy()
    for i in range(len(path_A) - 1):
        cv2.line(path_img, path_A[i], path_A[i + 1], (0, 0, 255), 2)
    cv2.imshow('Path', path_img)
    cv2.waitKey(0)

start_time = time.time()
new_action = 1
target = [50,50]
path_A.remove(path_A[0])

while True:
    if mmc.getRemainingImageCount() > 0:
        img = mmc.getLastImage()
        img = (cv2.resize(img, size) / 256).astype("uint8")
        # cv2.imshow('Video', img)
        step += 1

    if not step % 100:
        _, _, bbox = find_clusters(image=img, amount_of_clusters=1, verbose=False)
        bbox = bbox[0]
        # Initialize tracking algorithm
        tracker = TrackClusters(bbox=bbox)
        state, size_swarm = tracker.reset(img=img)

    current_time = time.time() - start_time
    state, size_swarm = tracker.update(img=img, action=new_action, target=target, current_time=current_time, step=step, path_test=path_A, verbose=True)
    state_buf.append(state)
    size_buf.append(size_swarm)

    target = path_A[0]
    offset = np.array(state) - np.array(target)
    new_action = calc_action(pos0=state, offset=offset, q_values=Q_VALUES_INITIAL, mode='naive')
    function_generator.set_frequency(frequency=PIEZO_RESONANCES[new_action])
    if not step % 5:
        actuator.move(new_action)

    if np.linalg.norm(offset)<10:
        if len(path_A)>1:
            path_A.remove(path_A[0])
        else:
            function_generator.turn_off()
            actuator.close(5)
            break

    print(new_action)
    # cv2.waitKey(0)
    # Exit if ESC pressed
    k = cv2.waitKey(1) & 0xff
    if k == 27:
        function_generator.turn_off()
        actuator.close(5)
        break

cv2.destroyAllWindows()
mmc.stopSequenceAcquisition()
mmc.reset()

actuator.close(5)
