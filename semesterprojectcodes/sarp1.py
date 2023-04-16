import time
from numpy import int0
import threading

import serial
import csv
import os
import cv2
import pyvisa as visa
import tektronix_func_gen as tfg
# import leica
from pymata4 import pymata4
from hamamatsu.dcam import dcam, Stream, copy_frame
import keyboard

print("Manipulating micro-objects in live animals")

# Tektronix settings
_VISA_ADDRESS = "USB0::0x0699::0x034F::C020081::INSTR"  # Visa address of the Tektronix function generator
amp = 2  # initial amplitude
freq = 4100000  # initial frequency
f_step_small = 500  # Steps between for frequency change
f_step_big = 50000
a_step_small = 0.1  # Steps between for amplitude change
a_step_big = 1

# Hamamatsu Settings
os.makedirs('C:/Users/ARSL/Desktop/Mahmoud/experiment1')  # create a folder named experiment
# cam=leica.VideoStreamHamamatsu()
EXPOSURE_TIME = 25  # Exposure time Hammamatsu
IMG_SIZE = 1000  # Image size
counter = 1  # counter for frames

with open('C:/Users/ARSL/Desktop/Yahya/experiment1/actuation.csv', 'w', newline='') as f:
    fieldnames = ['frame', 'function', 'amplitude', 'frequency']
    thewriter = csv.DictWriter(f, fieldnames=fieldnames)
    thewriter.writeheader()

    # Function Generator AFG3011 initialization
    with tfg.FuncGen(_VISA_ADDRESS) as fgen:

        print(" Please choose an action ")


        def acquisition():
            global counter
            while counter < 100000:
                pic = cam.snap(os.path.join('C:/Users/ARSL/Desktop/Mahmoud/experiment1', f"frame{counter}.png"))
                cv2.imshow('exp', pic)
                cv2.waitKey(1)
                act_settings = fgen.ch1.get_settings()
                frame_dict = {'frame': f"{counter}"}
                act_settings.update(frame_dict)
                thewriter.writerow(act_settings)
                counter += 1


        def tuning():
            global freq, amp
            while True:
                if keyboard.read_key() == 'up':
                    freq += f_step_small
                    fgen.ch1.set_frequency(freq, unit="Hz")
                    time.sleep(0.2)

                elif keyboard.read_key() == 'down':
                    freq -= f_step_small
                    fgen.ch1.set_frequency(freq, unit="Hz")
                    time.sleep(0.2)

                elif keyboard.read_key() == 'w':
                    freq += f_step_big
                    fgen.ch1.set_frequency(freq, unit="Hz")
                    time.sleep(0.2)

                elif keyboard.read_key() == 'z':
                    freq -= f_step_big
                    fgen.ch1.set_frequency(freq, unit="Hz")
                    time.sleep(0.2)

                elif keyboard.read_key() == 'right':
                    amp += a_step_small
                    fgen.ch1.set_amplitude(amp)
                    time.sleep(0.2)

                elif keyboard.read_key() == 'left':
                    amp -= a_step_small
                    fgen.ch1.set_amplitude(amp)
                    time.sleep(0.2)

                elif keyboard.read_key() == '9':
                    amp += a_step_big
                    fgen.ch1.set_amplitude(amp)
                    time.sleep(0.2)

                elif keyboard.read_key() == '1':
                    amp -= a_step_big
                    fgen.ch1.set_amplitude(amp)
                    time.sleep(0.2)

                elif keyboard.read_key() == 'enter':
                    fgen.ch1.print_settings()
                    cv2.destroyAllWindows()
                    break

                else:
                    print("invalid key in acoustic manipulation")


        t1 = threading.Thread(target=acquisition)
        t2 = threading.Thread(target=tuning)

        while True:

            # finish the programme
            if keyboard.read_key() == "space":
                break


            # Start acoustic manipulation
            elif keyboard.read_key() == "a":
                print(' Starting acoustic manipulation')
                s_waveform = input('Please choose the waveform: SIN, SQU, RAMP, PULS ').strip()  # Choosing the waveform
                fgen.ch1.set_function(s_waveform)
                fgen.ch1.set_frequency(freq, unit="Hz")
                fgen.ch1.set_amplitude(amp)
                fgen.ch1.set_output("ON")
                fgen.ch2.set_output("OFF")

                # Data acquisition
                t1.start()
                # Tuning the values of the frequency and amplitude values
                t2.start()

                t1.join()
                t2.join()

            else:
                print('invalid action')

print('Experiment finished')



