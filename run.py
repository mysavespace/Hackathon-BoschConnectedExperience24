# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC orginazers
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# ===================================== GENERAL IMPORTS ==================================
import sys
sys.path.append(".")
from multiprocessing import Queue, Event
import logging
import time

# Import the processSerialHandler module
from src.hardware.serialhandler.processSerialHandler import processSerialHandler, parallel_parking, forward_driving, engine_on, engine_off, parking_exit
# from src.hardware.utils.line import doImage

# ===================================== PROCESS IMPORTS ==================================
from src.gateway.processGateway import processGateway
# from src.hardware.camera.processCamera import processCamera
from src.hardware.serialhandler.processSerialHandler import processSerialHandler
# from src.utils.PCcommunicationDemo.processPCcommunication import (
#     processPCCommunicationDemo,
# )
# from src.utils.PCcommunicationDashBoard.processPCcommunication import (
#     processPCCommunicationDashBoard,
# )
# from src.data.CarsAndSemaphores.processCarsAndSemaphores import processCarsAndSemaphores
# from src.data.TrafficCommunication.processTrafficCommunication import (
#     processTrafficCommunication,
# )

# ======================================== SETTING UP ====================================
allProcesses = list()
queueList = {
    "Critical": Queue(),
    "Warning": Queue(),
    "General": Queue(),
    "Config": Queue(),
}

logging = logging.getLogger()

TrafficCommunication = False
Camera = False
SerialHandler = True
# ===================================== SETUP PROCESSES ==================================

# Initializing gateway
processGateway = processGateway(queueList, logging)
allProcesses.append(processGateway)

# Initializing camera
if Camera:
    processCamera = processCamera(queueList, logging)
    allProcesses.append(processCamera)

# Initializing GPS
if TrafficCommunication:
    processTrafficCommunication = processTrafficCommunication(queueList, logging, 3)
    allProcesses.append(processTrafficCommunication)

# Initializing serial connection NUCLEO - > PI
if SerialHandler:
    processSerialHandler = processSerialHandler(queueList, logging)
    allProcesses.append(processSerialHandler)

# ===================================== START PROCESSES ==================================
for process in allProcesses:
    process.daemon = True
    process.start()

import cv2
import base64
import os
import requests
import numpy as np

speed = 10
fails = 0

def ForwardDriving(speed,vtime):
    forward_driving(processSerialHandler, queueList, logging, False, speed)
    time.sleep(vtime)
    forward_driving(processSerialHandler, queueList, logging, False, 0)

# Call the parallel_parking function
def Start_parking():
    print("PARKING STARTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT")
    parallel_parking(processSerialHandler, queueList, logging, False)

def Exit_parking(parking):
    if(parking != True):
        parking = True
        parking_exit(processSerialHandler, queueList, logging, False)
        parking = False

# ForwardDriving(5,30)
# ForwardDriving(0,0)

# Start_parking(parking)
# Exit_parking(parking)
# # import io

# url = "http://34.22.223.14:3000/sign"

# def send_single_image(image_data):
#     # Create a dictionary containing the image data
#     files = {'image': ("image.jpg", image_data)}
#     # Send a POST request with the image data
#     response = requests.post(url, files=files)
    
#     # Print the response
#     print(f"Response: {response.json()}")

# def getCameraData(logger, queueList):
#     debug = False
#     if debug:
#         logger.warning("getting")
#     img = {"msgValue": 1}
#     while type(img["msgValue"]) != type(":text"):
#         img = queueList["General"].get()
#     image_data = base64.b64decode(img["msgValue"])
#     img = np.frombuffer(image_data, dtype=np.uint8)
#     image = cv2.imdecode(img, cv2.IMREAD_COLOR)
#     return send_single_image(image)
    # if debug:
    #     logger.warning("got")
    
    # # cv2.imwrite("test_test.jpg", image)
    # return image

# forward_driving(processSerialHandler, queueList, logging, False, 0.0)
# time.sleep(20)
# forward_driving(processSerialHandler, queueList, logging, False, 0.0)

# while(True):
#     getCameraData(logging, queueList)
#     time.sleep(0.5)

# while(True):
#     if(parking == False):
#         result = doImage(getCameraData(logging,queueList))
#         if(result == 1):
#             print("not in line...")
#             fails += result
#             if(fails >= 2):
#                 forward_driving(processSerialHandler, queueList, logging, False, 0.0)
#                 engine_off(processSerialHandler, queueList, logging, False)
#                 print("Line not found - Engine off")
#                 # break
#         else:
#             print("Found lines")
#             fails = 0
#     time.sleep(0.05)

# ===================================== STAYING ALIVE ====================================
# blocker = Event()
# try:
#     blocker.wait()
# except KeyboardInterrupt:
#     print("\nCatching a KeyboardInterruption exception! Shutdown all processes.\n")
#     for proc in allProcesses:
#         print("Process stopped", proc)
#         proc.stop()
#         proc.join()
