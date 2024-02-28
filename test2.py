# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
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
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE

if __name__ == "__main__":
    import sys

    sys.path.insert(0, "../../..")

from src.templates.workerprocess import WorkerProcess
from src.hardware.camera.threads.threadCamera import threadCamera
from multiprocessing import Pipe
from run import Start_parking
from PIL import Image

class processCamera(WorkerProcess):
    """This process handle camera.\n
    Args:
            queueList (dictionar of multiprocessing.queues.Queue): Dictionar of queues where the ID is the type of messages.
            logging (logging object): Made for debugging.
            debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    # ====================================== INIT ==========================================
    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        pipeRecv, pipeSend = Pipe(duplex=False)
        self.pipeRecv = pipeRecv
        self.pipeSend = pipeSend
        self.debugging = debugging
        super(processCamera, self).__init__(self.queuesList)

    # ===================================== STOP ==========================================
    def stop(self):
        """Function for stopping threads and the process."""
        for thread in self.threads:
            thread.stop()
            thread.join()
        super(processCamera, self).stop()

    # ===================================== RUN ==========================================
    def run(self):
        """Apply the initializing methods and start the threads."""
        super(processCamera, self).run()

    # ===================================== INIT TH ======================================
    def _init_threads(self):
        """Create the Camera Publisher thread and add to the list of threads."""
        camTh = threadCamera(
            self.pipeRecv, self.pipeSend, self.queuesList, self.logging, self.debugging
        )
        self.threads.append(camTh)


def check_for_parking(objects_list):
    for obj in objects_list:
        if obj.get('name') == 'parking':
            return True
    return False

def detect_parking_sign(image):
    # Convert image from BGR to HSV color space
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # Define range of blue color in HSV
    lower_blue = np.array([100, 50, 50])
    upper_blue = np.array([130, 255, 255])

    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(image, image, mask=mask)

    # # Find contours of blue objects
    # contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # # Draw bounding boxes around blue objects
    # for contour in contours:
    #     x, y, w, h = cv2.boundingRect(contour)
    #     print(f"{x},{y},{w},{h}")
    #     image = cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # return image
    return res

# def region_of_interest_square(image):
#     height = image.shape[0]
#     width = image.shape[1]
#     square_size = 500
#     x1 = int(width - 200 - square_size - 100)
#     y1 = int(0)
#     x2 = int(width + square_size)
#     y2 = int(height - 200)
#     square = np.array([[
#         (x1, y1),
#         (x1, y2),
#         (x2, y2),
#         (x2, y1),
#     ]], np.int32)
#     mask = np.ones_like(image) * 255  # Create a white mask
#     # cv2.fillPoly(mask, square, 0)      # Fill the region of interest with black
#     masked_image = cv2.bitwise_and(image, mask)

#     return masked_image

def region_of_intrested_square(image):
    x1 = int(0)
    y1 = int(0)
    x2 = int(200)
    y2 = int(1000)
    square = np.array([[
        (x1, y1),
        (x1, y2),
        (x2, y2),
        (x2, y1),
    ]], np.int32)
    mask = np.ones_like(image) * 255  # Create a white mask
    cv2.fillPoly(mask, square, 0)      # Fill the region of interest with black
    masked_image = cv2.bitwise_and(image, mask)

    return masked_image


def reg_2(image):
    x1 = int(350)
    y1 = int(0)
    x2 = int(800)
    y2 = int(1000)
    square = np.array([[
        (x1, y1),
        (x1, y2),
        (x2, y2),
        (x2, y1),
    ]], np.int32)
    mask = np.ones_like(image) * 255  # Create a white mask
    cv2.fillPoly(mask, square, 0)      # Fill the region of interest with black
    masked_image = cv2.bitwise_and(image, mask)

    return masked_image

# =================================== EXAMPLE =========================================
#             ++    THIS WILL RUN ONLY IF YOU RUN THE CODE FROM HERE  ++
#                  in terminal:    python3 processCamera.py
if __name__ == "__main__":
    from multiprocessing import Queue, Event
    import time
    import logging
    import cv2
    import base64
    import numpy as np
    # from ultralytics import YOLO
    # import json

    allProcesses = list()

    debugg = False

    queueList = {
        "Critical": Queue(),
        "Warning": Queue(),
        "General": Queue(),
        "Config": Queue(),
    }

    logger = logging.getLogger()

    process = processCamera(queueList, logger, False)

    process.daemon = True

    # model = YOLO('best (1).pt')

    time.sleep(4)
    
    process.start()
    
    # if debugg:
    #     logger.warning("getting")

    parking = 0
    parkingDetected = 0
    while True:
        debug = False
        if debug:
            logger.warning("getting")
        img = {"msgValue": 1}
        while type(img["msgValue"]) != type(":text"):
            img = queueList["General"].get()
        image_data = base64.b64decode(img["msgValue"])
        img = np.frombuffer(image_data, dtype=np.uint8)
        image = cv2.imdecode(img, cv2.IMREAD_COLOR)
        image = region_of_intrested_square(image)
        image = reg_2(image)
        if debug:
            logger.warning("got")
        
        # results = model(image, show=False, conf=0.10)[0]
        # data = json.loads(results.tojson())
        # print(data)
        # if check_for_parking(data):
        #     print("Parking found!")
        # else:
        #     print("Parking not found.")
        praking_sign = detect_parking_sign(image)
        # praking_sign = image
        cv2.imwrite("test.jpg", praking_sign)
        if np.any(praking_sign):
            print("Detected!")
            parkingDetected += 1
            if(parking == 0 and parkingDetected >= 1):
                Start_parking()
                parking = 1
                parkingDetected = 0
            # elif(parking == 1 and parkingDetected >= 3):
            #     Start_parking()
            #     print("PARKING STARTTTTTTTTTTTTTTTTTTT")
            #     parking = 2
            #     parkingDetected= 0
        else:
            print("Not detected")
            parkingDetected = 0
        print(f"Done {time.time()}")
            

    process.stop()
