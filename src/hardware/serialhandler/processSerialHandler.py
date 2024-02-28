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
import serial
from src.templates.workerprocess import WorkerProcess
from src.hardware.serialhandler.threads.filehandler import FileHandler
from src.hardware.serialhandler.threads.threadRead import threadRead
from src.hardware.serialhandler.threads.threadWrite import threadWrite
import time

class processSerialHandler(WorkerProcess):
    """This process handle connection between NUCLEO and Raspberry PI.\n
    Args:
        queueList (dictionar of multiprocessing.queues.Queue): Dictionar of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
        example (bool, optional): A flag for running the example. Defaults to False.
    """

    # ===================================== INIT =========================================
    def __init__(self, queueList, logging, debugging=False, speed=0.0, steer=0.0):
        devFile = "/dev/ttyACM0"
        logFile = "historyFile.txt"

        # comm init
        self.serialCom = serial.Serial(devFile, 19200, timeout=0.1)
        # self.serialCom.flvushInput()
        self.serialCom.flushOutput()

        # log file init
        self.historyFile = FileHandler(logFile)
        self.queuesList = queueList
        self.logger = logging
        self.debugging = debugging
        self.example = True
        self.speed = speed
        self.steer = steer 
        super(processSerialHandler, self).__init__(self.queuesList)

    # ===================================== STOP ==========================================
    def stop(self):
        """Function for stopping threads and the process."""
        for thread in self.threads:
            thread.stop()
            thread.join()
        super(processSerialHandler, self).stop()

    # ===================================== RUN ==========================================
    def run(self):
        """Apply the initializing methods and start the threads."""
        super(processSerialHandler, self).run()

        self.historyFile.close()

    # ===================================== INIT TH =================================
    def _init_threads(self):
        """Initializes the read and the write thread."""
        readTh = threadRead(self.serialCom, self.historyFile, self.queuesList)
        self.threads.append(readTh)
        writeTh = threadWrite(
            self.queuesList, self.serialCom, self.historyFile, self.speed, self.steer
        )
        self.threads.append(writeTh)


# =================================== EXAMPLE =========================================
#             ++    THIS WILL RUN ONLY IF YOU RUN THE CODE FROM HERE  ++
#                  in terminal:    python3 processSerialHandler.py


import time
from multiprocessing import Pipe

def startProcess(process,vtime):
    process.start()
    time.sleep(vtime)
    process.stop()

def forward_driving(process, queueList, logger, debugg, speed, steer=0.6):
    process = processSerialHandler(queueList, logger, debugg, speed, steer)
    if(speed == 0):
        startProcess(process, 0.1)
    else:
        process.start()

def engine_off(process, queueList, logger, debugg):
    pipeRecvRunningSignal, pipeSendRunningSignal = Pipe(duplex=True)
    pipeSendRunningSignal.send({"Type": "Run", "value": True})

def engine_on(process, queueList, logger, debugg):
    pipeRecvRunningSignal, pipeSendRunningSignal = Pipe(duplex=False)
    pipeSendRunningSignal.send({"Type": "Run", "value": True})
    print("Engine ON")

def parallel_parking(process, queueList, logger, debugg):

    print("Parking Started")
    process = processSerialHandler(queueList, logger, debugg, 0.0, 0.0)
    startProcess(process,0.5)

    process = processSerialHandler(queueList, logger, debugg, 25.0, 0.7)
    startProcess(process,4.4)
    
    process = processSerialHandler(queueList, logger, debugg, -20.0, 15.0)
    startProcess(process,3.3)

    process = processSerialHandler(queueList, logger, debugg, -24.0, -22.0)
    startProcess(process,2)

    process = processSerialHandler(queueList, logger, debugg, 10.0, 2.0)
    startProcess(process,2)

    print("Parking Finished")

def parking_exit(process, queueList, logger, debugg):
    print("Exit Parking")

    process = processSerialHandler(queueList, logger, debugg, -10.0, 0.0)
    startProcess(process,2)

    process = processSerialHandler(queueList, logger, debugg, 20.0, -15.0)
    startProcess(process,3)

    process = processSerialHandler(queueList, logger, debugg, 20.0, 20.0)
    startProcess(process,3)

    process = processSerialHandler(queueList, logger, debugg, 20.0, 0.0)
    startProcess(process,3)

    # process = processSerialHandler(queueList, logger, debugg, -10.0, 5.0)
    # startProcess(process,1.5)

    # process = processSerialHandler(queueList, logger, debugg, 24.0, -25.0)
    # startProcess(process,1.6)

    # process = processSerialHandler(queueList, logger, debugg, 20.0, 15.0)
    # startProcess(process,3.5)

    # process = processSerialHandler(queueList, logger, debugg, 25.0, 0.0)
    # startProcess(process,5)
    




if __name__ == "__main__":
    from multiprocessing import Queue, Pipe
    import logging

    allProcesses = list()
    debugg = False

    # We have a list of multiprocessing.Queue() which individualy represent a priority for processes.
    queueList = {
        "Critical": Queue(),
        "Warning": Queue(),
        "General": Queue(),
        "Config": Queue(),
    }
    logger = logging.getLogger()
    # pipeRecv, pipeSend = Pipe(duplex=False)

    parallel_parking(processSerialHandler, queueList, logger, debugg)

