import imu
import recorder
import uuid
import threading
import socket
import json
import time
import euclid
import traceback
#import serial
import sys
import math

##### ATTENTION!!! The '\n' at the end of each network message is extremely IMPORTANT!!!

# Constants:
HOST = '0.0.0.0'
PORT_IN = 8301
PORT_OUT = 8300
LIST_IDS_NAME = 'listIds'
IDS_PARAM_NAME = 'ids'
TARE_NAME = 'tare'
START_RECORD_NAME = 'startRecording'
STOP_RECORD_NAME = 'stopRecording'
SENSOR_ID_PARAM_NAME = 'sensorId'
STEP_TIME_PARAM_NAME = 'stepTime'
RECORD_ID_PARAM_NAME = 'recordId'
RECORD_DATA_PARAM_NAME = 'recordData'
DEFAULT_STEP_TIME = 16

# Global Variables:
valid_ids = ['python-imu']  # which sensor ids can this sensor use? this is not actually validated on this impl
recorders = dict()  # maps recordId => recorder, to hold the curves currently being recorded
ref_data = euclid.Quaternion(0.0, 0.0, 0.0, 0.0)
last_data = euclid.Quaternion(0.0, 0.0, 0.0, 0.0)


# The following methods are called when service calls are received,
# they all must store the response values on a map passed by reference (resp).

# lists valid ids
def list_ids(call, params, resp):
    resp[IDS_PARAM_NAME] = valid_ids


# stores a reference quaternion
def tare(call, params, resp):
    global ref_data
    ref_data = last_data


# starts a new curve recording
def start_recording(call, params, resp):
    sensor_id = params[SENSOR_ID_PARAM_NAME]
    step = params.get(STEP_TIME_PARAM_NAME, DEFAULT_STEP_TIME)
    recs = recorders.get(sensor_id, dict())
    record_id = str(uuid.uuid4())  # random id
    recs[record_id] = recorder.Recorder(step)
    recorders[sensor_id] = recs
    resp[RECORD_ID_PARAM_NAME] = record_id


# stops a previously started curve recording and returns it
def stop_recording(call, params, resp):
    sensor_id = params[SENSOR_ID_PARAM_NAME]
    record_id = params[RECORD_ID_PARAM_NAME]
    recs = recorders[sensor_id]
    rec = recs[record_id]
    del recs[record_id]
    resp[RECORD_DATA_PARAM_NAME] = rec.pack()


# should be called by the main loop whenever new data is available from the sensor
def on_sensor_changed(new_data, sensor_id):
    global last_data
    last_data = new_data
    data = euclid.Quaternion(
        new_data.w, new_data.x - ref_data.x, new_data.y - ref_data.y,  new_data.z - ref_data.z).normalized()
    recs = recorders.get(sensor_id, dict())
    ts = int(time.time() * 1000)  # current time in milliseconds
    for record_id, rec in recs.iteritems():
        rec.add(ts, data)


########
# Thread to handle one single connection.
class ConnectionThread(threading.Thread):
    def __init__(self, con):
        threading.Thread.__init__(self)
        self.con = con

    @staticmethod
    def process_msg(msg):
        call = json.loads(msg)
        service = call['service']
        params = call.get('parameters', dict())
        resp = {'type': 'SERVICE_CALL_RESPONSE'}
        resp_data = dict()
        try:
            if service == LIST_IDS_NAME:
                list_ids(call, params, resp_data)
            elif service == TARE_NAME:
                tare(call, params, resp_data)
            elif service == START_RECORD_NAME:
                start_recording(call, params, resp_data)
            elif service == STOP_RECORD_NAME:
                stop_recording(call, params, resp_data)
            resp['responseData'] = resp_data
        except:
            resp['error'] = 'error while processing request:\n' + traceback.format_exc()
        return resp

    def run(self):
        con = self.con
        chunks = []
        chunk = ''
        try:
            # connection receive
            while True:
                msg = ''
                # message loop
                while True:
                    chunk = con.recv(1024)
                    if len(chunk) == 0:
                        raise RuntimeError("socket connection broken")
                    i = chunk.find('\n')
                    if i < 0:
                        chunks.append(chunk)
                    else:
                        chunks.append(chunk[:i])
                        msg = msg.join(chunks)
                        chunks = [chunk[i + 1:]]
                        break
                print('[client] read message: ' + msg)
                resp = self.process_msg(msg)
                resp_msg = json.dumps(resp) + '\n'
                con.send(resp_msg)
                print('[client] sent message:' + resp_msg)
            con.close()
        except:
            traceback.print_exc(file=sys.stdout)


########
# Thread to accept new connections.
class ServerThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self._stop = threading.Event()
        self._sock = socket.socket()

    def stop(self):
        self._stop.set()
        self._sock.close()

    def run(self):
        try:
            self._stop.clear()
            s = self._sock
            local_ep = (HOST, PORT_IN)
            s.bind(local_ep)
            s.settimeout(10.0)
            s.listen(5)
            print('[server] listening on: ' + str(local_ep) + ';')
            while not self._stop.is_set():
                print('[server] accepting...')
                try:
                    con, addr = s.accept()
                    print('[server] connection received from: ' + str(addr) + ';')
                    con.settimeout(10.0)
                    ConnectionThread(con).start()
                except:
                    print('[server] timeout')
            print('[server] stopped;')
        except:
            if not self._stop.is_set():
                traceback.print_exc(file=sys.stdout)


########
# Thread to generate dummy data. Only use if the IMU is not available
class DummyDataThread(threading.Thread):
    def __init__(self, f):
        threading.Thread.__init__(self)
        self._stop = threading.Event()
        self.f = f

    def stop(self):
        self._stop.set()

    def run(self):
        f = self.f
        while not self._stop.is_set():
            sensor_id = valid_ids[0]
            a = math.fmod(2 * math.pi * time.time(), f) # which angle at f turns per second?
            q = euclid.Quaternion.new_rotate_euler(a, 0, 0)
            on_sensor_changed(q, sensor_id)
            time.sleep(0.015)


#### THE SERVER THREAD SHOULD BE HERE
server_thread = ServerThread()
server_thread.start()
dummy_thread = DummyDataThread(2) # 2 turns per second
dummy_thread.start()

i = raw_input('to stop, press any key...\n')
server_thread.stop()
dummy_thread.stop()
exit(0)

#### THE MAIN IMU LOOP SHOULD BE HERE
serial_port = serial.Serial(imu.get_port(),timeout=1,baudrate=115200)
sensor3 = imu.IMU(serial_port, 3)
sensor2 = imu.IMU(serial_port, 2)

# serial_port = serial.Serial('COM10', timeout=1)
sensor2.calibrate()
sensor3.calibrate()

sensor2.reset_timestamp()
sensor3.reset_timestamp()

sensor3.startStreaming()
sensor2.startStreaming()

running = True
while running:
    q = sensor2.listen_streaming()
    if q is not None:
        sensor_id = q[0]
        q = euclid.Quaternion(q[4], q[1], q[2], q[3])
        on_sensor_changed(q, sensor_id)
