import imu
import recorder
import uuid
import threading
import socket
import json
import time
import euclid
import traceback
import serial

##### ATTENTION!!! The '\n' at the end of each network message is extremely IMPORTANT!!!

# Constants:
HOST = '0.0.0.0'
PORT_IN = 8300
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
DEFAULT_STEP_TIME = 16;

# Global Variables:
valid_ids = ['python-imu']  # which sensor ids can this sensor use? this is not actually validated on this impl
recorders = dict()  # maps recordId => recorder, to hold the curves currently being recorded
ref_data = euclid.Quaternion(0.0, 0.0, 0.0, 0.0)
last_data = euclid.Quaternion(0.0, 0.0, 0.0, 0.0)


# The following methods are called when service calls are received,
# they all must store the response values on a map passed by reference (resp).

# lists valid ids
def list_ids(call, resp):
    resp[IDS_PARAM_NAME] = valid_ids


# stores a reference quaternion
def tare(call, resp):
    global ref_data
    ref_data = last_data


# starts a new curve recording
def start_recording(call, resp):
    sensor_id = call[SENSOR_ID_PARAM_NAME]
    step = call.get(STEP_TIME_PARAM_NAME, DEFAULT_STEP_TIME)
    recs = recorders.get(sensor_id, dict())
    record_id = str(uuid.uuid4())  # random id
    recs[record_id] = recorder.Recorder(step)
    recorders[sensor_id] = recs
    resp[RECORD_ID_PARAM_NAME] = record_id


# stops a previously started curve recording and returns it
def stop_recording(call, resp):
    sensor_id = call[SENSOR_ID_PARAM_NAME]
    record_id = call[RECORD_ID_PARAM_NAME]
    recs = recorders[sensor_id]
    rec = recs[record_id]
    del recs[record_id]
    resp[RECORD_DATA_PARAM_NAME] = rec.pack()


# This method should be called by the main loop whenever new data is available from the sensor
def on_sensor_changed(new_data, sensor_id):
    global last_data
    last_data = new_data
    data = euclid.Quaternion(
        new_data.w, new_data.x - ref_data.x, new_data.y - ref_data.y,  new_data.z - ref_data.z).normalized()
    recs = recorders.get(sensor_id, dict())
    ts = int(time.time() * 1000)  # current time in milliseconds
    for record_id, rec in recs.iteritems():
        rec.add(ts, data);


class ServiceCallThread(threading.Thread):
    def __init__(self, con, call):
        threading.Thread.__init__(self)
        self.con = con
        self.call = call

    def run(self):
        try:
            con = self.con
            call = self.call
            service = call['service']
            resp = {'type': 'SERVICE_CALL_RESPONSE'}
            resp_data = dict()
            if service == LIST_IDS_NAME:
                list_ids(call, resp_data)
            elif service == TARE_NAME:
                tare(call, resp_data)
            elif service == START_RECORD_NAME:
                start_recording(call, resp_data)
            elif service == STOP_RECORD_NAME:
                stop_recording(call, resp_data)
            resp['responseData'] = resp_data
            con.send(json.dumps(resp) + '\n')
            con.close()
        except ...:
            traceback.print_exc()


class ServerThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.running = False

    def run(self):
        try:
            s = socket.socket()
            local_ep = (HOST, PORT_IN)
            s.bind(local_ep)
            s.listen(5)
            print('listening on: ', local_ep, ';')
            self.running = True
            while self.running:
                con, addr = s.accept()
                print('msg received from: ', addr, ';')
                msg = con.recv(1024)
                while not msg.endswith('\n'):
                    msg = msg + con.recv(1024)
                print(msg)
                call = json.loads(msg)
                ServiceCallThread(con, call).start();
        except ...:
            traceback.print_exc()




#### THE SERVER THREAD SHOULD BE HERE
ServerThread().start()



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
        q = euclid.Quaternion(q[4], q[1], q[2], q[3]);
        on_sensor_changed(q, sensor_id)
