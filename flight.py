import logging
import math
from threading import Thread
import time
import json
import numpy as np
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
import socket
import struct
from multiprocessing import Process, Queue, Value
import quaternion

# Specify the uri of the drone to which we want to connect (if your radio
# channel is X, the uri should be 'radio://0/X/2M/E7E7E7E7E7')
uri = 'radio://0/35/2M/E7E7E7E7E7'

# Optitrack communication ports etc
OPTI_PORT = '1511'
OPTI_IP = '192.168.0.99'
CLIENT_ID = '192.168.1.166'
CLIENT_PORT = '3500'

# Logging level
LOGLEVEL = logging.INFO

# Specify the variables we want to log (all at 100 Hz)
variables = [
    # Measurements
    'kalman.q0',
    'kalman.q1',
    'kalman.q2',
    'kalman.q3',
    'stateEstimate.qx',
    'stateEstimate.qy',
    'stateEstimate.qz',
    'stateEstimate.qw',
    # Setpoint
    'ae483log.o_x_des',
    'ae483log.o_y_des',
    'ae483log.o_z_des',
]


class SimpleClient:
    def __init__(self, uri, use_controller=False, use_observer=0):
        self.init_time = time.time()
        self.use_controller = use_controller
        self.use_observer = use_observer
        self.cf = Crazyflie(rw_cache='./cache')
        self.cf.connected.add_callback(self.connected)
        self.cf.connection_failed.add_callback(self.connection_failed)
        self.cf.connection_lost.add_callback(self.connection_lost)
        self.cf.disconnected.add_callback(self.disconnected)
        print(f'Connecting to {uri}')
        self.cf.open_link(uri)
        self.is_connected = False
        self.data = {}
        self.log_skip_counter = 0

    def connected(self, uri):
        print(f'Connected to {uri}')
        self.is_connected = True

        # Start logging
        self.logconfs = []
        self.logconfs.append(LogConfig(name=f'LogConf0', period_in_ms=10))
        num_variables = 0
        for v in variables:
            num_variables += 1
            if num_variables > 5: # <-- could increase if you paid attention to types / sizes (max 30 bytes per packet)
                num_variables = 0
                self.logconfs.append(LogConfig(name=f'LogConf{len(self.logconfs)}', period_in_ms=10))
            self.data[v] = {'time': [], 'data': []}
            self.logconfs[-1].add_variable(v)
        for logconf in self.logconfs:
            try:
                self.cf.log.add_config(logconf)
                logconf.data_received_cb.add_callback(self.log_data)
                logconf.error_cb.add_callback(self.log_error)
                logconf.start()
            except KeyError as e:
                print(f'Could not start {logconf.name} because {e}')
                for v in logconf.variables:
                    print(f' - {v.name}')
            except AttributeError:
                print(f'Could not start {logconf.name} because of bad configuration')
                for v in logconf.variables:
                    print(f' - {v.name}')

         # Reset the stock EKF
        self.cf.param.set_value('kalman.resetEstimation', 1)
        
        # Enable the controller (1 for stock controller, 4 for ae483 controller)
        if self.use_controller: 
            self.cf.param.set_value('stabilizer.controller', 4)
        else:
            self.cf.param.set_value('stabilizer.controller', 1)

        # # Enable the observer (0 for disable, 1 for enable)
        # if self.use_observer:
        #     self.cf.param.set_value('ae483par.use_observer', 1)
        # else:
        #     self.cf.param.set_value('stabilizer.estimator', 2)
        #
        # # Reset the stock EKF
        # self.cf.param.set_value('kalman.resetEstimation', 1)
        # time.sleep(0.1)
        # # self.cf.param.set_value('kalman.resetEstimation', 0)

        # self.cf.param.set_value('ae483par.use_observer', self.use_observer)

    def connection_failed(self, uri, msg):
        print(f'Connection to {uri} failed: {msg}')

    def connection_lost(self, uri, msg):
        print(f'Connection to {uri} lost: {msg}')

    def disconnected(self, uri):
        print(f'Disconnected from {uri}')
        self.is_connected = False

    def log_data(self, timestamp, data, logconf):
        self.log_skip_counter += 1
        if self.log_skip_counter % 5 == 0:
            for v in logconf.variables:
                self.data[v.name]['time'].append(timestamp)
                self.data[v.name]['data'].append(data[v.name])
                if 'stateEstimate' in v.name or 'kalman' in v.name:
                    print(f'{current_milli_time()},Drone internal {v.name},{data[v.name]}')

    def log_error(self, logconf, msg):
        print(f'Error when logging {logconf}: {msg}')

    def move(self, x, y, z, yaw, dt):
        print(f'Move to {x}, {y}, {z} with yaw {yaw} degrees for {dt} seconds')
        start_time = time.time()
        while time.time() - start_time < dt:
            self.cf.commander.send_position_setpoint(x, y, z, yaw)
            time.sleep(0.1)

    def move_smooth(self, p1, p2, yaw, dt):
        print(f'Move smoothly from {p1} to {p2} with yaw {yaw} degrees in {dt} seconds')
        p1 = np.array(p1)
        p2 = np.array(p2)
        start_time = time.time()
        while True:
            current_time = time.time()
            s = (current_time - start_time) / dt
            p = (1 - s) * p1 + (s * p2)
            self.cf.commander.send_position_setpoint(p[0], p[1], p[2], yaw)
            if s >= 1:
                return
            else:
                time.sleep(0.1)

    def stop(self, dt):
        print(f'Stop for {dt} seconds')
        self.cf.commander.send_stop_setpoint()
        start_time = time.time()
        while time.time() - start_time < dt:
            time.sleep(0.1)

    def disconnect(self):
        self.cf.close_link()

    def write_data(self, filename='logged_data.json'):
        with open(filename, 'w') as outfile:
            json.dump(self.data, outfile, indent=4, sort_keys=False)

    def reset_estimator(self):
        logging.info('Resetting Kalman Filter')
        self.cf.param.set_value('kalman.resetEstimation', 1)
        time.sleep(0.1)
        self.cf.param.set_value('kalman.resetEstimation', 0)

    def activate_kalman_estimator(self):
        logging.info('Activating Kalman Filter')
        self.cf.param.set_value('stabilizer.estimator', 2)

        # Set the std deviation for the quaternion data pushed into the kalman filter.
        self.cf.param.set_value('locSrv.extQuatStdDev', 0.06)

# def get_euler_from_quaternion(qw, qx, qy, qz):
#     yaw = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (math.sqrt(qy) + math.sqrt(qz)))
#     yaw = np.rad2deg(yaw)
#     pitch = np.arcsin(2 * (qw * qy - qx * qz))
#     pitch = np.rad2deg(pitch)
#     roll = np.arctan2(2 * (qw * qx + qy * qz), 1 - 2 * (math.sqrt(qx) + math.sqrt(qy)))
#     roll = np.rad2deg(roll)

#     return [roll, yaw, pitch] 

import math
 
def euler_from_quaternion(w, x, y, z):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        roll = math.degrees(roll_x)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        pitch = math.degrees(pitch_y)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        yaw = math.degrees(yaw_z)
     
        return [roll, yaw, pitch] # in degrees

prev_quat = None
def optitrack(queue: Queue, run_process: Value):
    print('Beginning optitrack socket listener')
    skip_counter = 0
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(('0.0.0.0', int(CLIENT_PORT)))
        print(f'Starting optitrack socket listener')
        while run_process.value == 1:
            data = s.recvfrom(1024)[0]
            if not data:
                print(f'No data received')
                break
            else:
                [a, b, c, d, e, f, g, h, i, j, k, l] = struct.unpack('ffffffffffff', data)
                x = -a
                y = c
                z = b
                opti_x = d
                opti_y = e
                opti_z = f
                opti_w = g
                roll = -h
                yaw = i
                pitch = -j
                bodyID = k
                framecount = l
                print(f'{current_milli_time()}, Received from Optitrack, w x y z p r y,{opti_w},{opti_x},{opti_y},{opti_z},{pitch},{roll},{yaw}')
                quat_x = -opti_x
                quat_y = -opti_z
                quat_z = opti_y
                quat_w = opti_w
                conversion_roll = euler_from_quaternion(quat_w, quat_x, quat_y, quat_z)[0]
                conversion_yaw = euler_from_quaternion(quat_w, quat_x, quat_y, quat_z)[1]
                conversion_pitch = euler_from_quaternion(quat_w, quat_x, quat_y, quat_z)[2]
                print(f'{current_milli_time()}, Euler conversion, p r y, {conversion_pitch}, {conversion_roll}, {conversion_yaw}')
                # angle = 0
                # global prev_quat
                # if prev_quat is not None:
                #     new_quat = quaternion.from_float_array([quat_w, quat_x, quat_y, quat_z])
                #     rot = new_quat.conj() * prev_quat
                #     norm = np.linalg.norm(quaternion.as_float_array(rot))
                #     if norm != 0:
                #         rot = rot / norm
                #     angle = math.acos(rot.w)
                # if queue.empty():
                #     if angle > 0.5:
                #         print(f'Skipped because angle = {angle}')
                #     else:
                #         queue.put((x, y, z, quat_x, quat_y, quat_z, quat_w))
                #         prev_quat = quaternion.from_float_array([quat_w, quat_x, quat_y, quat_z])

                if queue.empty():
                    if abs(quat_w) > 0.2:
                        queue.put((x, y, z, quat_x, quat_y, quat_z, quat_w))
                    else:
                        print(f'Skipped because qw = {quat_w}')
                # prev_quat = quaternion.from_float_array([quad_w, quad_x, quad_y, quad_z])

    print('Ending optitrack socket listener')

def send_pose(client, queue: Queue):
    print('starting send_pose thread')
    while client.is_connected:
        x, y, z, qx, qy, qz, qw = queue.get()
        print(f'{current_milli_time()}, Sending quat to drone, qw qx qy qz,{qw},{qx},{qy},{qz}')
        client.cf.extpos.send_extpose(x, y, z, qx, qy, qz, qw) # or send to controller
    print('Ending send_pose thread')

def current_milli_time():
    return round(time.time() * 1000000)

if __name__ == '__main__':
    # Initialize everything
    logging.basicConfig(level=LOGLEVEL)
    cflib.crtp.init_drivers()

    # Create and start the client that will connect to the drone
    client = SimpleClient(uri, use_controller=False, use_observer=False)
    while not client.is_connected:
        print(f' ... connecting ...')
        time.sleep(2.0)

    # Listen to OptiTrack
    q = Queue()
    run_process = Value('b', 1)
    optitrack_process = Process(target=optitrack, args=(q, run_process))
    logging.info('beginning OptiTrack process')
    optitrack_process.start()

    client.activate_kalman_estimator()
    logging.info('activating kalman filter')
    client.reset_estimator()
    logging.info('resetting kalman filter')

    # Send position estimates from queue
    send_pose_thread = Thread(target=send_pose, args=(client, q,))
    logging.info('beginning estimate thread')
    send_pose_thread.start()
    
    # Leave time at the start to initialize and allow kalman filter to converge
    client.stop(5.0)

    # Take off and hover (with zero yaw)
    logging.info('Take off initiated')
    
    client.move(0.0, 0.0, 0.15, 0.0, 2)
    client.move(0.0, 0.0, 0.25, 0.0, 2)
    client.move(0.0, 0.0, 0.5, 0.0, 5)

    # Fly in a square five times (with a pause at each corner)
    num_squares = 1
    for i in range(num_squares):
        client.move(0.75, 0.0, 0.5, 0.0, 2.0)
        client.move(0.75, 0.75, 0.5, 0.0, 2.0)
        client.move(0.0, 0.75, 0.5, 0.0, 2.0)
        client.move(0.0, 0.0, 0.5, 0.0, 2.0)
        client.move(0.0, 0.0, 0.5, 45.0, 2.0)
        client.move(0.0, 0.0, 0.5, 0.0, 2.0)

    # Fly in a spiral
    # client.move(1, 0, 1, 0)
    # for step in range(100):
    #     sp_angle = 6 * math.pi * step / 100
    #     radius = 0.75 - 0.4 * step / 100
    #     x = math.cos(sp_angle) * radius
    #     y = math.sin(sp_angle) * radius
    #     yaw = (3 * 360 * step / 100) % 360
    #     height = 1.0 - 0.7 * step / 100
    #     dt = 0.2 - 0.1 * step / 100
    #     client.move(x, y, height, yaw, dt)

    # Go back to hover (with zero yaw) and prepare to land
    client.move(0.0, 0.0, 0.50, 0.0, 1.0)
    client.move(0.0, 0.0, 0.30, 0.0, 1.0)
    client.move(0.0, 0.0, 0.1, 0.0, 1.0)

    # Land
    client.stop(1.0)

    # Disconnect from drone
    client.disconnect()

    # Write data from flight
    # client.write_data('hardware_data.json')

    send_pose_thread.join()
    run_process.value = 0 
    optitrack_process.join()