import logging
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
    # State estimates (stock code)
    'ae483log.o_x',
    'ae483log.o_y',
    'ae483log.o_z',
    'ae483log.psi',
    'ae483log.theta',
    'ae483log.phi',
    'ae483log.v_x',
    'ae483log.v_y',
    'ae483log.v_z',
    # Measurements
    'ae483log.w_x',
    'ae483log.w_y',
    'ae483log.w_z',
    'ae483log.n_x',
    'ae483log.n_y',
    'ae483log.r',
    'ae483log.a_z',
    # Setpoint
    'ae483log.o_x_des',
    'ae483log.o_y_des',
    'ae483log.o_z_des',
    # Motor power commands
    'ae483log.m_1',
    'ae483log.m_2',
    'ae483log.m_3',
    'ae483log.m_4',
]


class SimpleClient:
    def __init__(self, uri, use_controller=False, use_observer=False):
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
        time.sleep(1)
        self.cf.param.set_value('kalman.resetEstimation', 0)

        # Enable the controller (1 for stock controller, 4 for ae483 controller)
        if self.use_controller: 
            self.cf.param.set_value('stabilizer.controller', 4)
        else:
            self.cf.param.set_value('stabilizer.controller', 4)

        # Enable the observer (0 for disable, 1 for enable)
        if self.use_observer:
            self.cf.param.set_value('ae483par.use_observer', 1)
        else:
            self.cf.param.set_value('stabilizer.estimator', 2)

    def connection_failed(self, uri, msg):
        print(f'Connection to {uri} failed: {msg}')

    def connection_lost(self, uri, msg):
        print(f'Connection to {uri} lost: {msg}')

    def disconnected(self, uri):
        print(f'Disconnected from {uri}')
        self.is_connected = False

    def log_data(self, timestamp, data, logconf):
        for v in logconf.variables:
            self.data[v.name]['time'].append(timestamp)
            self.data[v.name]['data'].append(data[v.name])

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

def optitrack(queue: Queue, run_process: Value):
    logging.info('Beginning socket listener')
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(('0.0.0.0', int(CLIENT_PORT)))
        logging.info(f'Starting socket listener')
        while run_process.value == 1:
            data = s.recvfrom(1024)[0]
            if not data:
                logging.info(f'No data received')
                break
            else:
                [a, b, c, d, e, f, g, h, i, j, k, l] = struct.unpack('ffffffffffff', data)
                x = -a
                y = c
                z = b
                qx = d
                qy = e
                qz = f
                qw = g
                roll = -h
                yaw = i
                pitch = -j
                bodyID = k
                framecount = l
                print(f'x = {x}, y = {y}, z = {z} \n qx = {qx}, qy = {qy}, qz = {qz}, qw = {qw} \n roll = {roll}, yaw = {yaw}, pitch = {pitch} \n bodyID = {bodyID}, framecount = {framecount}')
                if queue.empty():
                    queue.put((x, y, z, qx, qy, qz, qw))

def send_pose(client, queue: Queue):
    logging.info('sending full pose')
    while client.is_connected:
        x, y, z, qx, qy, qz, qw = queue.get()
        client.cf.extpos.send_extpose(x, y, z, qx, qy, qz, qw)
        time.sleep(5)

if __name__ == '__main__':
    # Initialize everything
    logging.basicConfig(level=LOGLEVEL)
    cflib.crtp.init_drivers()

    # Create and start the client that will connect to the drone
    client = SimpleClient(uri, use_controller=False, use_observer=False)
    while not client.is_connected:
        print(f' ... connecting ...')
        time.sleep(1.0)

    # Listen to OptiTrack
    q = Queue()
    run_process = Value('b', 1)
    optitrack_process = Process(target=optitrack, args=(q, run_process))
    logging.info('beginning OptiTrack process')
    optitrack_process.start()

    # Send position estimates from queue
    estimate_thread = Thread(target=send_pose, args=(client, q,))
    logging.info('beginning estimate thread')
    estimate_thread.start()
    
    # Leave time at the start to initialize
    client.stop(1.0)

    # Take off and hover (with zero yaw)
    client.move(0.0, 0.0, 0.15, 0.0, 1.0)
    client.move(0.0, 0.0, 0.50, 0.0, 1.0)

    # Fly in a square five times (with a pause at each corner)
    num_squares = 2
    for i in range(num_squares):
        #client.move_smooth([0.0, 0.0, 0.5], [0.5, 0.0, 0.5], 0.0, 2.0)
        client.move(0.5, 0.0, 0.5, 0.0, 1.0)
        #client.move_smooth([0.5, 0.0, 0.5], [0.5, 0.5, 0.5], 0.0, 2.0)
        client.move(0.5, 0.5, 0.5, 0.0, 1.0)
        #client.move_smooth([0.5, 0.5, 0.5], [0.0, 0.5, 0.5], 0.0, 2.0)
        client.move(0.0, 0.5, 0.5, 0.0, 1.0)
        #client.move_smooth([0.0, 0.5, 0.5], [0.0, 0.0, 0.5], 0.0, 2.0)
        client.move(0.0, 0.0, 0.5, 0.0, 1.0)

    # Go back to hover (with zero yaw) and prepare to land
    client.move(0.0, 0.0, 0.50, 0.0, 1.0)
    client.move(0.0, 0.0, 0.15, 0.0, 1.0)

    # Land
    client.stop(1.0)

    # Disconnect from drone
    client.disconnect()

    # Write data from flight
    client.write_data('hardware_data.json')

    estimate_thread.join()
    run_process.value = 0 
    optitrack_process.join()