import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.utils.reset_estimator import reset_estimator

# URI to the Crazyflie to connect to
uri1 = uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E708')

# Change the sequence according to your setup
#             x    y    z  YAW
sequence1 = [
    (-1.0, 0.0, 0.4, 0),
    (-1.0, 0.0, 0.5, 0),
    ( 1.0, 0.0, 0.5, 0),
    ( 1.0, 0.0, 0.4, 0),
    ( 1.0, 0.0, 0.0, 0),
]

curr_pos = (0, 0, 0)

deadzone = (0,0,0.5)

safety_distance = 0.5




def in_deadzone(position, deadzone ,safety_distance):
    x, y, z = position
    dx = abs(x - deadzone[0])
    dy = abs(y - deadzone[1])
    dz = abs(z - deadzone[2])
    if dx < safety_distance and dy < safety_distance and dz < safety_distance:
        return True
    return False




def take_off(cf, position):
    take_off_time = 1.0
    sleep_time = 0.1
    steps = int(take_off_time / sleep_time)
    vz = position[2] / take_off_time

    print(f'take off at {position[2]}')

    for i in range(steps):
        cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        time.sleep(sleep_time)


def position_callback(timestamp, data, logconf):
    x = data['kalman.stateX']
    y = data['kalman.stateY']
    z = data['kalman.stateZ']
    global curr_pos 
    curr_pos = (x, y, z)
    #print('pos: ({}, {}, {})'.format(x, y, z))
    



def start_position_printing(scf):
    log_conf = LogConfig(name='Position', period_in_ms=500)
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(position_callback)
    log_conf.start()


def run_sequence(scf, sequence):
    cf = scf.cf

    # Arm the Crazyflie
    cf.platform.send_arming_request(True)
    time.sleep(1.0)

    take_off(cf, sequence[0])
    time.sleep(1.0)

    for i, position in enumerate(sequence):
        print('Setting position {}'.format(position[0]))
        for i in range(50):
            cf.commander.send_position_setpoint(position[0],
                                                position[1],
                                                position[2],
                                                position[3])
            #print(curr_pos)
            while in_deadzone(curr_pos, deadzone, safety_distance):
                print('In deadzone, stopping')
                cf.commander.send_position_setpoint(curr_pos[0], curr_pos[1], curr_pos[2], 0.0) 
                
            time.sleep(0.1)

    cf.commander.send_stop_setpoint()
    # Hand control over to the high level commander to avoid timeout and locking of the Crazyflie
    cf.commander.send_notify_setpoint_stop()

    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)


if __name__ == '__main__':
    cflib.crtp.init_drivers()
    with SyncCrazyflie(uri1, cf=Crazyflie(rw_cache='./cache')) as scf:
        reset_estimator(scf)
        start_position_printing(scf)
        run_sequence(scf, sequence1) 
        # Print data
        

