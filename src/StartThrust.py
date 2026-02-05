import time
import logging
import sys
import signal
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.commander import Commander


# URI to the Crazyflie to connect to
uri = 'radio://0/100/2M/E7E7E7E708'
# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

def take_off(cf, position):
    take_off_time = 1.0
    sleep_time = 0.1
    steps = int(take_off_time / sleep_time)
    vz = position[2] / take_off_time

    print(f'take off at {position[2]}')

    for i in range(steps):
        cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        time.sleep(sleep_time)




if __name__ == '__main__':
    cflib.crtp.init_drivers()

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf

        take_off(cf, (0, 0, 0.5))

        time.sleep(1.0)

        print('Landing')

        take_off(cf, (0, 0, 0))

        time.sleep(1)

        #return 
        print('Landed')

        cf.commander.send_stop_setpoint()

        cf.commander.send_notify_setpoint_stop()

        time.sleep(0.1)    
        print('Disconnected')






        





















        