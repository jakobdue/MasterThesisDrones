import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

# URI to the Crazyflie to connect to
uri = 'radio://0/100/2M/E7E7E7E708'

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


def wait_for_param_download(cf, timeout=10.0):
    """
    Wait until the parameter TOC has been downloaded.
    If you query cf.param.toc.toc too early it will be empty.
    """
    t0 = time.time()
    while len(cf.param.toc.toc) == 0:
        if time.time() - t0 > timeout:
            raise RuntimeError("Param TOC not downloaded (timeout)")
        time.sleep(0.05)


def print_params_containing(cf, needle: str):
    # group -> {name -> ParamTocElement}
    for group, names in cf.param.toc.toc.items():
        for name in names.keys():
            full = f"{group}.{name}"
            if needle.lower() in full.lower():
                print(full)


def debug_params(cf):
    print("---- params containing 'posCtl' ----")
    print_params_containing(cf, "posCtl")
    print("---- params containing 'VelMax' ----")
    print_params_containing(cf, "VelMax")
    print("---- params containing 'xy' ----")
    print_params_containing(cf, "xy")
    print("---- params containing 'vel' ----")
    print_params_containing(cf, "vel")
    print("---- params containing 'max' ----")
    print_params_containing(cf, "max")


def print_param_groups(cf):
    print("Param groups:")
    for group in sorted(cf.param.toc.toc.keys()):
        print("  ", group)


def simple_log(scf, logconf):
    with SyncLogger(scf, logconf) as logger:
        for log_entry in logger:
            timestamp = log_entry[0]
            data = log_entry[1]
            logconf_name = log_entry[2]
            print('[%d][%s]: %s' % (timestamp, logconf_name, data))
            break


if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
    lg_stab.add_variable('stabilizer.roll', 'float')
    lg_stab.add_variable('stabilizer.pitch', 'float')
    lg_stab.add_variable('stabilizer.yaw', 'float')

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf

        # IMPORTANT: wait until params are actually downloaded
        wait_for_param_download(cf)

        # Now these will show real content
        print_param_groups(cf)
        debug_params(cf)

        # Your existing log example
        simple_log(scf, lg_stab)
