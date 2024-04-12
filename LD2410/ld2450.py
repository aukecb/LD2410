from sys import byteorder
from .radar import *
from .radar_consts import *
from .ld2450_consts import *

class LD2450(Radar):
    def __init__(self, port, baud_rate=PARAM_DEFAULT_BAUD, timeout=1, verbosity=logging.DEBUG) -> None:
        super().__init__(port, baud_rate=baud_rate, timeout=timeout, verbosity=verbosity)

    # Enable single target tracking
    def set_single_target_tracking(self):
        logging.info("Reading detection parameters")
        ret = self.send_command(PARAM_SINGLE_TARGET_TRACKING)

        return ret


    # Enable multi target tracking
    def set_multi_target_tracking(self):
        logging.info("Reading detection parameters")
        ret = self.send_command(PARAM_MULTI_TARGET_TRACKING)

        return ret


    # Get the current region filter
    def read_region_filter(self):
        logging.info("Reading detection parameters")
        ret = self.send_command(READ_REGION_FILTER)

        return ret

    # Set the regional filter
    # Exclude targets in rectangular area delimited by two diaognal vertex coordinates
    # Example: [[(-100,100),(100,100)], [(100,100),(200,200)]]
    def set_region_filter(self, config:list, filter_type:int):
        logging.info("Setting regional filter with the following config: %s and mode %d" % (str(config), filter_type))
        command = WRITE_REGION_FILTER
        command += REGION_FILTER_TYPE_LOOKUP[filter_type] 

        for p1, p2 in config:
            for x, y in p1, p2:
                command += x.to_bytes(2, byteorder='little', signed=True).hex()
                command += y.to_bytes(2, byteorder='little', signed=True).hex()

        if len(command) < REGION_MSG_LENGTH:
            command += (REGION_MSG_LENGTH - len(command))*'0'
        ret = self.send_command(command)

        return ret


