from sys import byteorder
from .radar import *
from .radar_consts import *
from .ld2450_consts import *
from math import sqrt

class LD2450(Radar):
    def __init__(self, port, baud_rate=PARAM_DEFAULT_BAUD, timeout=1, verbosity=logging.DEBUG) -> None:
        super().__init__(port, baud_rate=baud_rate, timeout=timeout, verbosity=verbosity)



    def calc_distance(self, target_data):
        x = int.from_bytes(target_data[0:2], byteorder='little', signed=True)
        y = int.from_bytes(target_data[2:4], byteorder='little', signed=True)
        speed = int.from_bytes(target_data[4:6], byteorder='little', signed=False)
        distance_resolution = int.from_bytes(target_data[6:8], byteorder='little', signed=False)

        x = x if x >=0 else -2**15 - x
        y = y if y >=0 else -2**15 - y
        speed = speed if speed >= 0 else -2**15 - speed
        distance = sqrt(x*x + y*y)
        return x, y, speed, distance_resolution, distance

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

    def get_data_frame(self):
        logging.debug("Getting raw CHILD dataframe")

        try:
            # b = self.ser.read_until(bytes.fromhex(REF_DATA_CRC))
            b = self.ser.read_until(b'\x55\xCC')
            b = b.split(b'\xAA\xFF\x03\x00')[1].split(b'\x55\xCC')[0]
            target1 = b[REF_MIN_TARGET1:REF_MAX_TARGET1]
            target2 = b[REF_MIN_TARGET2:REF_MAX_TARGET2]
            target3 = b[REF_MIN_TARGET3:REF_MAX_TARGET3]
        except Exception as e:
            logging.error("THRWING EXCEPTION!!!!")
            logging.error(str(e))
            return
        return (target1, target2, target3)


    def get_data(self):
        logging.debug(f"get_data returning {self.last_detection}")
        with self._lock:
            if not self.last_detection:
                logging.warning("Data is empty, have you started the radar yet?")
            return self.last_detection


    def get_radar_data(self):
        logging.debug("Getting raw dataframe")

        # target1, target2, target3 = self.get_data_frame()
        ret = self.get_data_frame()
        while not ret:
            logging.debug("Retry get_data_frame")
            ret = self.get_data_frame()

        logging.debug(f"Returning dataframes")
        ret2 = []
        for i in range(len(ret)):
            ret2.append(self.calc_distance(ret[i]))
        return ret2



    def poll_radar(self):
        while not self._stop_event.is_set():
            with self._lock:
                self.last_detection = self.get_radar_data()
            time.sleep(0.1)


