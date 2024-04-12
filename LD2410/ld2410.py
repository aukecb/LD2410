from .radar import *
from .ld2410_consts import *
import logging


class LD2410(Radar):
    def __init__(self, port, baud_rate=PARAM_DEFAULT_BAUD, timeout=1, verbosity=logging.DEBUG) -> None:
        super().__init__(port, baud_rate=baud_rate, timeout=timeout, verbosity=verbosity)


    # Validate that the input is within a valid range
    @staticmethod
    def validate_range(input, lower, upper):
        if input in range(lower, upper):
            return True
        else:
            input_name = f'{input=}'.split('=')[0]
            raise Exception(f"{input_name} {input} is not a valid setting, please pick a value between {lower} and {upper}")


    # Configure Detection Gates and Detect Duration
    def edit_detection_params(self, moving_max_gate, static_max_gate, timeout):
        logging.info("Editing detection parameters")
        self.validate_range(moving_max_gate, GATE_MIN, GATE_MAX+1)
        self.validate_range(static_max_gate, GATE_MIN, GATE_MAX+1)
        self.validate_range(timeout, TIMEOUT_MIN, TIMEOUT_MAX+1)

        command = CMD_PARAM_EDIT + PARAM_MAX_MOVING_GATE + self.int_to_4b(moving_max_gate) \
                      + PARAM_MAX_STATIC_GATE + self.int_to_4b(static_max_gate) \
                      + PARAM_EMPTY_DURATION + self.int_to_4b(timeout)
        
        self.send_command(command)

    # Read the currently configured parameters 
    #
    # Returns 3 arrays, 
    # 
    # first array: [moving gate threshold, static gate threshold, empty timeout]
    # second array: [moving gate sens 0.... moving gate sens 8]
    # third array: [static gate sens 0.... static gate sens 8]

    def read_detection_params(self):
        # Send command to retrieve parameters
        logging.info("Reading detection parameters")
        ret = self.send_command(CMD_PARAM_READ)
        
        # Process response
        # Threshold Params
        thresholds = [ret[REF_MAX_MOVING_GATE], ret[REF_MAX_STATIC_GATE], ret[REF_EMPTY_TIMEOUT]]


        # Movement Gate Sensitivities
        move_sens = [int(byte) for byte in ret[REF_MOVING_GATE_SENS_0:REF_MOVING_GATE_SENS_8]]
        # Static Gate Sensitivities
        static_sens = [int(byte) for byte in ret[REF_STATIC_GATE_SENS_0:REF_STATIC_GATE_SENS_8]]
        
        logging.debug(f"Thresholds:{thresholds}, Movement Sens:{move_sens}, Static Sens:{static_sens}")
        return thresholds, move_sens, static_sens
        
    # Enable Engineering Mode
    # Adds energy level of each gate to the radar output
    def enable_engineering_mode(self):
        logging.info("Enabling engineering mode")
        self.eng_mode = True
        self.send_command(CMD_ENG_MODE_ENABLE)
        
    # Disable Engineering Mode
    def disable_engineering_mode(self):
        logging.info("Disabling engineering mode")
        self.eng_mode = False
        self.send_command(CMD_ENG_MODE_DISABLE)

    # Configure Gate Movement and Static Sensitivities
    def edit_gate_sensitivity(self, gate, moving_sens, static_sens):
        logging.info("Editing gate sensitivity")
        self.validate_range(gate, GATE_MIN, GATE_MAX+1)
        self.validate_range(moving_sens, SENS_MIN, SENS_MAX+1)

        if gate == 1 or gate == 2:
            logging.warning("You cannot set gate 1 or 2 static sensitivity to anything other than 0")
            self.validate_range(static_sens, 0, 1)
        else:
            self.validate_range(static_sens, SENS_MIN, SENS_MAX+1)

        command = CMD_GATE_SENS_EDIT + PARAM_GATE_SELECT + self.int_to_4b(gate) \
                      + PARAM_MOVING_GATE_WORD + self.int_to_4b(moving_sens) \
                      + PARAM_STATIC_GATE_WORD + self.int_to_4b(static_sens)
        
        self.send_command(command)


    # Sets 3 lists (standard, move_energies, static_energies). If engineering mode is disabled, second and third list is empty
    def get_radar_data(self):
        logging.debug("Getting raw dataframe")

        # Keep trying until a successful read
        ret = self.get_data_frame()
        while not ret:
            ret = self.get_data_frame()

        move_energies = None
        static_energies = None
        
        target_type = ret[REF_TARGET_TYPE] # 0 No Target, 1 Moving, 2 Static, 3 Static + Moving
        
        moving_target_dist = int(bytes(reversed(ret[REF_TARGET_MOVE_DIST_HEAD:REF_TARGET_MOVE_DIST_TAIL+1])).hex(), 16)
        moving_target_energy = ret[REF_TARGET_MOVE_ENERGY]
        
        static_target_dist = int(bytes(reversed(ret[REF_TARGET_STATIC_DIST_HEAD:REF_TARGET_STATIC_DIST_TAIL+1])).hex(), 16)
        static_target_energy = ret[REF_TARGET_STATIC_ENERGY]

        detection_dist = ret[REF_DETECT_DIST]

        standard_frame = [target_type, moving_target_dist, moving_target_energy, static_target_dist, static_target_energy, detection_dist]
        
        logging.debug(f"Returning dataframes {standard_frame}, {move_energies}, {static_energies}")

        if self.eng_mode:
            # Movement Gate Sensitivities
            move_energies = [int(byte) for byte in ret[REF_MOVING_GATE_ENERGY_0:REF_MOVING_GATE_ENERGY_8+1]]
            # Static Gate Sensitivities
            static_energies = [int(byte) for byte in ret[REF_STATIC_GATE_ENERGY_0:REF_STATIC_GATE_ENERGY_8+1]]

        return standard_frame, move_energies, static_energies
    

    def poll_radar(self):
        while not self._stop_event.is_set():
            with self._lock:
                self.last_detection = self.get_radar_data()
            time.sleep(0.1)

    def start(self):
        logging.info("Radar polling started")
        self._worker_thread = threading.Thread(target=self.poll_radar)
        self._worker_thread.start()
        self._stop_event.clear()
        time.sleep(1) # Allow for a 1s init time

    def stop(self):
        if not self._stop_event.is_set():
            logging.info("Radar polling stopped")
            self._stop_event.set()
            self._worker_thread.join()
        else:
            logging.debug("Calling stop() but radar isn't running. This is normal.")
