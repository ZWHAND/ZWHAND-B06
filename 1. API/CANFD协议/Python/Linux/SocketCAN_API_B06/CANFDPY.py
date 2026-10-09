import threading
import time
import logging
import can

class PCANFD:
    def __init__(self, channel: str = 'can0', arb_baud_rate: int = 500000, data_baud_rate: int = 1000000,
                 lqs_id: int = 0x01):

        self._setup_logging()
        self.lqs_id = lqs_id
        self.initial_lqs_id = 0x01
        self.arb_baud_rate = arb_baud_rate
        self.data_baud_rate = data_baud_rate
        self.channel = channel
        self.bus = None
        self.is_connect = False
        self._lock = threading.Lock()
        self.motor_count = 6
        self.max_position = 1000
        self.write_single_fun_type = 0x06
        self.write_multipe_fun_type = 0x10
        self.read_fun_type = 0x04
        self.receive_detect_time = 0.05
        self.timeout = 0.1

        self.arb_baud_rate_list = ['500000', '1000000']
        self.initial_arb_baud_rate = '500000'
        self.data_baud_rate_list = ['500000', '1000000', '2000000', '5000000']
        self.initial_data_baud_rate = '1000000'
        self.BAUD_RATE_LEVELS = ['500K/500K', '500K/1000K', '500K/2000K', '1000K/5000K', '1000K/1000K']

        # 写寄存器地址
        self.SET_ID_ADDRESS = 0x00
        self.SET_BAUD_ADDRESS = 0x01
        self.CLEAR_ERROR_ADDRESS = 0x02
        self.SET_POWER_OFF_SAVE_ADDRESS = 0x03
        self.FACTORY_DATA_RESET_ADDRESS = 0x04
        self.SINGLE_MOTOR_CALIBRATION_ADDRESS = 0x05
        self.CONTROL_JOINT_MOTOR_ABSOLUTE_ADDRESS = 0x0B
        self.CONTROL_JOINT_MOTOR_RELATIVE_ADDRESS = 0x11
        self.SET_MOTOR_STOP_ADDRESS = 0x17
        self.SET_SPEED_ADDRESS = 0x1D
        self.SET_CURRENT_ADDRESS = 0x23
        self.ALL_MOTOR_CALIBRATION_ADDRESS = 0x29
        self.HAND_TEST_ADDRESS = 0x2A
        self.SET_RADIAN_ADDRESS = 0x2B
        self.SET_FINGER_CTRL_MODE = 0x31
        self.SET_PVT_CTRL_PARAM = 0x37
        self.SET_PC_CTRL_PARAM = 0x49

        # 读寄存器地址
        self.INITIALIZE_DATA_ADDRESS = 0x00
        self.BOOTLOADER_VERSION_ADDRESS = 0x01
        self.HARDWARE_VERSION_ADDRESS = 0x02
        self.SOFTWARE_VERSION_ADDRESS = 0x03
        self.HALL_ERROR_ADDRESS = 0x04
        self.DEVICE_VOLTAGE_ADDRESS = 0x0A
        self.MOVING_RANGE_ADDRESS = 0x0B
        self.MOTOR_LOCK_STATE_ADDRESS = 0x11
        self.MOTOR_ANGLE_ADDRESS = 0x17
        self.MOTOR_SPEED_ADDRESS = 0x1D
        self.MOTOR_CURRENT_ADDRESS = 0x23
        self.PHASE_LOSS_FAULT_ADDRESS = 0x29
        self.CUR_SAMP_ERROR_ADDRESS = 0x2F
        self.MOTOR_RADIAN_ADDRESS = 0x35
        self.JOINT_ABS_MAX_ADDRESS = 0x3B
        self.CUST_NUMBER_ADDRESS = 0x41
        self.ELE_SKIN_FORCE_INFOR_ADDRESS = 0x42
        self.MOTOR_TEMPERATURE_ADDRESS = 0x48
        self.HAND_TYPE_ADDRESS = 0x4E

    def _setup_logging(self):
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.setLevel(logging.DEBUG)

        if not self.logger.handlers:
            stream_handler = logging.StreamHandler()
            stream_handler.setFormatter(formatter)
            self.logger.addHandler(stream_handler)

    # --------------- 工具方法 ---------------

    @staticmethod
    def int_to_bytes(value: int) -> list[int]:
        return [(value >> 8) & 0xFF, value & 0xFF]

    @staticmethod
    def bytes_to_int(data: list[int], offset: int) -> int:
        if offset + 1 >= len(data):
            return 0
        value = (data[offset] << 8) | data[offset + 1]
        if value & 0x8000:
            value -= 0x10000
        return value

    @staticmethod
    def calculate_crc(data: list[int]) -> list[int]:
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return data + [crc & 0xFF, (crc >> 8) & 0xFF]

    # --------------- 连接管理 ---------------

    def connect(self) -> bool:
        try:
            self.bus = can.Bus(
                channel=self.channel,
                bustype='socketcan',
                fd=True,
                bitrate=self.arb_baud_rate,
                dbitrate=self.data_baud_rate,
            )
            self.is_connect = True
            self.logger.info(f'Connect to CAN on {self.channel}')
            return True
        except Exception as e:
            self.logger.error(f'Failed to connect CAN: {e}')
            self.is_connect = False
            return False

    def disconnect(self):
        if self.bus:
            self.bus.shutdown()
            self.is_connect = False
            self.logger.info("Disconnected")

    def clear_receive_buffer(self):
        if not self.is_connect:
            self.logger.warning("Not connected to CAN")
            return

        cleared_count = 0
        try:
            while True:
                msg = self.bus.recv(timeout=0.001)
                if msg is None:
                    break
                cleared_count += 1

            if cleared_count > 0:
                self.logger.info(f'Cleared {cleared_count} messages from receive buffer')
            else:
                self.logger.debug("Receive buffer was already empty")

        except Exception as e:
            self.logger.error(f"Error clearing receive buffer: {e}")

    # --------------- 基础收发 ---------------

    def send_message(self, data: list[int], is_extended_id: bool = False) -> bool:
        if not self.is_connect:
            self.logger.error("Not connect to CAN bus")
            return False

        canfd_cmd_len = len(data)
        if canfd_cmd_len > 64:
            self.logger.error("Data length exceeds maximum CAN FD payload (64 bytes)")
            return False

        # 按长度对齐填充 (与C++一致)
        padded_data = list(data)
        if 8 < canfd_cmd_len <= 24:
            while len(padded_data) % 4 != 0:
                padded_data.append(0)
        elif 24 < canfd_cmd_len <= 32:
            while len(padded_data) % 8 != 0:
                padded_data.append(0)
        elif 32 < canfd_cmd_len <= 64:
            while len(padded_data) % 16 != 0:
                padded_data.append(0)

        try:
            msg = can.Message(
                arbitration_id=self.lqs_id,
                data=padded_data,
                is_extended_id=is_extended_id,
                is_fd=True,
                bitrate_switch=True,
            )
            self.clear_receive_buffer()
            self.bus.send(msg)
            self.logger.info(f"CAN FD message sent: ID={hex(self.lqs_id)}, Len={len(padded_data)}")
            return True
        except can.CanError as e:
            self.logger.error(f"Failed to send CAN FD message: {e}")
            return False
        except Exception as e:
            self.logger.error(f"Unexpected error sending message: {e}")
            return False

    def receive_message(self, fun_type: int) -> list[int]:

        if not self.is_connect:
            self.logger.error("Not connected to CAN bus")
            return []

        start_time = time.time()
        while (time.time() - start_time) < self.receive_detect_time:
            msg = self.bus.recv(timeout=0.001)
            if msg is None:
                time.sleep(0.005)
                continue

            if msg.arbitration_id != self.lqs_id:
                continue

            data = list(msg.data)
            if fun_type == self.read_fun_type:
                if len(data) > 2 and data[1] == fun_type:
                    receive_number = data[2] + 5
                    if receive_number <= len(data):
                        result = data[3:3 + data[2]]
                        self.logger.info(f"Received read response: {[hex(b) for b in result]}")
                        return result

            elif fun_type in (self.write_single_fun_type, self.write_multipe_fun_type):
                if len(data) > 1 and data[1] == fun_type:
                    result = data[:min(8, len(data))]
                    self.logger.info(f"Received write response: {[hex(b) for b in result]}")
                    return result

            time.sleep(0.005)

        self.logger.warning(f"Receive timeout, fun_type={hex(fun_type)}")
        return []

    # --------------- 读写命令 ---------------

    def write_single_address(self, start_address: int, value: int) -> bool:
        with self._lock:
            cmd_data = [
                self.lqs_id,
                self.write_single_fun_type,
                0x00,
                start_address,
            ]
            cmd_data.extend(self.int_to_bytes(value))
            cmd_data = self.calculate_crc(cmd_data)
            return self.send_message(cmd_data)

    def write_multiple_address(self, start_address: int, data: list[int]) -> bool:
        with self._lock:
            addr_len = len(data)
            cmd_data = [
                self.lqs_id,
                self.write_multipe_fun_type,
                0x00,
                start_address,
                0x00,
                addr_len,
                addr_len * 2,
            ]
            for v in data:
                cmd_data.extend(self.int_to_bytes(v))
            cmd_data = self.calculate_crc(cmd_data)
            return self.send_message(cmd_data)

    def read_command(self, start_address: int, address_len: int) -> list[int]:
        with self._lock:
            cmd_data = [
                self.lqs_id,
                self.read_fun_type,
                0x00,
                start_address,
                0x00,
                address_len,
            ]
            cmd_data = self.calculate_crc(cmd_data)

            if not self.send_message(cmd_data):
                return []

            received = self.receive_message(self.read_fun_type)
            if not received:
                return []

            result = []
            for i in range(0, len(received) - 1, 2):
                result.append(self.bytes_to_int(received, i))
            return result

    # --------------- 配置功能 ---------------

    def set_id(self, new_id: int) -> bool:
        if new_id < 1 or new_id > 255:
            return False
        if self.write_single_address(self.SET_ID_ADDRESS, new_id):
            self.lqs_id = new_id
            return True
        return False

    def set_baud(self, baud_order: int) -> bool:
        if baud_order < 1 or baud_order > 5:
            return False
        return self.write_single_address(self.SET_BAUD_ADDRESS, baud_order)

    def set_error_clear(self) -> bool:
        return self.write_single_address(self.CLEAR_ERROR_ADDRESS, 1)

    def set_power_off_save(self, save_type: int) -> bool:
        if save_type < 1 or save_type > 2:
            return False
        return self.write_single_address(self.SET_POWER_OFF_SAVE_ADDRESS, save_type)

    def set_factory_data_reset(self) -> bool:
        if not self.write_single_address(self.FACTORY_DATA_RESET_ADDRESS, 1):
            return False
        self.lqs_id = self.initial_lqs_id
        self.disconnect()
        return self.connect()

    # --------------- 单电机控制 ---------------

    def set_single_motor_speed(self, motor_number: int, speed: int) -> bool:
        if motor_number < 1 or motor_number > self.motor_count or speed < 1 or speed > 100:
            return False
        return self.write_single_address(self.SET_SPEED_ADDRESS + motor_number - 1, speed)

    def set_all_motor_speed(self, speed_list: list[int]) -> bool:
        if len(speed_list) != self.motor_count:
            return False
        for s in speed_list:
            if s < 1 or s > 100:
                return False
        return self.write_multiple_address(self.SET_SPEED_ADDRESS, speed_list)

    def set_single_motor_current(self, motor_number: int, current: int) -> bool:
        if motor_number < 1 or motor_number > self.motor_count or current < 1 or current > 100:
            return False
        return self.write_single_address(self.SET_CURRENT_ADDRESS + motor_number - 1, current)

    def set_all_motor_current(self, current_list: list[int]) -> bool:
        if len(current_list) != self.motor_count:
            return False
        for c in current_list:
            if c < 1 or c > 100:
                return False
        return self.write_multiple_address(self.SET_CURRENT_ADDRESS, current_list)

    def set_single_motor_stop(self, motor_number: int) -> bool:
        if motor_number < 1 or motor_number > self.motor_count:
            return False
        return self.write_single_address(self.SET_MOTOR_STOP_ADDRESS + motor_number - 1, 1)

    def set_all_motor_stop(self, stop_flag_list: list[int]) -> bool:
        if len(stop_flag_list) != self.motor_count:
            return False
        for f in stop_flag_list:
            if f not in (0, 1):
                return False
        return self.write_multiple_address(self.SET_MOTOR_STOP_ADDRESS, stop_flag_list)

    def set_single_motor_absolute(self, motor_number: int, joint_angle: int) -> bool:
        if motor_number < 1 or motor_number > self.motor_count:
            return False
        if joint_angle < 0 or joint_angle > 1000:
            return False
        return self.write_single_address(
            self.CONTROL_JOINT_MOTOR_ABSOLUTE_ADDRESS + motor_number - 1, joint_angle)

    def set_all_motor_absolute(self, joint_angle_list: list[int]) -> bool:
        if len(joint_angle_list) != self.motor_count:
            return False
        for a in joint_angle_list:
            if a < 0 or a > 1000:
                return False
        return self.write_multiple_address(self.CONTROL_JOINT_MOTOR_ABSOLUTE_ADDRESS, joint_angle_list)

    def set_single_motor_relative(self, motor_number: int, joint_angle: int) -> bool:
        if motor_number < 1 or motor_number > self.motor_count:
            return False
        if joint_angle < -1000 or joint_angle > 1000:
            return False
        return self.write_single_address(
            self.CONTROL_JOINT_MOTOR_RELATIVE_ADDRESS + motor_number - 1, joint_angle)

    def set_all_motor_relative(self, joint_angle_list: list[int]) -> bool:
        if len(joint_angle_list) != self.motor_count:
            return False
        for a in joint_angle_list:
            if a < -1000 or a > 1000:
                return False
        return self.write_multiple_address(self.CONTROL_JOINT_MOTOR_RELATIVE_ADDRESS, joint_angle_list)

    def set_single_motor_calibration(self, motor_number: int) -> bool:
        if motor_number < 1 or motor_number > self.motor_count:
            return False
        return self.write_single_address(self.SINGLE_MOTOR_CALIBRATION_ADDRESS + motor_number - 1, 1)

    def set_all_motor_calibration(self) -> bool:
        return self.write_single_address(self.ALL_MOTOR_CALIBRATION_ADDRESS, 1)

    def set_hand_test(self, enable: int) -> bool:
        if enable not in (0, 1):
            return False
        return self.write_single_address(self.HAND_TEST_ADDRESS, enable)

    def set_all_motor_angle(self, angle_list: list[int]) -> bool:
        if len(angle_list) != self.motor_count:
            return False
        return self.write_multiple_address(self.SET_RADIAN_ADDRESS, angle_list)

    def set_finger_control_mode(self, mode_list: list[int]) -> bool:
        if len(mode_list) != self.motor_count:
            return False
        for m in mode_list:
            if m < 0 or m > 2:
                return False
        return self.write_multiple_address(self.SET_FINGER_CTRL_MODE, mode_list)

    def set_pvt_control_param(self, param_list: list[int]) -> bool:
        if len(param_list) != 18:
            return False
        for i in range(6):
            pos = param_list[i * 3]
            spd = param_list[i * 3 + 1]
            tim = param_list[i * 3 + 2]
            if pos < 0 or pos > 1000:
                return False
            if spd < 0 or spd > 33000:
                return False
            if tim < 450 or tim > 5000:
                return False
            if spd != 0 and tim != 0:
                return False
        return self.write_multiple_address(self.SET_PVT_CTRL_PARAM, param_list)

    def set_pc_control_param(self, param_list: list[int]) -> bool:
        if len(param_list) != self.motor_count:
            return False
        for p in param_list: 
            if p < -7500 or p > 7500:
                return False
        return self.write_multiple_address(self.SET_PC_CTRL_PARAM, param_list)

    # --------------- 读取功能 ---------------

    def get_initialize_state(self) -> int:
        result = self.read_command(self.INITIALIZE_DATA_ADDRESS, 1)
        return result[0] if result else 0

    def get_bootloader_version(self) -> int:
        result = self.read_command(self.BOOTLOADER_VERSION_ADDRESS, 1)
        return result[0] if result else 0

    def get_hardware_version(self) -> int:
        result = self.read_command(self.HARDWARE_VERSION_ADDRESS, 1)
        return result[0] if result else 0

    def get_software_version(self) -> int:
        result = self.read_command(self.SOFTWARE_VERSION_ADDRESS, 1)
        return result[0] if result else 0

    def get_device_error(self) -> list[int]:
        return self.read_command(self.HALL_ERROR_ADDRESS, 6)

    def get_device_voltage(self) -> float:
        result = self.read_command(self.DEVICE_VOLTAGE_ADDRESS, 1)
        return float(result[0]) * 0.001 if result else 0.0

    def get_motor_locked_state(self) -> list[int]:
        return self.read_command(self.MOTOR_LOCK_STATE_ADDRESS, self.motor_count)

    def get_motor_real_position(self) -> list[int]:
        return self.read_command(self.MOTOR_ANGLE_ADDRESS, self.motor_count)

    def get_all_motor_speed(self) -> list[int]:
        return self.read_command(self.MOTOR_SPEED_ADDRESS, self.motor_count)

    def get_all_motor_current(self) -> list[int]:
        return self.read_command(self.MOTOR_CURRENT_ADDRESS, self.motor_count)

    def get_phase_loss_fault(self) -> list[int]:
        return self.read_command(self.PHASE_LOSS_FAULT_ADDRESS, self.motor_count)

    def get_cur_samp_fault(self) -> list[int]:
        return self.read_command(self.CUR_SAMP_ERROR_ADDRESS, self.motor_count)

    def get_all_motor_angle(self) -> list[float]:
        raw = self.read_command(self.MOTOR_RADIAN_ADDRESS, self.motor_count)
        return [float(v) * 0.1 for v in raw]

    def get_abs_max_position(self) -> list[int]:
        return self.read_command(self.MOVING_RANGE_ADDRESS, self.motor_count)

    def get_joint_abs_max(self) -> list[float]:
        raw = self.read_command(self.JOINT_ABS_MAX_ADDRESS, self.motor_count)
        return [float(v) * 0.1 for v in raw]

    def get_customer_number(self) -> int:
        result = self.read_command(self.CUST_NUMBER_ADDRESS, 1)
        return result[0] if result else 0

    def get_skin_force(self) -> list[float]:
        raw = self.read_command(self.ELE_SKIN_FORCE_INFOR_ADDRESS, self.motor_count)
        return [float(v) * 0.01 for v in raw]

    def get_motor_temperature(self) -> list[float]:
        raw = self.read_command(self.MOTOR_TEMPERATURE_ADDRESS, self.motor_count)
        return [float(v) * 0.1 for v in raw]

    def get_hand_type(self) -> int:
        result = self.read_command(self.HAND_TYPE_ADDRESS, 1)
        return result[0] if result else 0
