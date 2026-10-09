import threading
import time

import serial
import serial.tools.list_ports


class ZWHAND:
    def __init__(self, port, baud=115200, lqs_id=0x01):
        self.ser = serial.Serial()
        self.port = port
        self.baud_rate = baud
        self.ser.timeout = 2
        self.receive_timeout = 0.1
        self._lock = threading.Lock()
        self.lqs_id = lqs_id                                        # 灵巧手ID
        self.initial_lqs_id = 0x01                                  # 初始ID
        self.motor_count = 6                                        # 电机数量
        self.max_position = 1000                                    # 最大位置
        self.baud_rate_list = ['9600', '115200', '921600', '2000000']             # 仲裁波特率列表
        self.initial_baud_rate = self.baud_rate_list[1]             # 初始波特率
        # 写数据地址
        self.SET_ID_ADDRESS = 0x00                                  # 设置ID地址
        self.SET_BAUD_ADDRESS = 0x01                                # 设置波特率地址
        self.CLEAR_ERROR_ADDRESS = 0x02                             # 清除错误地址
        self.SET_POWER_OFF_SAVE_ADDRESS = 0x03                      # 设置掉电保存地址
        self.FACTORY_DATA_RESET_ADDRESS = 0x04                      # 恢复出厂设置地址
        self.SINGLE_MOTOR_CALIBRATION_ADDRESS = 0x05                # 单电机校准地址
        self.CONTROL_JOINT_MOTOR_ABSOLUTE_ADDRESS = 0x0B            # 控制关节电机绝对位置地址
        self.CONTROL_JOINT_MOTOR_RELATIVE_ADDRESS = 0x11            # 控制关节电机相对位置地址
        self.SET_MOTOR_STOP_ADDRESS = 0x17                          # 设置电机停止地址
        self.SET_SPEED_ADDRESS = 0x1D                               # 设置电机速度地址
        self.SET_CURRENT_ADDRESS = 0x23                             # 设置电机电流地址
        self.ALL_MOTOR_CALIBRATION_ADDRESS = 0x29                   # 全电机校准地址
        self.HAND_TEST_ADDRESS = 0x2A                               # 整手测试地址
        self.SET_RADIAN_ADDRESS = 0x2B                              # 设置电机弧度地址
        self.SET_FINGER_CTRL_MODE = 0x31                            # 设置手指控制模式地址
        self.SET_PVT_CTRL_PARAM = 0x37                              # 设置PVT控制参数地址
        self.SET_PC_CTRL_PARAM = 0x49                               # 设置PC控制参数地址
        # 读取数据地址
        self.INITIALIZE_DATA_ADDRESS = 0x00                         # 初始化数据地址
        self.BOOTLOADER_VERSION_ADDRESS = 0x01                      # boot版本地址
        self.HARDWARE_VERSION_ADDRESS = 0x02                        # 硬件版本地址
        self.SOFTWARE_VERSION_ADDRESS = 0x03                        # 软件版本地址
        self.HALL_ERROR_ADDRESS = 0x04                              # 霍尔错误地址
        self.DEVICE_VOLTAGE_ADDRESS = 0x0A                          # 设备电压地址
        self.MOVING_RANGE_ADDRESS = 0x0B                            # 电机运动范围地址
        self.MOTOR_LOCK_STATE_ADDRESS = 0x11                        # 电机堵转状态地址
        self.MOTOR_ANGLE_ADDRESS = 0x17                             # 电机当前位置角度地址
        self.MOTOR_SPEED_ADDRESS = 0x1D                             # 电机当前速度地址
        self.MOTOR_CURRENT_ADDRESS = 0x23                           # 电机当前电流地址
        self.PHASE_LOSS_FAULT_ADDRESS = 0x29                        # 电机相位丢失故障地址
        self.CUR_SAMP_ERROR_ADDRESS = 0x2F                          # 电流采样错误地址
        self.MOTOR_RADIAN_ADDRESS = 0x35                            # 电机当前弧度地址
        self.JOINT_ABS_MAX_ADDRESS = 0x3B                           # 关节绝对最大位置地址
        self.CUST_NUMBER_ADDRESS = 0x41                             # 客户编号地址
        self.ELE_SKIN_FORCE_INFOR_ADDRESS = 0x42                    # 电子皮肤力信息地址
        self.MOTOR_TEMPERATURE_ADDRESS = 0x48                       # 电机温度地址
        self.HAND_TYPE_ADDRESS = 0x4E                               # 左右手区分
        self.open_start_device()

    def open_start_device(self):
        try:
            self.ser = serial.Serial()
            self.ser.port = self.port
            self.ser.baudrate = self.baud_rate
            self.ser.timeout = 2
            self.ser.open()
            if not self.ser.is_open:
                return False
            return True
        except serial.SerialException as e:
            print(f"[串口异常] 无法打开端口 {self.ser.port}: {e}")
            return False
        except Exception as e:
            print(f"[未知错误] 初始化串口时发生异常: {e}")
            return False

    def close_device(self):
        self.ser.close()
        return 1

    def modbus_is_open(self):
        """
        检查串口是否打开。

        此函数返回一个布尔值，指示串口是否已打开。
        """
        return self.ser.is_open

    def modbus_in_waiting(self):
        """
        获取输入缓冲区中的字节数。

        此函数返回输入缓冲区中的字节数。
        """
        return self.ser.in_waiting

    def modbus_input_buffer(self):
        """
        清空输入缓冲区。

        此函数将清空串口输入缓冲区，并返回清空的字节数。
        """
        if self.ser and self.ser.is_open:
            return self.ser.reset_input_buffer()
        else:
            return 0

    def modbus_read_data(self, receive_data_len):
        data = self.ser.read(receive_data_len)
        return data

    def modbus_write_data(self, data):
        """
        发送数据到串口。

        参数:
        data -- 要发送的数据，字节数组

        此函数将给定的数据发送到串口。
        """
        if self.ser and self.ser.is_open:
            try:
                # 尝试发送数据
                send_num = self.ser.write(data)
                return send_num
            except Exception as e:
                # 可记录异常日志，例如：logging.error(f"串口写入失败: {e}")
                return 0

    #  modbus接收数据
    def modbus_data_receive(self, detect_data, receive_data_len):
        """
        接收串口数据函数。

        该函数从串口接收数据，并根据数据格式解析数据。如果数据有效，它将处理数据并发出信号。
        """
        detect_len = len(detect_data)
        start_time = time.time()
        while True:
            if time.time() - start_time > self.receive_timeout:
                available_num = self.ser.in_waiting
                #  读取数据
                data = self.ser.read(available_num)
                #  界面数据更新
                text = ''.join(f"{byte:02X} " for byte in data)
                print('超时', '数据接收失败', text)
                break
            try:
                if self.ser and self.ser.is_open:
                    available_num = self.ser.in_waiting
                else:
                    # 可根据实际需求记录日志或设置默认值
                    available_num = 0
            except Exception as e:
                # 可记录异常日志，例如：logging.error(f"串口读取失败: {e}")
                available_num = 0

            # 如果有数据等待读取
            if available_num >= receive_data_len:
                #  读取数据
                data = self.ser.read(available_num)
                #  界面数据更新
                text = ''.join(f"{byte:02X} " for byte in data)
                print('接收数据:', text)
                #  接受数据校验
                data_list = list(data)
                if detect_len and detect_len <= len(data_list):
                    for num in range(len(detect_data)):
                        if data_list[num] != detect_data[num]:
                            print('接收数据校验失败！')
                            return False
                return data_list
                #  截取数据段
        return False

    #  modbus的CRC校验和计算
    @staticmethod
    def modbus_crc(data):
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        crc_high = (crc >> 8) & 0xFF
        crc_low = crc & 0xFF
        data += [crc_low, crc_high]
        cmd = bytes(data)
        return cmd

    #  发送单地址指令
    def send_single_data(self, address, data, signed=False):
        """
        发送单个地址的数据指令到指定设备。

        参数:
        : address -- 寄存器地址，int
        : data -- 要写入的数据，int（-32768 ~ 32767 for 2 bytes）
        : param signed: 数据有无符号标志

        此函数负责将给定的数据打包成指令，并通过串口发送到指定设备。
        """
        with self._lock:
            if self.ser and self.ser.is_open:
                # 清空输入缓冲区，避免接收旧数据
                self.ser.reset_input_buffer()
                # 将整数转换为2字节的二进制数据
                bytes_value = data.to_bytes(2, byteorder='big', signed=signed)
                step_list = list(bytes_value)
                # 构造单电机控制指令
                single_motor_cmd = [self.lqs_id, 0x06, 0x00, address, step_list[0], step_list[1]]
                # 添加CRC校验
                cmd = self.modbus_crc(single_motor_cmd)
                try:
                    # 打印发送的数据
                    print("发送数据:", ''.join(f"{byte:02X} " for byte in list(cmd)))
                    send_num = self.ser.write(cmd)
                    if send_num == len(single_motor_cmd):
                        # 成功发送后接收数据
                        receive_data = self.modbus_data_receive(single_motor_cmd[:6], 8)
                        return receive_data
                        # return 1
                    else:
                        # 发送失败提示
                        print("发送失败")
                except Exception as e:
                    # 异常处理，打印发送失败的原因
                    print("发送失败:", e)
                return False

    #  发送多地址指令
    def send_multiple_data(self, start_address, data, signed=False):
        """
        发送多个数据到指定的灵巧手。

        :param start_address: 数据写入的起始地址，决定了数据在设备内存中的写入位置。
        :param data: 待发送的数据列表，包含多个要写入设备内存的整数数据。
        :param signed: 数据有无符号标志
        :return: 如果数据发送成功并正确接收到回应数据，则返回接收的数据；否则返回0。
        """
        with self._lock:
            # 检查串口是否已打开
            if self.ser and self.ser.is_open:
                # 清空输入缓冲区，避免接收旧数据
                self.ser.reset_input_buffer()
                # 计算地址长度，即需要写入的数据数量
                address_len = len(data)
                # 初始化命令数据数组，长度为7加上两倍的地址长度（每个地址对应两个字节的数据）
                cmd_data = [0] * (7 + address_len*2)

                #  灵巧手ID
                cmd_data[0] = self.lqs_id
                #  写入服务
                cmd_data[1] = 0x10
                #  起始地址
                cmd_data[2] = 0x0
                cmd_data[3] = start_address
                #  地址数
                cmd_data[4] = 0x0
                cmd_data[5] = address_len
                #  数据位
                cmd_data[6] = address_len * 2
                #  数据
                for i in range(len(data)):
                    # 将整数转换为2字节的二进制数据，以适应Modbus协议
                    bytes_value = data[i].to_bytes(2, byteorder='big', signed=signed)
                    step_list = list(bytes_value)
                    cmd_data[i * 2 + 7] = step_list[0]
                    cmd_data[i * 2 + 8] = step_list[1]

                # 添加CRC校验码
                cmd = self.modbus_crc(cmd_data)
                # test_cmd_data = cmd_data[:]
                # test_cmd_data[0] += 1
                # test_cmd = self.modbus_crc(test_cmd_data)

                try:
                    # 打印发送的数据
                    print("发送数据:", ''.join(f"{byte:02X} " for byte in list(cmd)))
                    # 发送数据
                    send_num = self.ser.write(cmd)

                    # 检查发送的数据长度是否与预期相符
                    if send_num == len(cmd_data):
                        # 接收设备返回的数据，并返回解析后的数据
                        receive_data = self.modbus_data_receive(cmd_data[:6], 8)
                        # time.sleep(0.05)
                        # print("发送数据:", ''.join(f"{byte:02X} " for byte in test_cmd))
                        # self.ser.write(test_cmd)
                        return receive_data
                        # return 1
                    else:
                        # 发送失败
                        print("发送失败")
                except Exception as e:
                    # 捕获异常并打印错误信息
                    print("发送失败:", e)

                # 如果发送失败或接收数据失败，则返回0
                return False

    #  读取多地址数据指令
    def read_multiple_data(self, start_address, address_len, signed=True):
        """
        读取多个地址的数据。

        构建一个Modbus指令，用于从指定的起始地址读取指定长度的数据。

        参数:
        : start_address: 起始地址。
        : address_len: 要读取的数据长度。
        : param signed: 数据有无符号标志

        返回:
        - 成功读取的数据，如果发生错误或数据未成功发送，则返回0。
        """
        with self._lock:
            if self.ser and self.ser.is_open:
                # 构建命令数据
                cmd_data = [0] * 6
                cmd_data[0] = self.lqs_id
                cmd_data[1] = 0x04
                cmd_data[2] = 0x0
                cmd_data[3] = start_address
                cmd_data[4] = 0x0
                cmd_data[5] = address_len
                # 添加CRC校验
                cmd = self.modbus_crc(cmd_data)
                try:
                    # 直接清空输入缓冲区
                    self.ser.reset_input_buffer()
                    # 打印发送的数据
                    print("发送数据:", ''.join(f"{byte:02X} " for byte in list(cmd)))
                    # 发送命令
                    send_num = self.ser.write(cmd)
                    # 检查发送的数据长度是否正确
                    if send_num == len(cmd_data):
                        # 接收返回的数据
                        receive_data = self.modbus_data_receive([self.lqs_id, 0x04, address_len*2], address_len*2+5)
                        if receive_data:
                            data_list = receive_data[3:3+address_len*2]
                            handle_data = [int.from_bytes(data_list[i:i+2], byteorder='big', signed=signed) for i in range(0, len(data_list), 2)]
                            return handle_data
                except Exception as e:
                    print("发送失败:", e)
                return False

    def set_id(self, new_id):
        """
        设置设备ID。

        参数:
        - new_id: 设置的设备ID
        - 类型：int
        - 范围：1-255

        返回:
        - 成功:响应报文
        - 失败:False。
        """
        # 参数验证
        if not isinstance(new_id, int):
            print(f"Failed to set ID: <{new_id}>，Input parameter type error!")
            return False
        if not (0x01 <= new_id <= 0xFF):
            print(f"Failed to set ID: <{new_id}>，The parameter is outside the valid range of 1 to 255!")
            return False
        try:
            send_ret = self.send_single_data(self.SET_ID_ADDRESS, new_id)
            if send_ret:
                self.lqs_id = new_id
                return send_ret
            else:
                print(f"Failed to set ID: <{new_id}>")
                return False
        except Exception as e:
            print(f"Failed to set ID: <{new_id}>，Data transmission error: {str(e)}")
            return False

    def set_baud(self, baud_order=0x02):
        """
        设置设备波特率。

        波特率档位：1-'9600', 2-'115200', 3-'921600', 4-'2000000'

        参数:
        - baud_order: 波特率顺序。
        - 类型：int
        - 范围：1-4

        返回:
        - 成功:初始化ZWHAND类对象
        - 失败:False。
        """

        # 参数验证
        if not isinstance(baud_order, int):
            print(f"Failed to set the baud rate: {baud_order} - Input parameter type error!")
            return False
        if baud_order < 1 or baud_order > len(self.baud_rate_list):
            print(f"Failed to set the baud rate: {baud_order} - Parameter out of valid range!")
            return False
        index = baud_order - 1
        try:
            if self.ser.baudrate != baud_order:
                send_ret = self.send_single_data(self.SET_BAUD_ADDRESS, baud_order)
                if send_ret:
                    # 更新波特率设置
                    self.baud_rate = self.baud_rate_list[index]
                    self.close_device()
                    self.ser.open()
                    return True
                else:
                    print(f"Failed to set the baud rate: {self.baud_rate_list[index]}")
                    return False
            return True
        except Exception as e:
            print(f"Abnormal baud rate setting: {self.baud_rate_list[index]} - {str(e)}")
            return False

    def set_error_clear(self):
        """
        清除错误。

        返回:
        - 成功:响应报文
        - 失败:False。
        """
        try:
            send_ret = self.send_single_data(self.CLEAR_ERROR_ADDRESS, 1)
            if send_ret:
                return send_ret
            else:
                print("Failed to clear errors")
                return False
        except Exception as e:
            print(f"Failed to clear errors - {str(e)}")
            return False

    def set_power_off_save(self, save_type=0x02):
        """
        设置掉电保存。

        参数:
        - save_type: 1-配置保存(设备id、波特率等)，2-参数保存(运动速度、电流等)。
        - 类型：int
        - 范围：1-2

        返回:
        - 成功:响应报文
        - 失败:False。
        """
        # 验证参数范围
        if not isinstance(save_type, int) or save_type < 1 or save_type > 2:
            print(f"Failed to save function settings: {save_type} - Parameter out of valid range!")
            return False
        try:
            send_ret = self.send_single_data(self.SET_POWER_OFF_SAVE_ADDRESS, save_type)
            if send_ret is not False:
                return send_ret
            else:
                print(f"Failed to save function settings: {save_type}")
                return False
        except Exception as e:
            print(f"Failed to save function settings: {save_type} - Error: {str(e)}")
            return False

    def set_factory_data_reset(self):
        """
        恢复出厂设置。

        返回:
        - 成功:初始化ZWHAND类对象
        - 失败:False。
        """
        try:
            send_ret = self.send_single_data(self.FACTORY_DATA_RESET_ADDRESS, 1)
            if send_ret:
                # 保存原始状态用于回滚
                original_lqs_id = self.lqs_id
                original_baud_rate = self.baud_rate

                try:
                    # 重置本地状态变量到初始值
                    self.lqs_id = self.initial_lqs_id
                    self.baud_rate = self.initial_baud_rate
                    self.ser.baudrate = self.initial_baud_rate
                except Exception as e:
                    # 回滚状态
                    self.lqs_id = original_lqs_id
                    self.baud_rate = original_baud_rate
                    self.ser.baudrate = original_baud_rate
                    print(f"Error occurred when restoring factory settings: {e}")
                finally:
                    self.close_device()
                    self.ser.open()
                    if not self.ser.is_open:
                        return False
                    return True
            else:
                print("Failed to restore factory settings!")
                return False
        except Exception as e:
            print(f"Error occurred when sending command: {e}")
            return False

    def set_single_motor_speed(self, motor_number, speed):
        """
        设置单个电机速度。

        参数:
        - motor_number: 电机序号
        - 类型：int
        - 范围：1-6
        - speed: 速度档位
        - 类型：int
        - 范围：1-100

        返回:
        - 成功:响应报文
        - 失败:False。
        """
        # 参数类型检查
        if not isinstance(motor_number, int) or not isinstance(speed, int):
            print(f"Failed to set speed: Input parameter type error!")
            return False
        # 参数范围检查
        if motor_number < 1 or motor_number > self.motor_count:
            print(f"Failed to set speed: {motor_number} - Parameter out of valid range!")
            return False
        if speed < 1 or speed > 100:
            print(f"Failed to set speed: {speed} - Speed out of valid range!")
            return False
        address = self.SET_SPEED_ADDRESS + motor_number - 1
        try:
            send_ret = self.send_single_data(address, speed)
            if send_ret:
                return send_ret
            else:
                print(f"Failed to set speed: {speed}")
                return False
        except Exception as e:
            print(f"Abnormal speed setting: {str(e)}")
            return False

    def set_all_motor_speed(self, speed_list):
        """
        设置所有电机速度。

        参数:
        - speed_list: 速度档位列表
        - 类型：list[int] * 6 or tuple[int] * 6 or int
        - 范围：0-100

        返回:
        - 成功:响应报文
        - 失败:False。
        """
        # 验证参数类型和范围
        if isinstance(speed_list, int):
            speed_list = [speed_list]*self.motor_count
        if not isinstance(speed_list, list) and not isinstance(speed_list, tuple):
            print(f"Failed to set speed: Input parameter type error!")
            return False
        for speed in speed_list:
            if not isinstance(speed, int):
                print(f"Failed to set speed: Input parameter type error!")
                return False
            if speed < 0 or speed > 100:
                print(f"Failed to set speed: {speed} - Speed out of valid range!")
                return False
        try:
            set_ret = self.send_multiple_data(self.SET_SPEED_ADDRESS, speed_list)
            if set_ret:
                return set_ret
            else:
                print(f"Failed to set speed: {speed_list}")
                return False
        except Exception as e:
            print(f"Abnormal speed setting: {str(e)}")
            return False

    def set_single_motor_current(self, motor_number, current):
        """
        设置单个电机电流。

        参数:
        - motor_number: 电机序号
        - 类型：int
        - 范围：1-6
        - current: 电流档位
        - 类型：int
        - 范围：1-100

        返回:
        - 成功:响应报文
        - 失败:False。
        """
        # 验证参数类型和范围
        if not isinstance(motor_number, int) or not isinstance(current, int):
            print(f"Failed to set current: Input parameter type error!")
            return False
        if motor_number < 1 or motor_number > self.motor_count:
            print(f"Failed to set current: {motor_number} - Parameter out of valid range!")
            return False
        if current < 1 or current > 100:
            print(f"Failed to set current: {current} - Current out of valid range!")
            return False
        address = self.SET_CURRENT_ADDRESS + motor_number - 1
        try:
            send_ret = self.send_single_data(address, current)
            if send_ret:
                return send_ret
            else:
                print(f"Failed to set current: {current}")
                return False
        except Exception as e:
            print(f"Failed to set current: {e}")
            return False

    def set_all_motor_current(self, current_list):
        """
        设置所有电机电流。

        参数:
        - current: 电流档位列表
        - 类型：list[int] * 6 or tuple[int] * 6 or int
        - 范围：1-100

        返回:
        - 成功:响应报文
        - 失败:False。
        """
        # 验证参数类型和范围
        if isinstance(current_list, int):
            current_list = [current_list]*self.motor_count
        if not isinstance(current_list, list) and not isinstance(current_list, tuple):
            print(f"Failed to set current: Input parameter type error!")
            return False
        for current in current_list:
            if not isinstance(current, int):
                print(f"Failed to set current: Input parameter type error!")
                return False
            if current < 1 or current > 100:
                print(f"Failed to set current: {current} - Current out of valid range!")
                return False
        try:
            set_ret = self.send_multiple_data(self.SET_CURRENT_ADDRESS, current_list)
            if set_ret:
                return set_ret
            else:
                print(f"Failed to set current: {current_list}")
                return False
        except Exception as e:
            print(f"Failed to set current: {e}")
            return False

    def set_single_motor_stop(self, motor_number):
        """
        设置单个电机紧急停止。

        参数:
        - motor_number: 电机序号
        - 类型：int
        - 范围：1-6

        返回:
        - 成功:响应报文
        - 失败:False。
        """
        if not isinstance(motor_number, int):
            print(f"Failed to set motor stop: {motor_number} - Input parameter type error!")
            return False
        if motor_number < 1 or motor_number > self.motor_count:
            print(f"Failed to set motor stop: {motor_number} - Parameter out of valid range!")
            return False
        address = self.SET_MOTOR_STOP_ADDRESS + motor_number - 1
        try:
            send_ret = self.send_single_data(address, 1)
            if send_ret:
                return True
            else:
                print(f"Failed to set motor stop: {motor_number}")
                return False
        except Exception as e:
            print(f"Failed to set motor stop: {e}")
            return False

    def set_all_motor_stop(self, stop_flag_list=None):
        """
        设置所有电机紧急停止。

        参数:
        - stop_flag_list: 电机急停标志列表，1-停止，0-默认
        - 类型：list[int] * 6

        返回:
        - 成功:响应报文
        - 失败:False。
        """
        try:
            if stop_flag_list is None:
                stop_flag_list = [1]*self.motor_count
            if not isinstance(stop_flag_list, list) and not isinstance(stop_flag_list, tuple):
                print(f"Failed to set all motor stop: Input parameter type error!")
                return False
            for stop_flag in stop_flag_list:
                if not isinstance(stop_flag, int):
                    print(f"Failed to set all motor stop: Input parameter type error!")
                    return False
                if stop_flag != 0 and stop_flag != 1:
                    print(f"Failed to set all motor stop: {stop_flag} - Parameter out of valid range!")
                    return False
            set_ret = self.send_multiple_data(self.SET_MOTOR_STOP_ADDRESS, stop_flag_list)
            if set_ret:
                return set_ret
            else:
                print(f"Failed to set all motor stop")
                return False
        except Exception as e:
            print(f"Failed to set all motor stop: {e}")
            return False

    def set_single_motor_absolute(self, motor_number, joint_angle):
        """
        设置单个电机绝对位置角度挡位。

        参数:
        - motor_number: 电机序号
        - 类型：int
        - 范围：1-6
        - joint_angle: 角度挡位
        - 类型：int
        - 范围：0-1000

        返回:
        - 成功:响应报文
        - 失败:False。
        """
        if not isinstance(motor_number, int) or not isinstance(joint_angle, int):
            print(f"Failed to set motor angle: Input parameter type error!")
            return False
        if motor_number < 1 or motor_number > self.motor_count:
            print(f"Failed to set motor angle: {motor_number} - Input parameter out of valid range")
            return False
        if joint_angle < 0 or joint_angle > self.max_position:
            print(f"Failed to set motor angle: {joint_angle} - Input parameter out of valid range")
            return False
        address = self.CONTROL_JOINT_MOTOR_ABSOLUTE_ADDRESS + motor_number - 1
        try:
            send_ret = self.send_single_data(address, joint_angle)
            if send_ret:
                return send_ret
            else:
                print(f"Failed to set motor angle: {joint_angle}")
                return False
        except Exception as e:
            print(f"Failed to set motor angle: {e}")
            return False

    def set_all_motor_absolute(self, joint_angle_list):
        """
        设置所有关节电机绝对位置角度挡位。

        参数:
        - joint_angle_list: 关节角度挡位列表
        - 类型：[int]*6
        - 范围：0-1000

        返回:
        - 成功:响应报文
        - 失败:False。
        """
        if not isinstance(joint_angle_list, list) and not isinstance(joint_angle_list, tuple):
            print(f"Failed to set motor angle: Input parameter type error!")
            return False
        for item in joint_angle_list:
            if not isinstance(item, int):
                print(f"Failed to set motor angle: Input parameter type error!")
                return False
            if item < 0 or item > 1000:
                print(f"Failed to set motor angle: {item} - Input parameter out of valid range")
                return False
        try:
            set_ret = self.send_multiple_data(self.CONTROL_JOINT_MOTOR_ABSOLUTE_ADDRESS, joint_angle_list)
            if set_ret:
                return set_ret
            else:
                print(f"Failed to set all motor angle: {joint_angle_list}")
                return False
        except Exception as e:
            print(f"Failed to set all motor angle: {e}")
            return False

    def set_single_motor_relative(self, motor_number, joint_angle):
        """
        设置单个关节电机相对位置角度挡位。

        参数:
        - motor_number: 电机序号
        - 类型：int
        - 范围：1-6
        - joint_angle: 角度挡位
        - 类型：int
        - 范围：-1000-1000

        返回:
        - 成功:响应报文
        - 失败:False。
        """
        if not isinstance(motor_number, int) or not isinstance(joint_angle, int):
            print(f"Failed to set motor angle: Input parameter type error!")
            return False
        if motor_number < 1 or motor_number > self.motor_count:
            print(f"Failed to set motor angle: {motor_number} - Input parameter out of valid range!")
            return False
        if joint_angle < -1000 or joint_angle > 1000:
            print(f"Failed to set motor angle: {joint_angle} - Input parameter out of valid range!")
            return False
        address = self.CONTROL_JOINT_MOTOR_RELATIVE_ADDRESS + motor_number - 1
        try:
            send_ret = self.send_single_data(address, joint_angle, True)
            if send_ret:
                return send_ret
            else:
                print(f"Failed to set motor angle: {joint_angle}")
                return False
        except Exception as e:
            print(f"Failed to set motor angle: {e}")
            return False

    def set_all_motor_relative(self, joint_angle_list):
        """
        设置所有关节电机相对位置角度挡位。

        参数:
        - joint_angle_list: 关节角度挡位列表
        - 类型：[int]*6
        - 范围：-1000-1000

        返回:
        - 成功:响应报文
        - 失败:False。
        """
        if not isinstance(joint_angle_list, list) and not isinstance(joint_angle_list, tuple):
            print(f"Failed to set motor angle: Input parameter type error!!")
            return False
        for item in joint_angle_list:
            if not isinstance(item, int):
                print(f"Failed to set motor angle: Input parameter type error!!")
                return False
            if item < -1000 or item > 1000:
                print(f"Failed to set motor angle: {item} - Input parameter out of valid range!")
                return False
        try:
            set_ret = self.send_multiple_data(self.CONTROL_JOINT_MOTOR_RELATIVE_ADDRESS, joint_angle_list, True)
            if set_ret:
                return set_ret
            else:
                print(f"Failed to set all motor angle: {joint_angle_list}")
                return False
        except Exception as e:
            print(f"Failed to set all motor angle: {e}")
            return False

    def set_single_motor_calibration(self, motor_number):
        """
        单个关节电机零位校准。

        参数:
        - motor_number: 电机序号
        - 类型：int
        - 范围：1-6

        返回:
        - 成功:响应报文
        - 失败:False。
        """
        if not isinstance(motor_number, int):
            print(f"Failed to calibrate single motor zero position: {motor_number} - Input parameter type error!")
            return False
        if motor_number < 1 or motor_number > self.motor_count:
            print(f"Failed to calibrate single motor zero position: {motor_number} - Input parameter out of valid range!")
            return False
        address = self.SINGLE_MOTOR_CALIBRATION_ADDRESS + motor_number - 1
        try:
            send_ret = self.send_single_data(address, 1)
            if send_ret:
                return send_ret
            else:
                print(f"Failed to calibrate single motor zero position: {motor_number}")
                return False
        except Exception as e:
            print(f"Failed to calibrate single motor zero position: {e}")
            return False

    def set_all_motor_calibration(self):
        """
        设置全关节电机(整手)零位校准。

        返回:
        - 成功:响应报文
        - 失败:False。
        """
        try:
            send_ret = self.send_single_data(self.ALL_MOTOR_CALIBRATION_ADDRESS, 1)
            if send_ret:
                return send_ret
            else:
                print(f"Failed to calibrate all motor zero position!")
                return False
        except Exception as e:
            print(f"Failed to calibrate all motor zero position: {e}")
            return False

    def set_hand_test(self, enable: bool = True):
        """
        整手测试功能。

        参数:
            enable (bool): True - 打开测试 (发送1)
                        False - 关闭测试 (发送0)

        返回:
            - 成功: 响应报文
            - 失败: False
        """
        try:
            value = 1 if enable else 0
            send_ret = self.send_single_data(self.HAND_TEST_ADDRESS, value)
            if send_ret:
                return send_ret
            else:
                action = "start" if enable else "stop"
                print(f"Failed to {action} hand test!")
                return False
        except Exception as e:
            action = "start" if enable else "stop"
            print(f"Failed to {action} hand test: {e}")
            return False

    def set_all_motor_radian(self, radian_list):
        """
        设置所有电机弧度。
        - 系数：0.1

        参数:
        - radian_list: 弧度列表
        - 类型：list[float] * 6
        - 范围：0-3600

        返回:
        - 成功:响应报文
        - 失败:False。
        """
        # 验证参数类型和范围
        if not isinstance(radian_list, list) or len(radian_list) > self.motor_count:
            print(f"Failed to set radian: Input parameter type error!")
            return False
        for radian in radian_list:
            if not isinstance(radian, int):
                print(f"Failed to set radian: Input parameter type error!")
                return False
            if radian < 0 or radian > 3600:
                print(f"Failed to set radian: {radian} - Radian out of valid range!")
                return False
        try:
            set_ret = self.send_multiple_data(self.SET_RADIAN_ADDRESS, radian_list)
            if set_ret:
                return set_ret
            else:
                print(f"Failed to set all motor radian")
                return False
        except Exception as e:
            print(f"Failed to set all motor radian: {e}")
            return False

    def set_finger_control_mode(self, mode_list=1):
        """
        设置手指控制模式。

        参数:
        - mode_list: 指定手指控制模式列表
        - 类型：[int]*6
        - 值：0-2, 0:PC；1:PSC；2:PVT

        返回:
        - 获取成功:响应报文
        - 获取失败:False。
        """
        if isinstance(mode_list, int):
            mode_list = [mode_list] * self.motor_count
        if not isinstance(mode_list, list) or len(mode_list) != self.motor_count:
            print(f"Failed to set finger control mode: Input parameter type error!")
        for item in mode_list:
            if not isinstance(item, int):
                print(f"Failed to set finger control mode: Input parameter type error!")
                return False
            if item < 0 or item > 2:
                print(f"Failed to set finger control mode: {item} - Input parameter out of valid range!")
                return False
        try:
            send_ret = self.send_multiple_data(self.SET_FINGER_CTRL_MODE, mode_list)
            if send_ret:
                return send_ret
            else:
                print(f"Failed to set finger control mode: {mode_list}")
                return False
        except Exception as e:
            print(f"Failed to set finger control mode: {e}")
            return False

    def set_pvt_control_param(self, pvt_param_list):
        """
        设置PVT控制参数。

        参数:
        - pvt_param_list: 指定PVT控制参数列表
        - 类型：[P, V, T]*6
        - V与T的参数同一时间只能给其中一个赋值，另一个参数保持为0，如V非0，T为0，则模式为PV模式，否则为PT模式，如果VT都设置，默认为PV模式
        - 范围：P（0-1000）， V（0-33000）， T（450-5000）

        返回:
        - 获取成功:响应报文
        - 获取失败:False。
        """
        if not isinstance(pvt_param_list, list) or len(pvt_param_list) != self.motor_count * 3:
            print(f"Failed to set PVT control parameter: Input parameter type error!")
            return False
        for item in pvt_param_list:
            if not isinstance(item, int):
                print(f"Failed to set PVT control parameter: Input parameter type error!")
                return False
        try:
            send_ret = self.send_multiple_data(self.SET_PVT_CTRL_PARAM, pvt_param_list)
            if send_ret:
                return send_ret
            else:
                print(f"Failed to set PVT control parameter: {pvt_param_list}")
                return False
        except Exception as e:
            print(f"Failed to set PVT control parameter: {e}")
            return False

    def set_pc_control_param(self, pc_param_list):
        """
        设置PC控制参数。

        参数:
        - pc_param_list: 获取PC控制参数列表
        - 类型：[int]*6
        - 值：[-8000,8000]

        返回:
        - 获取成功:响应报文
        - 获取失败:False。
        """
        if not isinstance(pc_param_list, list) or len(pc_param_list) != self.motor_count:
            print(f"Failed to set PC control parameter: Input parameter type error!")
            return False
        for item in pc_param_list:
            if not isinstance(item, int):
                print(f"Failed to set PC control parameter: Input parameter type error!")
                return False
            if item < -8000 or item > 8000:
                print(f"Failed to set PC control parameter: {item} - Input parameter out of valid range!")
                return False
        try:
            send_ret = self.send_multiple_data(self.SET_PC_CTRL_PARAM, pc_param_list, True)
            if send_ret:
                return send_ret
            else:
                print(f"Failed to set PC control parameter: {pc_param_list}")
                return False
        except Exception as e:
            print(f"Failed to set PC control parameter: {e}")
            return False

    def get_initialize_state(self):
        """
        获取设备初始化状态。

        返回:
        - 获取成功:1-初始化状态
        - 获取失败:False。
        """
        try:
            send_ret = self.read_multiple_data(self.INITIALIZE_DATA_ADDRESS, 1)
            if send_ret:
                return send_ret[0]
            else:
                print(f"Failed to get initialization state!")
                return False
        except Exception as e:
            print(f"Failed to get initialization state: {e}")
            return False

    def get_bootloader_version(self):
        """
        获取设备bootloader版本。

        返回:
        - 获取成功:XX，XX为十进制格式的bootloader版本
        - 获取失败:False。
        """
        try:
            send_ret = self.read_multiple_data(self.BOOTLOADER_VERSION_ADDRESS, 1)
            if send_ret:
                return send_ret[0]
            else:
                print(f"Failed to get bootloader version!")
                return False
        except Exception as e:
            print(f"Failed to get bootloader version: {e}")
            return False

    def get_hardware_version(self):
        """
        获取设备硬件版本。

        返回:
        - 获取成功:XX，XX为十进制格式的硬件版本
        - 获取失败:False
        """
        try:
            send_ret = self.read_multiple_data(self.HARDWARE_VERSION_ADDRESS, 1)
            if send_ret:
                new_data = send_ret[0].to_bytes(2, byteorder='big', signed=False)
                hex_data = f'P{new_data[1]:02X} {new_data[0]:02X}'
                return hex_data
            else:
                print(f"Failed to get hardware version!")
                return False
        except Exception as e:
            print(f"Failed to get hardware version: {e}")
            return False

    def get_software_version(self):
        """
        获取设备软件版本。

        返回:
        - 获取成功:XX，XX为十进制格式的软件版本
        - 获取失败:False。
        """
        try:
            send_ret = self.read_multiple_data(self.SOFTWARE_VERSION_ADDRESS, 1)
            if send_ret:
                return send_ret[0]
            else:
                print(f"Failed to get software version!")
                return False
        except Exception as e:
            print(f"Failed to get software version: {e}")
            return False

    def get_motor_hall_fault(self):
        """
        获取电机Hall故障状态。

        返回:
        - 获取成功:数据列表[XX]*6，XX为十进制格式的设备错误码
        - 获取失败:False。
        """
        try:
            send_ret = self.read_multiple_data(self.HALL_ERROR_ADDRESS, 6)
            if send_ret:
                return send_ret
            else:
                print("Failed to get motor Hall fault state!")
                return False
        except Exception as e:
            print(f"Failed to get motor Hall fault state: {e}")
            return False

    def get_device_voltage(self):
        """
        获取设备电压。

        电压系数：0.001
        电压单位：V

        返回:
        - 获取成功:XX，XX*0.001为十进制格式的设备电压
        - 获取失败:False。
        """
        try:
            send_ret = self.read_multiple_data(self.DEVICE_VOLTAGE_ADDRESS, 1)
            if send_ret:
                return send_ret[0]
            else:
                print("Failed to get device voltage!")
                return False
        except Exception as e:
            print(f"Failed to get device voltage: {e}")
            return False

    def get_motor_locked_state(self):
        """
        获取所有关节电机堵转状态。

        电机堵转状态：1-堵转，0-正常

        返回:
        - 获取成功:数据列表[XX]*6，XX为十进制格式的电机堵转状态
        - 获取失败:False。
        """
        try:
            send_ret = self.read_multiple_data(self.MOTOR_LOCK_STATE_ADDRESS, 6)
            if send_ret:
                return send_ret
            else:
                print("Failed to get motor locked state!")
                return False
        except Exception as e:
            print(f"Failed to get motor locked state: {e}")
            return False

    def get_motor_real_angle(self):
        """
        获取所有关节电机实际角度挡位:0-1000。

        返回:
        - 获取成功:数据列表[XX]*6，XX为十进制格式的电机实际角度挡位
        - 获取失败:False。
        """
        try:
            send_ret = self.read_multiple_data(self.MOTOR_ANGLE_ADDRESS, 6)
            if send_ret:
                return send_ret
            else:
                print("Failed to get motor real angle!")
                return False
        except Exception as e:
            print(f"Failed to get motor real angle: {e}")
            return False

    def get_all_motor_speed(self):
        """
        获取所有关节电机实际速度。

        返回:
        - 获取成功:数据列表[XX]*6，XX为十进制格式的电机实际速度
        - 获取失败:False。
        """
        try:
            send_ret = self.read_multiple_data(self.MOTOR_SPEED_ADDRESS, 6)
            if send_ret:
                return send_ret
            else:
                print("Failed to get all motor speed!")
                return False
        except Exception as e:
            print(f"Failed to get all motor speed: {e}")
            return False

    def get_all_motor_current(self):
        """
        获取所有关节电机实际电流。

        电流系数：1
        电流单位：mA

        返回:
        - 获取成功:数据列表[XX]*6，XX为十进制格式的电机实际电流
        - 获取失败:False。
        """
        try:
            send_ret = self.read_multiple_data(self.MOTOR_CURRENT_ADDRESS, 6)
            if send_ret:
                return send_ret
            else:
                print("Failed to get all motor current!")
                return False
        except Exception as e:
            print(f"Failed to get all motor current: {e}")
            return False

    def get_phase_loss_fault(self):
        """
        获取所有电机缺相状态。

        电机相位丢失状态：1-相位丢失，0-正常

        返回:
        - 获取成功:数据列表[XX]*6，XX为十进制格式的电机相位丢失状态
        - 获取失败:False。
        """
        try:
            send_ret = self.read_multiple_data(self.PHASE_LOSS_FAULT_ADDRESS, 6)
            if send_ret:
                return send_ret
            else:
                print("Failed to get phase loss fault!")
                return False
        except Exception as e:
            print(f"Failed to get phase loss fault: {e}")
            return False

    def get_cur_samp_fault(self):
        """
        获取所有电机电流采样错误状态。

        电机电流采样错误状态：1-电流采样错误，0-正常

        返回:
        - 获取成功:数据列表[XX]*6，XX为十进制格式的电机电流采样错误状态
        - 获取失败:False。
        """
        try:
            send_ret = self.read_multiple_data(self.CUR_SAMP_ERROR_ADDRESS, 6)
            if send_ret:
                return send_ret
            else:
                print("Failed to get cur samp fault!")
                return False
        except Exception as e:
            print(f"Failed to get cur samp fault: {e}")
            return False

    def get_all_motor_radian(self):
        """
        获取所有关节电机实际角度弧度。
        - 系数：0.1

        返回:
        - 获取成功:数据列表[XX]*6，XX为十进制格式的电机实际角度弧度
        - 获取失败:False。
        """
        try:
            send_ret = self.read_multiple_data(self.MOTOR_RADIAN_ADDRESS, self.motor_count)
            if send_ret:
                new_data = [data * 0.1 for data in send_ret]
                return new_data
            else:
                print("Failed to get all motor radian!")
                return False
        except Exception as e:
            print(f"Failed to get all motor radian: {e}")
            return False

    def get_all_motor_max_radian(self):
        """
        获取所有关节电机实际角度弧度最大值。
        - 系数：0.1

        返回:
        - 获取成功:数据列表[XX]*6，XX为十进制格式的电机实际角度弧度最大值
        - 获取失败:False。
        """
        try:
            send_ret = self.read_multiple_data(self.JOINT_ABS_MAX_ADDRESS, self.motor_count)
            if send_ret:
                return send_ret
            else:
                print("Failed to get all motor max radian!")
                return False
        except Exception as e:
            print(f"Failed to get all motor max radian: {e}")
            return False

    def get_cust_number(self):
        """
        获取客户编号。

        返回:
        - 获取成功:数据列表XX，XX为十进制格式的客户编号
        - 获取失败:False。
        """
        try:
            send_ret = self.read_multiple_data(self.CUST_NUMBER_ADDRESS, 1)
            if send_ret:
                return send_ret[0]
            else:
                print("Failed to get cust number!")
                return False
        except Exception as e:
            print(f"Failed to get cust number: {e}")
            return False

    def get_ele_skin_force_infor(self):
        """
        获取电子皮肤力数据。
        - 数据顺序：小指->无名指->中指->食指->大拇指->手掌
        - 系数：0.01

        返回:
        - 获取成功:数据列表[XX]*6，XX为十进制格式的电子皮肤力数据
        - 获取失败:False。
        """
        try:
            send_ret = self.read_multiple_data(self.ELE_SKIN_FORCE_INFOR_ADDRESS, self.motor_count)
            if send_ret:
                return send_ret
            else:
                print("Failed to get ele skin force infor!")
                return False
        except Exception as e:
            print(f"Failed to get ele skin force infor: {e}")
            return False

    def get_all_motor_temperature(self):
        """
        获取所有关节电机实际温度。
        - 系数：0.1

        返回:
        - 获取成功:数据列表[XX]*6，XX为十进制格式的电机实际温度
        - 获取失败:False。
        """
        try:
            send_ret = self.read_multiple_data(self.MOTOR_TEMPERATURE_ADDRESS, self.motor_count)
            if send_ret:
                return send_ret
            else:
                print("Failed to get all motor temperature!")
                return False
        except Exception as e:
            print(f"Failed to get all motor temperature: {e}")
            return False
    
    def get_hand_type(self):
        """
        获取手类型。
    
        返回:
        - 获取成功: 0-右手, 1-左手
        - 获取失败: False。
        """
        try:
            send_ret = self.read_multiple_data(self.HAND_TYPE_ADDRESS, 1)
            if send_ret:
                return send_ret[0]
            else:
                print("Failed to get hand type!")
                return False
        except Exception as e:
            print(f"Failed to get hand type: {e}")
            return False

'''
if __name__ == '__main__':
    hand = ZWHAND('COM5', 115200, 0x01)
    init_state = hand.get_initialize_state()
    if init_state:
        print("Device initialization successful!")
        ret = hand.set_hand_test()
        if ret:
            print("test successful!")
            time.sleep(20)
            hand.set_all_motor_absolute([0, 1000, 1000, 1000, 1000, 1000])
        else:
            print("test failed!")
    else:
        print("Device initialization failed!")
'''


if __name__ == '__main__':
    hand = ZWHAND('COM5', 115200, 0x01)      
    init_state = hand.get_initialize_state()
    if init_state:
        print("Device initialization successful!")
        
        # 读取手类型
        hand_type = hand.get_hand_type()
        if hand_type is not False:
            if hand_type == 0:
                print("Current hand type: Right hand")
            else:
                print("Current hand type: Left hand")
        
        ret = hand.set_hand_test()
        if ret:
            print("test successful!")
            time.sleep(20)
            hand.set_all_motor_absolute([0, 1000, 1000, 1000, 1000, 1000])
        else:
            print("test failed!")
    else:
        print("Device initialization failed!")
