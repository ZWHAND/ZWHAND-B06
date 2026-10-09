import threading

from zlgcan_main import *


class ZWHAND:
    def __init__(self, lqs_id=0x01, arb_baud_rate='500000', data_baud_rate='1000000', zlgcan_type=1, canfd_channel=1):
        """
        初始化CAN设备配置参数

        参数:
            lqs_id (int): 设备ID，默认值为0x01
            arb_baud_rate (str): 仲裁波特率，默认值为'500000'
            data_baud_rate (str): 数据波特率，默认值为'1000000'
            zlgcan_type (int): ZLG CAN设备类型，默认值为0--智嵌物联，1--周立功
            canfd_channel (int): CAN通道数，默认值为1，或2
        """
        self.lqs_id = lqs_id                                # 设备ID
        self.initial_lqs_id = 0x01                          # 初始设备ID
        self.can_type = zlgcan_type                         # ZLG CAN设备类型
        self.canfd_channel = canfd_channel                  # CAN通道数
        self.arb_baud_rate = str(arb_baud_rate)                  # 仲裁波特率
        self.data_baud_rate = str(data_baud_rate)                # 数据波特率

        self.zlg_can = ZCAN(zlgcan_type)                    # 实例化ZCAN类
        self.handle = ''                                    # 设备句柄
        self.chn_handle = ''                                # 通道句柄
        self.device_index = 0                               # 设备索引
        self.reserved = 0                                   # 保留参数
        self.chn_index = 0                                  # 通道索引
        self.max_position = 1000                            # 最大位置

        self.can_is_open = False                            # 设备是否打开
        self._lock = threading.Lock()
        # self.receive_thread = threading.Thread(target=self.receive_messages)
        self.motor_count = 6                                # 电机数量
        self.write_single_fun_type = 0x06                   # 写单个功能码
        self.write_multipie_fun_type = 0x10                 # 写多个功能码
        self.read_fun_type = 0x04                           # 读取功能码
        self.receive_detect_time = 0.1
        self.arb_baud_rate_list = ["500000", "1000000"]         # 仲裁波特率列表
        self.initial_arb_baud_rate = "500000"
        self.data_baud_rate_list = ['500000', '1000000', '2000000', '5000000']      # 数据波特率列表
        self.initial_data_baud_rate = "1000000"
        self.BAUD_RATE_LEVELS = ['500K/500K', '500K/1000K', '500K/2000K', '1000K/5000K', '1000K/1000K']
        # 设置数据地址
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
        self.ELE_SKIN_FORCE_INFOR_ADDRESS = 0x42                    # 电容皮肤力信息地址
        self.MOTOR_TEMPERATURE_ADDRESS = 0x48                       # 电机温度地址
        self.HAND_TYPE_ADDRESS = 0x4E                               # 左右手区分

    def open_start_device(self):
        """
        打开CANFD设备并启动CANFD通道

        返回值:
            成功：True
            失败：False
        """
        open_ret = self.open_canfd_device()
        if open_ret:
            return self.start_canfd()
        return False

    def open_canfd_device(self, device_index=0, reserved=0):
        """
        打开CANFD设备

        参数:
            device_index (int): 设备索引号，默认为0
            reserved (int): 保留参数，默认为0

        返回值:
            成功：True
            失败：False
        """
        self.device_index = device_index
        self.reserved = reserved
        # 根据CANFD通道数量选择对应的设备类型
        if self.canfd_channel == 1:
            ZCAN_USBCANFD = ZCAN_USBCANFD_100U
        else:
            ZCAN_USBCANFD = ZCAN_USBCANFD_200U
        self.handle = self.zlg_can.OpenDevice(ZCAN_USBCANFD, self.device_index, self.reserved)
        if self.handle == INVALID_DEVICE_HANDLE:
            print("Open CANFD Device failed!")
            return False
        print("device handle:%d." % self.handle)
        info = self.zlg_can.GetDeviceInf(self.handle)
        print("Device Information:\n%s" % info)
        return True

    def start_canfd(self, chn=0):
        """
        启动CANFD通道

        参数:
            chn (int): 通道索引，默认为0

        返回值:
            成功：True
            失败：False
        """
        # 设置通道索引
        self.chn_index = chn

        # 启动CAN通道并获取通道句柄
        self.chn_handle = can_start(self.zlg_can, self.handle, self.chn_index, self.arb_baud_rate, self.data_baud_rate)

        # 检查通道句柄是否有效
        if self.chn_handle != self.chn_handle:

            print("Start CANFD Channel failed!")
            return False
        print("channel handle:%d." % self.chn_handle)
        # self.receive_thread.start()
        return True

    def close_device(self):
        """
        关闭CANFD设备
        """
        self.can_is_open = False
        close_ret = self.zlg_can.CloseDevice(self.handle)
        if close_ret == 1:
            print("Close Device success! ")
        return close_ret

    def clear_buffer(self):
        """
        清空CANFD通道接收缓存
        """
        clear_ret = self.zlg_can.ClearBuffer(self.chn_handle)
        if clear_ret == 1:
            # print("Clear Buffer success! ")
            pass
        else:
            print("Clear Buffer failed! ")
        return clear_ret

    def reset_canfd(self):
        """
        重置CAN通道配置
        """
        return self.zlg_can.ResetCAN(self.chn_handle)

    def receive_messages(self, fun_type):
        """
        接收CANFD消息并处理特定功能类型的数据

        参数:
            fun_type: 功能类型标识，用于匹配接收到的消息

        返回值:
            list: 当接收到匹配的消息时，返回消息数据中指定长度的数据列表
            bool: 当超时或未接收到匹配消息时，返回False
        """
        receive_start_time = time.time()
        # 在设定的检测时间内循环接收消息
        while time.time() - receive_start_time < self.receive_detect_time:
            rcv_canfd_num = self.zlg_can.GetReceiveNum(self.chn_handle, ZCAN_TYPE_CANFD)
            if rcv_canfd_num:
                # print("Receive CANFD message number:%d" % rcv_canfd_num)
                rcv_canfd_msgs, rcv_canfd_num = self.zlg_can.ReceiveFD(self.chn_handle, rcv_canfd_num)
                if not rcv_canfd_msgs:
                    continue  # 避免空数据访问越界
                # hex_data_list = [f'{hex_data:02X}' for hex_data in rcv_canfd_msgs[0].frame.data]
                # print(hex(rcv_canfd_msgs[0].frame.can_id), hex_data_list)
                # 检查接收到的消息ID是否与预期的LQS ID匹配
                if rcv_canfd_msgs[0].frame.can_id == self.lqs_id:
                    message_data = rcv_canfd_msgs[0].frame.data
                    # 读取功能码数据接收
                    if fun_type == self.read_fun_type and message_data[1] == fun_type:
                        # 计算需要接收的数据字节数
                        receive_number = message_data[2] + 5
                        int_data = [data for data in message_data[:receive_number]]
                        print("接收报文：", hex(rcv_canfd_msgs[0].frame.can_id), int_data)
                        # 返回从第4个字节开始的指定长度数据
                        return int_data[3:3+message_data[2]]
                    elif fun_type == self.write_single_fun_type and message_data[1] == fun_type:
                        hex_data = [f'{data:02X}' for data in message_data[:8]]
                        print("接收报文：", hex(rcv_canfd_msgs[0].frame.can_id), hex_data)
                        # 返回从第4个字节开始的指定长度数据
                        return hex_data
                    elif fun_type == self.write_multipie_fun_type and message_data[1] == fun_type:
                        hex_data = [f'{data:02X}' for data in message_data[:8]]
                        print("接收报文：", hex(rcv_canfd_msgs[0].frame.can_id), hex_data)
                        # 返回从第4个字节开始的指定长度数据
                        return hex_data
                    else:
                        return False
        # 超时未接收到匹配消息，返回False
        return False

    def int_canfd_cmd(self, canFD_cmd):
        """
        发送一帧CAN FD消息到指定通道。

        参数:
            canFD_cmd (list): 要发送的CAN FD数据命令列表，每个元素为一个字节（0-255）。

        返回值:
            bool: 发送成功返回True，否则返回False。
        """
        canFD_cmd_len = len(canFD_cmd)
        # 根据数据长度对canFD_cmd进行填充或截断处理，以满足特定长度要求
        if 24 > canFD_cmd_len > 8:
            canFD_cmd += [0] * (4 - canFD_cmd_len % 4)
        elif 32 > canFD_cmd_len > 24:
            canFD_cmd += [0] * (8 - canFD_cmd_len % 8)
        elif 64 > canFD_cmd_len > 32:
            canFD_cmd += [0] * (16 - canFD_cmd_len % 16)
        elif canFD_cmd_len > 64:
            canFD_cmd = canFD_cmd[:64]
        else:
            pass

        # 初始化要发送的CAN FD消息结构体数组
        transmit_canfd_num = 1
        canfd_msgs = (ZCAN_TransmitFD_Data * transmit_canfd_num)()

        # 填充CAN FD消息内容
        for i in range(transmit_canfd_num):
            canfd_msgs[i].transmit_type = 0  # 0-正常发送，2-自发自收
            canfd_msgs[i].frame.eff = 0      # 0-标准帧，1-扩展帧
            canfd_msgs[i].frame.rtr = 0      # 0-数据帧，1-远程帧
            canfd_msgs[i].frame.brs = 1      # BRS 加速标志位：0不加速，1加速
            canfd_msgs[i].frame.can_id = self.lqs_id  # 设置CAN ID
            canfd_msgs[i].frame.len = len(canFD_cmd)  # 数据长度
            # 拷贝数据到帧中
            for j in range(canfd_msgs[i].frame.len):
                canfd_msgs[i].frame.data[j] = canFD_cmd[j]

        # 清空接收缓存
        self.clear_buffer()
        # 发送CAN FD消息
        ret = self.zlg_can.TransmitFD(self.chn_handle, canfd_msgs, transmit_canfd_num)

        # 构造日志文本用于打印
        text = hex(self.lqs_id)
        for cmd in canFD_cmd:
            text += ' ' + format(cmd & 0xFF, '02X')

        # 判断是否发送成功并输出结果
        if ret == transmit_canfd_num:
            print(f"Send one frame of canFD message：", text)
            return True
        else:
            print(f"Send one frame of canFD message failed：", text)
            return False

    #  CANFD的CRC校验和计算
    @staticmethod
    def canfd_crc(data):
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
        return data

    #  CAN FD发送单地址写指令
    def send_single_data(self, start_address, data, signed=False):
        """
        通过CAN FD协议向指定地址写入数据

        参数:
            start_address(int): 起始地址，用于指定要写入数据的目标地址
            data(int): 要写入的数据值，将被转换为2字节的二进制格式

        返回值:
            成功时返回接收到的hex数据列表
            失败时返回False
        """
        with self._lock:
            # 初始化命令数据数组，长度为6个字节
            cmd_data = [0] * 6

            # 设置命令头信息：灵巧手ID和写入服务类型
            cmd_data[0] = self.lqs_id
            cmd_data[1] = self.write_single_fun_type

            # 设置起始地址信息
            cmd_data[2] = 0x0
            cmd_data[3] = start_address

            # 将整数数据转换为2字节的二进制数据，以适应Modbus协议要求
            bytes_value = data.to_bytes(2, byteorder='big', signed=signed)
            step_list = list(bytes_value)
            cmd_data[4] = step_list[0]
            cmd_data[5] = step_list[1]

            # 添加CRC校验并发送CAN FD命令
            canfd_cmd = self.canfd_crc(cmd_data)
            send_ret = self.int_canfd_cmd(canfd_cmd)

            # 处理发送结果，如果发送成功则接收返回消息
            if send_ret:
                data = self.receive_messages(self.write_single_fun_type)
                print("Receive data：", data)
                return send_ret
            else:
                return False

    #  CAN FD发送多地址写指令
    def send_multiple_data(self, start_address, data, signed=False):
        """
        通过CAN FD协议发送多地址写指令

        参数:
            start_address(int): 起始地址
            data(int_list): 要写入的数据列表，每个元素为一个整数

        返回值:
            成功时返回接收到的hex数据列表
            失败时返回False
        """
        with self._lock:
            address_len = len(data)
            # 初始化命令数据数组，长度为7加上两倍的地址长度（每个地址对应两个字节的数据）
            cmd_data = [0] * (7 + address_len * 2)

            # 设置命令头部信息
            cmd_data[0] = self.lqs_id              # 灵巧手ID
            cmd_data[1] = self.write_multipie_fun_type  # 写入服务
            cmd_data[2] = 0x0                      # 起始地址高字节
            cmd_data[3] = start_address           # 起始地址低字节
            cmd_data[4] = 0x0                      # 地址数高字节
            cmd_data[5] = address_len             # 地址数低字节
            cmd_data[6] = address_len * 2         # 数据位长度

            # 将数据转换为字节并填充到命令数组中
            for i in range(len(data)):
                # 将整数转换为2字节的二进制数据，以适应Modbus协议
                bytes_value = data[i].to_bytes(2, byteorder='big', signed=signed)
                step_list = list(bytes_value)
                cmd_data[i * 2 + 7] = step_list[0]
                cmd_data[i * 2 + 8] = step_list[1]

            # 添加CRC校验并发送命令
            canfd_cmd = self.canfd_crc(cmd_data)
            send_ret = self.int_canfd_cmd(canfd_cmd)

            # 处理响应数据
            if send_ret:
                data = self.receive_messages(self.write_multipie_fun_type)
                print("Receive data：", data)
                return send_ret
            else:
                return False

    #  CAN FD发送读指令
    def read_multiple_data(self, start_address, address_len, signed=True):
        """
        通过CAN FD总线发送读取指令

        参数:
            start_address(int): 起始地址
            address_len(int): 地址长度

        返回值:
            成功时返回接收到的int数据列表
            失败时返回False
        """
        with self._lock:
            # 构造命令数据包
            cmd_data = [0] * 6
            cmd_data[0] = self.lqs_id
            cmd_data[1] = self.read_fun_type
            cmd_data[2] = 0x0
            cmd_data[3] = start_address
            cmd_data[4] = 0x0
            cmd_data[5] = address_len

            # 添加CRC校验
            canfd_cmd = self.canfd_crc(cmd_data)
            send_ret = self.int_canfd_cmd(canfd_cmd)
            if send_ret:
                # 接收并处理返回数据
                data = self.receive_messages(self.read_fun_type)
                if data:
                    print("Receive：", data)
                    int_data = [int.from_bytes(data[i:i+2], byteorder='big',
                                               signed=signed) for i in range(0, len(data), 2)]
                    return int_data
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
        if not (1 <= new_id <= 255):
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

    def set_baud(self, baud_order):
        """
        设置设备波特率。

        波特率档位：1-'500K/500K', 2-'500K/1000K', 3-'500K/2000K', 4-'1000K/5000K', 5-'1000K/1000K'

        参数:
        - baud_order: 波特率顺序。
        - 类型：int
        - 范围：1-5

        返回:
        - 成功:初始化ZWHAND类对象
        - 失败:False。
        """

        # 参数验证
        if not isinstance(baud_order, int):
            print(f"Failed to set the baud rate: {baud_order} - Input parameter type error!")
            return False
        if baud_order < 1 or baud_order > len(self.BAUD_RATE_LEVELS):
            print(f"Failed to set the baud rate: {baud_order} - Parameter out of valid range!")
            return False
        index = baud_order - 1
        try:
            send_ret = self.send_single_data(self.SET_BAUD_ADDRESS, baud_order)
            if send_ret:
                # 更新波特率设置
                if baud_order <= 3:
                    self.arb_baud_rate = self.arb_baud_rate_list[0]
                else:
                    self.arb_baud_rate = self.arb_baud_rate_list[1]
                if baud_order == 5:
                    self.data_baud_rate = self.data_baud_rate_list[1]
                else:
                    self.data_baud_rate = self.data_baud_rate_list[index]
                self.reset_canfd()
                self.close_device()
                open_ret = self.open_start_device()
                return open_ret
            else:
                print(f"Failed to set the baud rate: {self.BAUD_RATE_LEVELS[index]}")
                return False
        except Exception as e:
            print(f"Abnormal baud rate setting: {self.BAUD_RATE_LEVELS[index]} - {str(e)}")
            return False

    def set_error_clear(self):
        """
        清除错误。

        返回:
        - 成功:响应报文
        - 失败:False。
        """
        # try:
        send_ret = self.send_single_data(self.CLEAR_ERROR_ADDRESS, 1)
        if send_ret:
            return send_ret
        else:
            print("Failed to clear errors")
            return False
        # except Exception as e:
        #     print(f"Failed to clear errors - {str(e)}")
        #     return False

    def set_power_off_save(self, save_type):
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
                original_arb_baud_rate = self.arb_baud_rate
                original_data_baud_rate = self.data_baud_rate

                try:
                    # 重置本地状态变量到初始值
                    self.lqs_id = self.initial_lqs_id
                    self.arb_baud_rate = self.initial_arb_baud_rate
                    self.data_baud_rate = self.initial_data_baud_rate
                except Exception as e:
                    # 回滚状态
                    self.lqs_id = original_lqs_id
                    self.arb_baud_rate = original_arb_baud_rate
                    self.data_baud_rate = original_data_baud_rate
                    print(f"Error occurred when restoring factory settings: {e}")
                finally:
                    self.close_device()
                    return self.open_start_device()
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
            speed_list = [speed_list] * self.motor_count
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
            current_list = [current_list] * self.motor_count
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
                stop_flag_list = [1] * self.motor_count
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
            print(
                f"Failed to calibrate single motor zero position: {motor_number} - Input parameter out of valid range!")
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




# if __name__ == '__main__':
#     hand = ZWHAND(0x01, '500000', '1000000')
#     open_ret = hand.open_start_device()
#     if open_ret:
#         init_state = hand.get_initialize_state()
#         if init_state:
#             print("Device initialization successful!")
#             ret = hand.set_all_motor_calibration()
#             if ret:
#                 print("Calibration successful!")
#                 time.sleep(20)
#                 hand.set_all_motor_absolute([0, 1000, 1000, 1000, 1000, 1000])
#             else:
#                 print("Calibration failed!")
#         else:
#             print("Device initialization failed!")
#         hand.close_device()

'''
if __name__ == '__main__':
    hand = ZWHAND(0x01, '500000', '1000000')
    open_ret = hand.open_start_device()
    if open_ret:
        print("Device open successful!")
        init_state = hand.get_initialize_state()
        if init_state:
            print("Device initialization successful!")
        else:
            print("Device initialization failed!")
    else:
        print("Device open failed!")
'''
if __name__ == '__main__':
    # 初始化灵巧手实例（请根据实际 CAN 参数调整）
    hand = ZWHAND(lqs_id=0x01, arb_baud_rate='500000', data_baud_rate='1000000')
    
    # 打开设备
    open_ret = hand.open_start_device()
    if not open_ret:
        print("Device open failed, exit.")
        exit(1)
    
    print("Device opened successfully.\n")
    
    # ---------- 测试整手测试功能 ----------
    print("=== Test set_hand_test ===")
    # 打开整手测试
    ret = hand.set_hand_test(True)
    print("Start hand test result:", ret)
    
    # 可在这里等待几秒观察实际电机动作（如有必要）
    # import time; time.sleep(2)
    
    # 关闭整手测试
    ret = hand.set_hand_test(False)
    print("Stop hand test result:", ret)
    
    print("\n=== Test get_hand_type ===")
    # ---------- 测试获取手型 ----------
    hand_type = hand.get_hand_type()
    if hand_type is not False:
        if hand_type == 0:
            print("This is a RIGHT hand.")
        elif hand_type == 1:
            print("This is a LEFT hand.")
        else:
            print(f"Unknown hand type value: {hand_type}")
    else:
        print("Failed to read hand type.")
    
    # 关闭设备
    hand.close_device()
    print("\nDevice closed.")

    