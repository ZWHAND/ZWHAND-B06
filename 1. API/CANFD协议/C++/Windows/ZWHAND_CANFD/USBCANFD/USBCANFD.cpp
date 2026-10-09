// USBCANFD.cpp : 定义控制台应用程序的入口点。


#include "stdafx.h"
#include "zlgcan.h"
#include <iostream>
#include <windows.h>
#include <thread>
#include <vector>
#include <mutex>
#include <ctime>
#include <chrono>


// 打印锁
CRITICAL_SECTION print_mutex;


// 打印加锁宏
#define LOCKED_BLOCK(code) do { \
    EnterCriticalSection(&print_mutex); \
    code; \
    LeaveCriticalSection(&print_mutex); \
} while(0)


// 线程运行标识
int g_thd_run = 1;




// 示例 构造CANFD报文
void Construct_CANFD_Frame(ZCAN_TransmitFD_Data& canfd_data, canid_t id, UINT delay = 0)
{
	memset(&canfd_data, 0, sizeof(canfd_data));
	canfd_data.frame.can_id = id;
	canfd_data.frame.len = sizeof(canfd_data);				// 数据长度
	canfd_data.transmit_type = 0;			// 正常发送
	canfd_data.frame.flags |= CANFD_BRS;	// CANFD加速 0x1
	// canfd_data.frame.flags |= TX_ECHO_FLAG;	// 发送回显 0x20

	if (delay > 0){
		canfd_data.frame.flags |= 0x80;				// 设置延时标志位 ，0x80为1ms精度，0xC0为0.1ms精度
		canfd_data.frame.__res0 = LOBYTE(delay);	// 帧间隔低字节
		canfd_data.frame.__res1 = HIBYTE(delay);	// 帧间隔高字节
	}

	for (int i = 0; i < 64; ++i) {
		canfd_data.frame.data[i] = i;
	}
}



// 普通接收线程
void thread_task(CHANNEL_HANDLE chn)
{
	ZCAN_ReceiveFD_Data canfdData[100] = {};
	int chn_idx = (uintptr_t)chn & 0x000000FF;	// 通道号

	while (g_thd_run)
	{
		if (ZCAN_GetReceiveNum(chn, 1))		// 1-CANFD
		{
			uint32_t ReceiveNum = ZCAN_ReceiveFD(chn, canfdData, 100, 10);
			for (int i = 0; i < ReceiveNum; i++)
			{
				LOCKED_BLOCK({
					printf("CHN:%d [%lld] CANFD", chn_idx, canfdData[i].timestamp);			// 通道和时间戳
					printf((canfdData[i].frame.flags & 0x01) == 0x01 ? "加速 " : "");		// 加速

					printf(IS_EFF(canfdData[i].frame.can_id) ? " 扩展帧 " : " 标准帧 ");		// 帧类型
					//printf(IS_RTR(canfdData[i].frame.can_id) ? " 远程帧 " : " 数据帧 ");	// 帧格式
					printf(IS_TX_ECHO(canfdData[i].frame.flags) ? " Tx " : " Rx ");	// 方向

					printf("ID:0x%X Data: ", GET_ID(canfdData[i].frame.can_id));			// ID
					for (int j = 0; j < canfdData[i].frame.len; j++) {						// 数据
						printf("%02x ", canfdData[i].frame.data[j]);
					}
					printf("\n");
				});
			}
		}
		Sleep(10);
	}
}


// 发送示例
void Send_test(DEVICE_HANDLE dev, CHANNEL_HANDLE chn){
	const int send_num = 1;
	int send_count = 0;


	// CANFD
	ZCAN_TransmitFD_Data canfdData[send_num];
	memset(canfdData, 0, sizeof(canfdData));
	for (int i = 0; i < send_num; ++i) {
		Construct_CANFD_Frame(canfdData[i], i);
	}
	send_count = ZCAN_TransmitFD(chn, canfdData, send_num);

	LOCKED_BLOCK({
		printf("\n发送 %d 条CANFD报文\n", send_count);
	});

}


// 初始化 USBCANFD 通道
CHANNEL_HANDLE Init_chn_USBCANFD(DEVICE_HANDLE dev, int chn_idx)
{
	// 请按照以下顺序配置设备
	CHANNEL_HANDLE chn = nullptr;
	char path[24] = {};

	// 设置通道 仲裁域波特率
	sprintf_s(path, 24, "%d/canfd_abit_baud_rate", chn_idx);
	if (0 == ZCAN_SetValue(dev, path, "500000")) {
		printf("设置仲裁段波特率失败\n");
	}
	// 设置通道 数据域波特率
	sprintf_s(path, "%d/canfd_dbit_baud_rate", chn_idx);
	if (0 == ZCAN_SetValue(dev, path, "1000000")) {
		printf("设置数据段波特率失败\n");
	}

	//// 自定义波特率
	//sprintf_s(path, "%d/baud_rate_custom", chn_idx);
	//if (0 == ZCAN_SetValue(dev, path, "1.0Mbps(85%),5.0Mbps(81%),(80,00C0020F,0000020B)")) {
	//	printf("设置数据段波特率失败\n");
	//}

	// 初始化通道
	ZCAN_CHANNEL_INIT_CONFIG config;	// 通道结构体
	memset(&config, 0, sizeof(config));
	config.can_type = 1;				// 1 = CANFD，CANFD设备必须是初始化CANFD!!
	config.canfd.mode = 0;				// 0-正常模式，1-只听模式

	chn = ZCAN_InitCAN(dev, chn_idx, &config);
	if (chn == INVALID_CHANNEL_HANDLE) {
		printf("初始化通道失败\n");
		return nullptr;
	}

	// 使能终端电阻
	sprintf_s(path, "%d/initenal_resistance", chn_idx);
	if (ZCAN_SetValue(dev, path, "1") == STATUS_ERR) {
		printf("使能终端电阻失败\n");
		return nullptr;
	}


	// 启动通道
	if (ZCAN_StartCAN(chn) == STATUS_ERR) {
		printf("开启通道失败\n");
		return nullptr;
	}

	return chn;
}




class ZWHAND {
private:
    int lqs_id;                                // 设备ID
    int initial_lqs_id;                        // 初始设备ID
    int canfd_channel;                         // CAN通道数
    std::string arb_baud_rate;                 // 仲裁波特率
    std::string data_baud_rate;                // 数据波特率

    DEVICE_HANDLE handle;                      // 设备句柄
    CHANNEL_HANDLE chn_handle;                 // 通道句柄
    int device_index;                          // 设备索引
    int reserved;                              // 保留参数
    int chn_index;                             // 通道索引

    bool can_is_open;                          // 设备是否打开
    std::mutex _lock;                          // 互斥锁
    int motor_count;                           // 电机数量
    int write_single_fun_type;                 // 写单个功能码
    int write_multipie_fun_type;               // 写多个功能码
    int read_fun_type;                         // 读取功能码
    int receive_detect_time;                   // 接收检测时间
    std::vector<std::string> arb_baud_rate_list;  // 仲裁波特率列表
    std::string initial_arb_baud_rate;         // 初始仲裁波特率
    std::vector<std::string> data_baud_rate_list; // 数据波特率列表
    std::string initial_data_baud_rate;        // 初始数据波特率
    std::vector<std::string> BAUD_RATE_LEVELS; // 波特率级别
    // 地址定义
    int SET_ID_ADDRESS;
    int SET_BAUD_ADDRESS;
    int CLEAR_ERROR_ADDRESS;
    int SET_POWER_OFF_SAVE_ADDRESS;
    int FACTORY_DATA_RESET_ADDRESS;
    int SINGLE_MOTOR_CALIBRATION_ADDRESS;
    int CONTROL_JOINT_MOTOR_ABSOLUTE_ADDRESS;
    int CONTROL_JOINT_MOTOR_RELATIVE_ADDRESS;
    int SET_MOTOR_STOP_ADDRESS;
    int SET_SPEED_ADDRESS;
    int SET_CURRENT_ADDRESS;
    int ALL_MOTOR_CALIBRATION_ADDRESS;
    int INITIALIZE_DATA_ADDRESS;
    int BOOTLOADER_VERSION_ADDRESS;
    int HARDWARE_VERSION_ADDRESS;
    int SOFTWARE_VERSION_ADDRESS;
    int HALL_ERROR_ADDRESS;
    int DEVICE_VOLTAGE_ADDRESS;
    int MOVING_RANGE_ADDRESS;
    int MOTOR_LOCK_STATE_ADDRESS;
    int MOTOR_ANGLE_ADDRESS;
    int MOTOR_SPEED_ADDRESS;
    int MOTOR_CURRENT_ADDRESS;
    int PHASE_LOSS_FAULT_ADDRESS;
    int CUR_SAMP_ERROR_ADDRESS;

    int SET_HAND_TEST_ADDRESS;
    int HAND_TYPE_ADDRESS;
    int SET_RADIAN_ADDRESS;
    int SET_FINGER_CTRL_MODE_ADDRESS;
    // int SET_PVT_CTRL_PARAM_ADDRESS;
    int SET_PC_CTRL_PARAM_ADDRESS;
    int MOTOR_RADIAN_ADDRESS;
    int JOINT_ABS_MAX_ADDRESS;
    int ELE_SKIN_FORCE_INFOR_ADDRESS;
    int MOTOR_TEMPERATURE_ADDRESS;
    int CUST_NUMBER_ADDRESS;  // 客户编号地址
    uint16_t SET_PVT_CTRL_PARAM_ADDRESS;

public:
    ZWHAND(int lqs_id = 0x01, std::string arb_baud_rate = "500000",
        std::string data_baud_rate = "1000000", int canfd_channel = 1) {
        this->lqs_id = lqs_id;
        this->initial_lqs_id = 0x01;
        this->canfd_channel = canfd_channel;
        this->arb_baud_rate = arb_baud_rate;
        this->data_baud_rate = data_baud_rate;

        this->handle = INVALID_DEVICE_HANDLE;
        this->chn_handle = INVALID_CHANNEL_HANDLE;
        this->device_index = 0;
        this->reserved = 0;
        this->chn_index = 0;

        this->can_is_open = false;
        this->motor_count = 6;
        this->write_single_fun_type = 0x06;
        this->write_multipie_fun_type = 0x10;
        this->read_fun_type = 0x04;
        this->receive_detect_time = 2;
        this->arb_baud_rate_list = { "500000", "1000000" };
        this->initial_arb_baud_rate = "500000";
        this->data_baud_rate_list = { "500000", "1000000", "2000000", "5000000" };
        this->initial_data_baud_rate = "1000000";
        this->BAUD_RATE_LEVELS = { "500K/500K", "500K/1000K", "500K/2000K", "1000K/5000K", "1000K/1000K" };
        this->SET_ID_ADDRESS = 0x00;
        this->SET_BAUD_ADDRESS = 0x01;
        this->CLEAR_ERROR_ADDRESS = 0x02;
        this->SET_POWER_OFF_SAVE_ADDRESS = 0x03;
        this->FACTORY_DATA_RESET_ADDRESS = 0x04;
        this->SINGLE_MOTOR_CALIBRATION_ADDRESS = 0x05;
        this->CONTROL_JOINT_MOTOR_ABSOLUTE_ADDRESS = 0x0B;
        this->CONTROL_JOINT_MOTOR_RELATIVE_ADDRESS = 0x11;
        this->SET_MOTOR_STOP_ADDRESS = 0x17;
        this->SET_SPEED_ADDRESS = 0x1D;
        this->SET_CURRENT_ADDRESS = 0x23;
        this->ALL_MOTOR_CALIBRATION_ADDRESS = 0x29;
        this->INITIALIZE_DATA_ADDRESS = 0x00;
        this->BOOTLOADER_VERSION_ADDRESS = 0x01;
        this->HARDWARE_VERSION_ADDRESS = 0x02;
        this->SOFTWARE_VERSION_ADDRESS = 0x03;
        this->HALL_ERROR_ADDRESS = 0x04;
        this->DEVICE_VOLTAGE_ADDRESS = 0x0A;
        this->MOTOR_LOCK_STATE_ADDRESS = 0x11;
        this->MOTOR_ANGLE_ADDRESS = 0x17;
        this->MOTOR_SPEED_ADDRESS = 0x1D;
        this->MOTOR_CURRENT_ADDRESS = 0x23;
        this->PHASE_LOSS_FAULT_ADDRESS = 0x29;
        this->CUR_SAMP_ERROR_ADDRESS = 0x2F;
        
        this->SET_HAND_TEST_ADDRESS = 0x2A;
        this->HAND_TYPE_ADDRESS = 0x4E;
        this->SET_RADIAN_ADDRESS = 0x2B;
        this->SET_FINGER_CTRL_MODE_ADDRESS = 0x31;
        this->SET_PVT_CTRL_PARAM_ADDRESS = 0x37;
        this->SET_PC_CTRL_PARAM_ADDRESS = 0x49;
        this->MOTOR_RADIAN_ADDRESS = 0x35;
        this->JOINT_ABS_MAX_ADDRESS = 0x3B;
        this->ELE_SKIN_FORCE_INFOR_ADDRESS = 0x42;
        this->MOTOR_TEMPERATURE_ADDRESS = 0x48;
        this->CUST_NUMBER_ADDRESS = 0x41;
    }

    bool open_to_start() {
        bool open_ret = open_canfd_device();
        if (open_ret) {
            return start_canfd();
        }
        return false;
    }

    bool open_canfd_device(int device_index = 0, int reserved = 0) {
        this->device_index = device_index;
        this->reserved = reserved;

        // 根据CANFD通道数量选择对应的设备类型
        UINT ZCAN_USBCANFD;
        if (this->canfd_channel == 1) {
            ZCAN_USBCANFD = ZCAN_USBCANFD_100U;
        }
        else {
            ZCAN_USBCANFD = ZCAN_USBCANFD_200U;
        }

        this->handle = ZCAN_OpenDevice(ZCAN_USBCANFD, this->device_index, this->reserved);
        if (this->handle == INVALID_DEVICE_HANDLE) {
            printf("Open CANFD Device failed!\n");
            return false;
        }
        printf("device handle: %d.\n", this->handle);

        ZCAN_DEVICE_INFO info;
        if (ZCAN_GetDeviceInf(this->handle, &info)) {
            printf("Device Information:\n");
            // 打印设备信息
            printf("Hardware Version: %d\n", info.hw_Version);
        }
        return true;
    }

    bool start_canfd(int chn = 0) {
        // 设置通道索引
        this->chn_index = chn;

        // 设置仲裁段波特率
        char path[24];
        sprintf_s(path, 24, "%d/canfd_abit_baud_rate", this->chn_index);
        if (0 == ZCAN_SetValue(this->handle, path, this->arb_baud_rate.c_str())) {
            printf("设置仲裁段波特率失败\n");
            return false;
        }

        // 设置数据段波特率
        sprintf_s(path, 24, "%d/canfd_dbit_baud_rate", this->chn_index);
        if (0 == ZCAN_SetValue(this->handle, path, this->data_baud_rate.c_str())) {
            printf("设置数据段波特率失败\n");
            return false;
        }

        // 初始化通道配置
        ZCAN_CHANNEL_INIT_CONFIG config;
        memset(&config, 0, sizeof(config));
        config.can_type = 1;                // 1 = CANFD
        config.canfd.mode = 0;              // 0-正常模式，1-只听模式

        this->chn_handle = ZCAN_InitCAN(this->handle, this->chn_index, &config);
        if (this->chn_handle == INVALID_CHANNEL_HANDLE) {
            printf("初始化通道失败\n");
            return false;
        }

        // 使能终端电阻
        sprintf_s(path, "%d/initenal_resistance", this->chn_index);
        if (ZCAN_SetValue(this->handle, path, "1") == STATUS_ERR) {
            printf("使能终端电阻失败\n");
            return false;
        }

        // 启动通道
        if (ZCAN_StartCAN(this->chn_handle) == STATUS_ERR) {
            printf("启动通道失败\n");
            return false;
        }

        printf("channel handle: %d.\n", this->chn_handle);
        return true;
    }

    bool close_device() {
        this->can_is_open = false;
        bool close_ret = ZCAN_CloseDevice(this->handle) == STATUS_OK;
        if (close_ret) {
            printf("Close Device success! \n");
        }
        return close_ret;
    }

    bool clear_buffer() {
        bool clear_ret = ZCAN_ClearBuffer(this->chn_handle) == STATUS_OK;
        if (clear_ret) {
            printf("Clear Buffer success! \n");
        }
        return clear_ret;
    }

    bool reset_canfd() {
        return ZCAN_ResetCAN(this->chn_handle) == STATUS_OK;
    }

    std::vector<int> receive_messages(int fun_type) {
        auto start_time = std::chrono::high_resolution_clock::now();
        auto timeout = std::chrono::seconds(this->receive_detect_time);

        while (std::chrono::high_resolution_clock::now() - start_time < timeout) {
            uint32_t rcv_canfd_num = ZCAN_GetReceiveNum(this->chn_handle, 1); // 1-CANFD
            if (rcv_canfd_num) {
                ZCAN_ReceiveFD_Data* canfdData = new ZCAN_ReceiveFD_Data[rcv_canfd_num];
                uint32_t received_num = ZCAN_ReceiveFD(this->chn_handle, canfdData, rcv_canfd_num, 100);

                for (int i = 0; i < received_num; i++) {
                    // 检查接收到的消息ID是否与预期的LQS ID匹配
                    // 提取CAN ID的低8位作为设备ID
                    int received_id = canfdData[i].frame.can_id & 0xFF;
                    if (received_id == this->lqs_id) {
                        std::vector<BYTE> message_data(canfdData[i].frame.data,
                            canfdData[i].frame.data + canfdData[i].frame.len);

                        // 读取功能码数据接收
                        if (fun_type == this->read_fun_type && message_data.size() > 1 && message_data[1] == fun_type) {
                            int receive_number = message_data[2] + 5;
                            std::vector<int> int_data;
                            for (int j = 0; j < receive_number; j++) {
                                int_data.push_back(message_data[j]);
                            }
                            printf("接收报文：0x%X ", received_id);
                            for (auto val : int_data) {
                                printf("%02X ", val);
                            }
                            
                            printf("\n");
                            // 返回从第4个字节开始的指定长度数据
                            std::vector<int> result;
                            for (int j = 3; j < 3 + message_data[2] && j < int_data.size(); j++) {
                                result.push_back(int_data[j]);
                            }
                            delete[] canfdData;
                            return result;
                        }
                        else if (fun_type == this->write_single_fun_type && message_data.size() > 1 && message_data[1] == fun_type) {
                            std::vector<int> hex_data;
                            for (int j = 0; j < (int)message_data.size(); j++) {
                                hex_data.push_back(message_data[j]);
                            }
                            printf("接收报文：0x%X ", received_id);
                            for (auto val : hex_data) {
                                printf("%02X ", val);
                            }
                            printf("\n");
                            delete[] canfdData;
                            return hex_data;
                        }
                        else if (fun_type == this->write_multipie_fun_type && message_data.size() > 1 && message_data[1] == fun_type) {
                            std::vector<int> hex_data;
                            for (int j = 0; j < (int)message_data.size(); j++) {
                                hex_data.push_back(message_data[j]);
                            }
                            printf("接收报文：0x%X ", received_id);
                            for (auto val : hex_data) {
                                printf("%02X ", val);
                            }
                            printf("\n");
                            delete[] canfdData;
                            return hex_data;
                        }
                    }
                }
                delete[] canfdData;
            }
            Sleep(10);
        }
        // 超时未接收到匹配消息，返回空向量
        return std::vector<int>();
    }

    // CRC校验和计算
    std::vector<BYTE> canfd_crc(std::vector<BYTE> data) {
        WORD crc = 0xFFFF;
        for (BYTE byte : data) {
            crc ^= byte;
            for (int i = 0; i < 8; i++) {
                if (crc & 0x0001) {
                    crc >>= 1;
                    crc ^= 0xA001;
                }
                else {
                    crc >>= 1;
                }
            }
        }
        BYTE crc_high = (crc >> 8) & 0xFF;
        BYTE crc_low = crc & 0xFF;
        data.push_back(crc_low);
        data.push_back(crc_high);
        return data;
    }

    // 发送CANFD消息
    bool int_canfd_cmd(std::vector<BYTE> canFD_cmd) {
        int canFD_cmd_len = canFD_cmd.size();
        // 根据数据长度对canFD_cmd进行填充或截断处理
        if (24 > canFD_cmd_len && canFD_cmd_len > 8) {
            int pad = 4 - canFD_cmd_len % 4;
            for (int i = 0; i < pad; i++) {
                canFD_cmd.push_back(0);
            }
        }
        else if (32 > canFD_cmd_len && canFD_cmd_len > 24) {
            int pad = 8 - canFD_cmd_len % 8;
            for (int i = 0; i < pad; i++) {
                canFD_cmd.push_back(0);
            }
        }
        else if (64 > canFD_cmd_len && canFD_cmd_len > 32) {
            int pad = 16 - canFD_cmd_len % 16;
            for (int i = 0; i < pad; i++) {
                canFD_cmd.push_back(0);
            }
        }
        else if (canFD_cmd_len > 64) {
            canFD_cmd.resize(64);
        }

        // 初始化要发送的CAN FD消息结构体数组
        int transmit_canfd_num = 1;
        ZCAN_TransmitFD_Data canfd_msgs[1];
        memset(canfd_msgs, 0, sizeof(canfd_msgs));


        // 填充CAN FD消息内容
        for (int i = 0; i < transmit_canfd_num; i++) {
            canfd_msgs[i].transmit_type = 0;  // 0-正常发送，2-自发自收
            canfd_msgs[i].frame.flags |= CANFD_BRS;      // BRS 加速标志位：0不加速，1加速
            canfd_msgs[i].frame.can_id = this->lqs_id;  // 设置CAN ID
            canfd_msgs[i].frame.len = canFD_cmd.size();  // 数据长度
            // 拷贝数据到帧中
            for (int j = 0; j < canfd_msgs[i].frame.len && j < 64; j++) {
                canfd_msgs[i].frame.data[j] = canFD_cmd[j];
            }
        }

        // 清空接收缓存
        clear_buffer();
        // 发送CAN FD消息
        uint32_t ret = ZCAN_TransmitFD(this->chn_handle, canfd_msgs, transmit_canfd_num);

        // 构造日志文本用于打印
        printf("Send one frame of canFD message：0x%X ", this->lqs_id);
        for (BYTE cmd : canFD_cmd) {
            printf("%02X ", cmd);
        }
        printf("\n");

        // 判断是否发送成功并输出结果
        if (ret == transmit_canfd_num) {
            printf("Send successful\n");
            return true;
        }
        else {
            printf("Send failed\n");
            return false;
        }
    }

    // CAN FD发送单地址写指令
    std::vector<int> canfd_write_single_address(int start_address, int data) {
        std::lock_guard<std::mutex> lock(this->_lock);

        // 初始化命令数据数组，长度为6个字节
        std::vector<BYTE> cmd_data(6);

        // 设置命令头信息：灵巧手ID和写入服务类型
        cmd_data[0] = this->lqs_id;
        cmd_data[1] = this->write_single_fun_type;

        // 设置起始地址信息
        cmd_data[2] = 0x0;
        cmd_data[3] = start_address;

        // 将整数数据转换为2字节的二进制数据
        BYTE high_byte = (data >> 8) & 0xFF;
        BYTE low_byte = data & 0xFF;
        cmd_data[4] = high_byte;
        cmd_data[5] = low_byte;

        // 添加CRC校验并发送CAN FD命令
        std::vector<BYTE> canfd_cmd = canfd_crc(cmd_data);
        bool send_ret = int_canfd_cmd(canfd_cmd);

        // 处理发送结果，如果发送成功则接收返回消息
        if (send_ret) {
            std::vector<int> data = receive_messages(this->write_single_fun_type);
            printf("Receive data：");
            for (int val : data) {
                printf("%d ", val);
            }
            printf("\n");
            return data;
        }
        else {
            return std::vector<int>();
        }
    }

    // CAN FD发送多地址写指令
    std::vector<int> canfd_write_multiple_address(int start_address, std::vector<int> data) {
        std::lock_guard<std::mutex> lock(this->_lock);

        int address_len = data.size();
        // 初始化命令数据数组，长度为7加上两倍的地址长度
        std::vector<BYTE> cmd_data(7 + address_len * 2);

        // 设置命令头部信息
        cmd_data[0] = this->lqs_id;              // 灵巧手ID
        cmd_data[1] = this->write_multipie_fun_type;  // 写入服务
        cmd_data[2] = 0x0;                      // 起始地址高字节
        cmd_data[3] = start_address;           // 起始地址低字节
        cmd_data[4] = 0x0;                      // 地址数高字节
        cmd_data[5] = address_len;             // 地址数低字节
        cmd_data[6] = address_len * 2;         // 数据位长度

        // 将数据转换为字节并填充到命令数组中
        for (int i = 0; i < data.size(); i++) {
            // 将整数转换为2字节的二进制数据
            BYTE high_byte = (data[i] >> 8) & 0xFF;
            BYTE low_byte = data[i] & 0xFF;
            cmd_data[i * 2 + 7] = high_byte;
            cmd_data[i * 2 + 8] = low_byte;
        }

        // 添加CRC校验并发送命令
        std::vector<BYTE> canfd_cmd = canfd_crc(cmd_data);
        bool send_ret = int_canfd_cmd(canfd_cmd);

        // 处理响应数据
        if (send_ret) {
            std::vector<int> data = receive_messages(this->write_multipie_fun_type);
            printf("Receive data：");
            for (int val : data) {
                printf("%d ", val);
            }
            printf("\n");
            return data;
        }
        else {
            return std::vector<int>();
        }
    }



    // CAN FD发送多地址写指令 (uint16_t版本 - 专用于PVT)
    std::vector<uint16_t> canfd_write_multiple_address_uint16(int start_address, std::vector<uint16_t> data) {
        std::lock_guard<std::mutex> lock(this->_lock);

        int address_len = data.size();
        // 初始化命令数据数组，长度为7加上两倍的地址长度
        std::vector<BYTE> cmd_data(7 + address_len * 2);

        // 设置命令头部信息
        cmd_data[0] = this->lqs_id;              // 灵巧手ID
        cmd_data[1] = this->write_multipie_fun_type;  // 写入服务
        cmd_data[2] = 0x0;                      // 起始地址高字节
        cmd_data[3] = start_address;           // 起始地址低字节
        cmd_data[4] = 0x0;                      // 地址数高字节
        cmd_data[5] = address_len;             // 地址数低字节
        cmd_data[6] = address_len * 2;         // 数据位长度

        // 将uint16_t数据转换为字节并填充到命令数组中
        for (int i = 0; i < data.size(); i++) {
            // 将uint16_t转换为2字节的二进制数据（大端序）
            BYTE high_byte = (data[i] >> 8) & 0xFF;
            BYTE low_byte = data[i] & 0xFF;
            cmd_data[i * 2 + 7] = high_byte;
            cmd_data[i * 2 + 8] = low_byte;
        }

        // 添加CRC校验并发送命令
        std::vector<BYTE> canfd_cmd = canfd_crc(cmd_data);
        bool send_ret = int_canfd_cmd(canfd_cmd);

        // 处理响应数据
        if (send_ret) {
            std::vector<int> data = receive_messages(this->write_multipie_fun_type);
            printf("Receive data：");
            for (int val : data) {
                printf("%d ", val);
            }
            printf("\n");
            
            // 转换为uint16_t返回
            std::vector<uint16_t> uint16_result;
            for (int val : data) {
                uint16_result.push_back(static_cast<uint16_t>(val));
            }
            return uint16_result;
        }
        else {
            return std::vector<uint16_t>();
        }
    }



    // CAN FD发送读指令
    std::vector<int> canfd_read_cmd(int start_address, int address_len) {
        std::lock_guard<std::mutex> lock(this->_lock);
        std::vector<BYTE> cmd_data(6);

        // 设置命令头部信息
        cmd_data[0] = this->lqs_id;              // 灵巧手ID
        cmd_data[1] = this->read_fun_type;      // 读取服务
        cmd_data[2] = 0x0;                      // 起始地址高字节
        cmd_data[3] = start_address;           // 起始地址低字节
        cmd_data[4] = 0x0;                      // 地址数高字节
        cmd_data[5] = address_len;             // 地址数低字节

        // 添加CRC校验
        std::vector<BYTE> canfd_cmd = canfd_crc(cmd_data);
        bool send_ret = int_canfd_cmd(canfd_cmd);
        if (send_ret) {
            // 接收并处理返回数据
            std::vector<int> data = receive_messages(this->read_fun_type);
            if (!data.empty()) {
                std::vector<int> int_data;
                for (int i = 0; i < data.size(); i += 2) {
                    if (i + 1 < data.size()) {
                        int value = (data[i] << 8) | data[i + 1];
                        // 处理有符号整数
                        if (value > 32767) value -= 65536;
                        int_data.push_back(value);
                    }
                }
                return int_data;
            }
        }
        return std::vector<int>();
    }

    // 设置设备ID
    std::vector<int> set_id(int new_id) {
        // 参数验证
        if (new_id < 1 || new_id > 255) {
            printf("Failed to set ID: <%d>, The parameter is outside the valid range of 1 to 255!\n", new_id);
            return std::vector<int>();
        }
        try {
            std::vector<int> send_ret = canfd_write_single_address(this->SET_ID_ADDRESS, new_id);
            if (!send_ret.empty()) {
                this->lqs_id = new_id;
                return send_ret;
            }
            else {
                printf("Failed to set ID: <%d>\n", new_id);
                return std::vector<int>();
            }
        }
        catch (...) {
            printf("Failed to set ID: <%d>, Data transmission error\n", new_id);
            return std::vector<int>();
        }
    }

    // 设置设备波特率
    bool set_baud(int baud_order) {
        // 参数验证
        if (baud_order < 1 || baud_order > this->BAUD_RATE_LEVELS.size()) {
            printf("Failed to set the baud rate: %d - Parameter out of valid range!\n", baud_order);
            return false;
        }
        int index = baud_order - 1;
        try {
            std::vector<int> send_ret = canfd_write_single_address(this->SET_BAUD_ADDRESS, baud_order);
            if (!send_ret.empty()) {
                // 更新波特率设置
                if (baud_order <= 3) {
                    this->arb_baud_rate = this->arb_baud_rate_list[0];
                }
                else {
                    this->arb_baud_rate = this->arb_baud_rate_list[1];
                }
                if (baud_order == 5) {
                    this->data_baud_rate = this->data_baud_rate_list[1];
                }
                else {
                    this->data_baud_rate = this->data_baud_rate_list[index];
                }
                reset_canfd();
                close_device();
                return open_to_start();
            }
            else {
                printf("Failed to set the baud rate: %s\n", this->BAUD_RATE_LEVELS[index].c_str());
                return false;
            }
        }
        catch (...) {
            printf("Abnormal baud rate setting: %s - Error occurred\n", this->BAUD_RATE_LEVELS[index].c_str());
            return false;
        }
    }

    // 清除错误
    std::vector<int> set_error_clear() {
        try {
            std::vector<int> send_ret = canfd_write_single_address(this->CLEAR_ERROR_ADDRESS, 1);
            if (!send_ret.empty()) {
                return send_ret;
            }
            else {
                printf("Failed to clear errors\n");
                return std::vector<int>();
            }
        }
        catch (...) {
            printf("Failed to clear errors - Error occurred\n");
            return std::vector<int>();
        }
    }

    // 设置掉电保存
    std::vector<int> set_power_off_save(int save_type) {
        // 验证参数范围
        if (save_type < 1 || save_type > 2) {
            printf("Failed to save function settings: %d - Parameter out of valid range!\n", save_type);
            return std::vector<int>();
        }
        try {
            std::vector<int> send_ret = canfd_write_single_address(this->SET_POWER_OFF_SAVE_ADDRESS, save_type);
            if (!send_ret.empty()) {
                return send_ret;
            }
            else {
                printf("Failed to save function settings: %d\n", save_type);
                return std::vector<int>();
            }
        }
        catch (...) {
            printf("Failed to save function settings: %d - Error occurred\n", save_type);
            return std::vector<int>();
        }
    }

    // 恢复出厂设置
    bool set_factory_data_reset() {
        try {
            std::vector<int> send_ret = canfd_write_single_address(this->FACTORY_DATA_RESET_ADDRESS, 1);
            if (!send_ret.empty()) {
                // 保存原始状态用于回滚
                int original_lqs_id = this->lqs_id;
                std::string original_arb_baud_rate = this->arb_baud_rate;
                std::string original_data_baud_rate = this->data_baud_rate;

                try {
                    // 重置本地状态变量到初始值
                    this->lqs_id = this->initial_lqs_id;
                    this->arb_baud_rate = this->initial_arb_baud_rate;
                    this->data_baud_rate = this->initial_data_baud_rate;
                }
                catch (...) {
                    // 回滚状态
                    this->lqs_id = original_lqs_id;
                    this->arb_baud_rate = original_arb_baud_rate;
                    this->data_baud_rate = original_data_baud_rate;
                    printf("Error occurred when restoring factory settings\n");
                }
                close_device();
                return open_to_start();
            }
            else {
                printf("Failed to restore factory settings!\n");
                return false;
            }
        }
        catch (...) {
            printf("Error occurred when sending command\n");
            return false;
        }
    }

    // 设置单个电机速度
    std::vector<int> set_single_motor_speed(int motor_number, int speed) {
        // 参数范围检查
        if (motor_number < 1 || motor_number > this->motor_count) {
            printf("Failed to set speed: %d - Parameter out of valid range!\n", motor_number);
            return std::vector<int>();
        }
        if (speed < 1 || speed > 100) {
            printf("Failed to set speed: %d - Speed out of valid range!\n", speed);
            return std::vector<int>();
        }
        int address = this->SET_SPEED_ADDRESS + motor_number - 1;
        try {
            std::vector<int> send_ret = canfd_write_single_address(address, speed);
            if (!send_ret.empty()) {
                return send_ret;
            }
            else {
                printf("Failed to set speed: %d\n", speed);
                return std::vector<int>();
            }
        }
        catch (...) {
            printf("Abnormal speed setting: Error occurred\n");
            return std::vector<int>();
        }
    }

    // 设置所有电机速度
    std::vector<int> set_all_motor_speed(std::vector<int> speed_list) {
        // 验证参数范围
        if (speed_list.size() != this->motor_count) {
            printf("Failed to set speed: Input parameter size error!\n");
            return std::vector<int>();
        }
        for (int speed : speed_list) {
            if (speed < 0 || speed > 100) {
                printf("Failed to set speed: %d - Speed out of valid range!\n", speed);
                return std::vector<int>();
            }
        }
        try {
            std::vector<int> set_ret = canfd_write_multiple_address(this->SET_SPEED_ADDRESS, speed_list);
            if (!set_ret.empty()) {
                return set_ret;
            }
            else {
                printf("Failed to set speed: Input parameter error!\n");
                return std::vector<int>();
            }
        }
        catch (...) {
            printf("Abnormal speed setting: Error occurred\n");
            return std::vector<int>();
        }
    }

    // 设置单个电机电流
    std::vector<int> set_single_motor_current(int motor_number, int current) {
        // 验证参数范围
        if (motor_number < 1 || motor_number > this->motor_count) {
            printf("Failed to set current: %d - Parameter out of valid range!\n", motor_number);
            return std::vector<int>();
        }
        if (current < 1 || current > 100) {
            printf("Failed to set current: %d - Current out of valid range!\n", current);
            return std::vector<int>();
        }
        int address = this->SET_CURRENT_ADDRESS + motor_number - 1;
        try {
            std::vector<int> send_ret = canfd_write_single_address(address, current);
            if (!send_ret.empty()) {
                return send_ret;
            }
            else {
                printf("Failed to set current: %d\n", current);
                return std::vector<int>();
            }
        }
        catch (...) {
            printf("Failed to set current: Error occurred\n");
            return std::vector<int>();
        }
    }

    // 设置所有电机电流
    std::vector<int> set_all_motor_current(std::vector<int> current_list) {
        // 验证参数范围
        if (current_list.size() != this->motor_count) {
            printf("Failed to set current: Input parameter size error!\n");
            return std::vector<int>();
        }
        for (int current : current_list) {
            if (current < 1 || current > 100) {
                printf("Failed to set current: %d - Current out of valid range!\n", current);
                return std::vector<int>();
            }
        }
        try {
            std::vector<int> set_ret = canfd_write_multiple_address(this->SET_CURRENT_ADDRESS, current_list);
            if (!set_ret.empty()) {
                return set_ret;
            }
            else {
                printf("Failed to set current: Input parameter error!\n");
                return std::vector<int>();
            }
        }
        catch (...) {
            printf("Failed to set current: Error occurred\n");
            return std::vector<int>();
        }
    }

    // 设置单个电机停止
    bool set_single_motor_stop(int motor_number) {
        if (motor_number < 1 || motor_number > this->motor_count) {
            printf("Failed to set motor stop: %d - Parameter out of valid range!\n", motor_number);
            return false;
        }
        int address = this->SET_MOTOR_STOP_ADDRESS + motor_number - 1;
        try {
            std::vector<int> send_ret = canfd_write_single_address(address, 1);
            if (!send_ret.empty()) {
                return true;
            }
            else {
                printf("Failed to set motor stop: %d\n", motor_number);
                return false;
            }
        }
        catch (...) {
            printf("Failed to set motor stop: Error occurred\n");
            return false;
        }
    }

    // 设置所有电机停止
    std::vector<int> set_all_motor_stop(std::vector<int> stop_flag_list) {
        try {
            std::vector<int> set_ret = canfd_write_multiple_address(this->SET_MOTOR_STOP_ADDRESS, stop_flag_list);
            if (!set_ret.empty()) {
                return set_ret;
            }
            else {
                printf("Failed to set all motor stop\n");
                return std::vector<int>();
            }
        }
        catch (...) {
            printf("Failed to set all motor stop: Error occurred\n");
            return std::vector<int>();
        }
    }

    // 设置单个电机绝对位置
    std::vector<int> set_single_motor_absolute(int motor_number, int joint_angle) {
        if (motor_number < 1 || motor_number > this->motor_count) {
            printf("Failed to set motor angle: %d - Input parameter out of valid range\n", motor_number);
            return std::vector<int>();
        }
        if (joint_angle < 0 || joint_angle > 1000) {
            printf("Failed to set motor angle: %d - Input parameter out of valid range\n", joint_angle);
            return std::vector<int>();
        }
        int address = this->CONTROL_JOINT_MOTOR_ABSOLUTE_ADDRESS + motor_number - 1;
        try {
            std::vector<int> send_ret = canfd_write_single_address(address, joint_angle);
            if (!send_ret.empty()) {
                return send_ret;
            }
            else {
                printf("Failed to set motor angle: %d\n", joint_angle);
                return std::vector<int>();
            }
        }
        catch (...) {
            printf("Failed to set motor angle: Error occurred\n");
            return std::vector<int>();
        }
    }

    // 设置所有电机绝对位置
    std::vector<int> set_all_motor_absolute(std::vector<int> joint_angle_list) {
        if (joint_angle_list.size() != this->motor_count) {
            printf("Failed to set motor angle: Input parameter size error!\n");
            return std::vector<int>();
        }
        for (int item : joint_angle_list) {
            if (item < 0 || item > 1000) {
                printf("Failed to set motor angle: %d - Input parameter out of valid range\n", item);
                return std::vector<int>();
            }
        }
        try {
            std::vector<int> set_ret = canfd_write_multiple_address(this->CONTROL_JOINT_MOTOR_ABSOLUTE_ADDRESS, joint_angle_list);
            if (!set_ret.empty()) {
                return set_ret;
            }
            else {
                printf("Failed to set all motor angle: Input parameter error!\n");
                return std::vector<int>();
            }
        }
        catch (...) {
            printf("Failed to set all motor angle: Error occurred\n");
            return std::vector<int>();
        }
    }

    // 设置单个电机相对位置
    std::vector<int> set_single_motor_relative(int motor_number, int joint_angle) {
        if (motor_number < 1 || motor_number > this->motor_count) {
            printf("Failed to set motor angle: %d - Input parameter out of valid range!\n", motor_number);
            return std::vector<int>();
        }
        if (joint_angle < -1000 || joint_angle > 1000) {
            printf("Failed to set motor angle: %d - Input parameter out of valid range!\n", joint_angle);
            return std::vector<int>();
        }
        int address = this->CONTROL_JOINT_MOTOR_RELATIVE_ADDRESS + motor_number - 1;
        try {
            std::vector<int> send_ret = canfd_write_single_address(address, joint_angle);
            if (!send_ret.empty()) {
                return send_ret;
            }
            else {
                printf("Failed to set motor angle: %d\n", joint_angle);
                return std::vector<int>();
            }
        }
        catch (...) {
            printf("Failed to set motor angle: Error occurred\n");
            return std::vector<int>();
        }
    }

    // 设置所有电机相对位置
    std::vector<int> set_all_motor_relative(std::vector<int> joint_angle_list) {
        if (joint_angle_list.size() != this->motor_count) {
            printf("Failed to set motor angle: Input parameter size error!!\n");
            return std::vector<int>();
        }
        for (int item : joint_angle_list) {
            if (item < 0 || item > 1000) {
                printf("Failed to set motor angle: %d - Input parameter out of valid range!\n", item);
                return std::vector<int>();
            }
        }
        try {
            std::vector<int> set_ret = canfd_write_multiple_address(this->CONTROL_JOINT_MOTOR_RELATIVE_ADDRESS, joint_angle_list);
            if (!set_ret.empty()) {
                return set_ret;
            }
            else {
                printf("Failed to set all motor angle: Input parameter error!\n");
                return std::vector<int>();
            }
        }
        catch (...) {
            printf("Failed to set all motor angle: Error occurred\n");
            return std::vector<int>();
        }
    }

    // 单个电机校准
    std::vector<int> set_single_motor_calibration(int motor_number) {
        if (motor_number < 1 || motor_number > this->motor_count) {
            printf("Failed to calibrate single motor zero position: %d - Input parameter out of valid range!\n", motor_number);
            return std::vector<int>();
        }
        int address = this->SINGLE_MOTOR_CALIBRATION_ADDRESS + motor_number - 1;
        try {
            std::vector<int> send_ret = canfd_write_single_address(address, 1);
            if (!send_ret.empty()) {
                return send_ret;
            }
            else {
                printf("Failed to calibrate single motor zero position: %d\n", motor_number);
                return std::vector<int>();
            }
        }
        catch (...) {
            printf("Failed to calibrate single motor zero position: Error occurred\n");
            return std::vector<int>();
        }
    }

    // 所有电机校准
    std::vector<int> set_all_motor_calibration() {
        try {
            std::vector<int> send_ret = canfd_write_single_address(this->ALL_MOTOR_CALIBRATION_ADDRESS, 1);
            if (!send_ret.empty()) {
                return send_ret;
            }
            else {
                printf("Failed to calibrate all motor zero position!\n");
                return std::vector<int>();
            }
        }
        catch (...) {
            printf("Failed to calibrate all motor zero position: Error occurred\n");
            return std::vector<int>();
        }
    }

    // 设置整手测试
    bool set_hand_test(bool enable) {
        try {
            int value = enable ? 1 : 0;
            std::vector<int> send_ret = canfd_write_single_address(this->SET_HAND_TEST_ADDRESS, value);
            if (!send_ret.empty()) {
                // 根据实际协议，可能返回报文，这里简单返回 true
                return true;
            }
            else {
                printf("Failed to %s hand test!\n", enable ? "start" : "stop");
                return false;
            }
        }
        catch (...) {
            printf("Failed to %s hand test: Error occurred\n", enable ? "start" : "stop");
            return false;
        }
    }

    // 1. 设置全部关节电机弧度 (范围 0-3600，系数 0.1)
    std::vector<int> set_all_motor_radian(std::vector<int> radian_list) {
        if (radian_list.size() != this->motor_count) {
            printf("Failed to set radian: Input parameter size error!\n");
            return std::vector<int>();
        }
        for (int rad : radian_list) {
            if (rad < 0 || rad > 3600) {
                printf("Failed to set radian: %d out of range!\n", rad);
                return std::vector<int>();
            }
        }
        try {
            return canfd_write_multiple_address(this->SET_RADIAN_ADDRESS, radian_list);
        } catch (...) {
            printf("Failed to set all motor radian: Error occurred\n");
            return std::vector<int>();
        }
    }

    // 2. 设置手指控制模式 (0:PC, 1:PSC, 2:PVT)
    std::vector<int> set_finger_control_mode(std::vector<int> mode_list) {
        if (mode_list.size() != this->motor_count) {
            printf("Failed to set finger control mode: size error!\n");
            return std::vector<int>();
        }
        for (int mode : mode_list) {
            if (mode < 0 || mode > 2) {
                printf("Failed to set finger control mode: %d out of range!\n", mode);
                return std::vector<int>();
            }
        }
        try {
            return canfd_write_multiple_address(this->SET_FINGER_CTRL_MODE_ADDRESS, mode_list);
        } catch (...) {
            printf("Failed to set finger control mode: Error occurred\n");
            return std::vector<int>();
        }
    }

    // 3. 设置 PVT 控制参数 (18个元素：每电机[P, V, T])
    std::vector<uint16_t> set_pvt_control_param(std::vector<uint16_t> pvt_param_list) {
        if (pvt_param_list.size() != this->motor_count * 3) {
            printf("Failed to set PVT param: size must be %d!\n", this->motor_count * 3);
            return std::vector<uint16_t>();
        }
        
        // 验证每个PVT参数的范围
        for (int i = 0; i < pvt_param_list.size(); i += 3) {
            uint16_t position = pvt_param_list[i];
            uint16_t velocity = pvt_param_list[i + 1];
            uint16_t time = pvt_param_list[i + 2];
            
            // 根据实际协议调整范围
            if (position > 1000) {
                printf("Failed: Motor %d Position(%d) out of range\n", i/3 + 1, position);
                return std::vector<uint16_t>();
            }
            if (velocity > 5000) {
                printf("Failed: Motor %d Velocity(%d) out of range\n", i/3 + 1, velocity);
                return std::vector<uint16_t>();
            }
            if (time > 30000) {
                printf("Failed: Motor %d Time(%d) out of range\n", i/3 + 1, time);
                return std::vector<uint16_t>();
            }
        }
        
        try {
            return canfd_write_multiple_address_uint16(this->SET_PVT_CTRL_PARAM_ADDRESS, pvt_param_list);
        } catch (...) {
            printf("Failed to set PVT param: Error occurred\n");
            return std::vector<uint16_t>();
        }
    }

    // 4. 设置 PC 控制参数 (6个有符号电流参数，范围 -8000~8000)
    std::vector<int> set_pc_control_param(std::vector<int> pc_param_list) {
        if (pc_param_list.size() != this->motor_count) {
            printf("Failed to set PC param: size error!\n");
            return std::vector<int>();
        }
        for (int val : pc_param_list) {
            if (val < -8000 || val > 8000) {
                printf("Failed to set PC param: %d out of range!\n", val);
                return std::vector<int>();
            }
        }
        try {
            // 直接发送，负数会自动转换为补码字节流
            return canfd_write_multiple_address(this->SET_PC_CTRL_PARAM_ADDRESS, pc_param_list);
        } catch (...) {
            printf("Failed to set PC param: Error occurred\n");
            return std::vector<int>();
        }
    }

    // 5. 获取全部电机角度弧度 (系数 0.1，返回实际弧度值)
    std::vector<double> get_all_motor_radian() {
        try {
            std::vector<int> raw = canfd_read_cmd(this->MOTOR_RADIAN_ADDRESS, this->motor_count);
            if (!raw.empty()) {
                std::vector<double> result;
                for (int v : raw) {
                    result.push_back(v * 0.1);
                }
                return result;
            } else {
                printf("Failed to get all motor radian!\n");
                return std::vector<double>();
            }
        } catch (...) {
            printf("Failed to get all motor radian: Error occurred\n");
            return std::vector<double>();
        }
    }

    // 6. 获取全部关节可运动最大角度 (原始寄存器值，系数 0.1 未应用)
    std::vector<int> get_all_motor_max_radian() {
        try {
            std::vector<int> ret = canfd_read_cmd(this->JOINT_ABS_MAX_ADDRESS, this->motor_count);
            if (!ret.empty()) return ret;
            printf("Failed to get all motor max radian!\n");
            return std::vector<int>();
        } catch (...) {
            printf("Failed to get all motor max radian: Error occurred\n");
            return std::vector<int>();
        }
    }

    // 7. 获取电子皮肤力数据 (原始值，系数 0.01)
    std::vector<int> get_ele_skin_force_infor() {
        try {
            std::vector<int> ret = canfd_read_cmd(this->ELE_SKIN_FORCE_INFOR_ADDRESS, this->motor_count);
            if (!ret.empty()) return ret;
            printf("Failed to get ele skin force info!\n");
            return std::vector<int>();
        } catch (...) {
            printf("Failed to get ele skin force info: Error occurred\n");
            return std::vector<int>();
        }
    }

    // 8. 获取全部电机温度 (原始值，系数 0.1)
    std::vector<int> get_all_motor_temperature() {
        try {
            std::vector<int> ret = canfd_read_cmd(this->MOTOR_TEMPERATURE_ADDRESS, this->motor_count);
            if (!ret.empty()) return ret;
            printf("Failed to get all motor temperature!\n");
            return std::vector<int>();
        } catch (...) {
            printf("Failed to get all motor temperature: Error occurred\n");
            return std::vector<int>();
        }
    }

    // 获取初始化状态
    int get_initialize_state() {
        try {
            std::vector<int> send_ret = canfd_read_cmd(this->INITIALIZE_DATA_ADDRESS, 1);
            if (!send_ret.empty()) {
                return send_ret[0];
            }
            else {
                printf("Failed to get initialization state!\n");
                return -1;
            }
        }
        catch (...) {
            printf("Failed to get initialization state: Error occurred\n");
            return -1;
        }
    }

    // 获取bootloader版本
    int get_bootloader_version() {
        try {
            std::vector<int> send_ret = canfd_read_cmd(this->BOOTLOADER_VERSION_ADDRESS, 1);
            if (!send_ret.empty()) {
                return send_ret[0];
            }
            else {
                printf("Failed to get bootloader version!\n");
                return -1;
            }
        }
        catch (...) {
            printf("Failed to get bootloader version: Error occurred\n");
            return -1;
        }
    }

    // 获取硬件版本
    int get_hardware_version() {
        try {
            std::vector<int> send_ret = canfd_read_cmd(this->HARDWARE_VERSION_ADDRESS, 1);
            if (!send_ret.empty()) {
                return send_ret[0];
            }
            else {
                printf("Failed to get hardware version!\n");
                return -1;
            }
        }
        catch (...) {
            printf("Failed to get hardware version: Error occurred\n");
            return -1;
        }
    }

    // 获取软件版本
    int get_software_version() {
        try {
            std::vector<int> send_ret = canfd_read_cmd(this->SOFTWARE_VERSION_ADDRESS, 1);
            if (!send_ret.empty()) {
                return send_ret[0];
            }
            else {
                printf("Failed to get software version!\n");
                return -1;
            }
        }
        catch (...) {
            printf("Failed to get software version: Error occurred\n");
            return -1;
        }
    }

    // 获取设备错误
    std::vector<int> get_device_error() {
        try {
            std::vector<int> send_ret = canfd_read_cmd(this->HALL_ERROR_ADDRESS, 6);
            if (!send_ret.empty()) {
                return send_ret;
            }
            else {
                printf("Failed to get device error code!\n");
                return std::vector<int>();
            }
        }
        catch (...) {
            printf("Failed to get device error code: Error occurred\n");
            return std::vector<int>();
        }
    }

    // 获取设备电压
    int get_device_voltage() {
        try {
            std::vector<int> send_ret = canfd_read_cmd(this->DEVICE_VOLTAGE_ADDRESS, 1);
            if (!send_ret.empty()) {
                return send_ret[0];
            }
            else {
                printf("Failed to get device voltage!\n");
                return -1;
            }
        }
        catch (...) {
            printf("Failed to get device voltage: Error occurred\n");
            return -1;
        }
    }

    // 获取电机堵转状态
    std::vector<int> get_motor_locked_state() {
        try {
            std::vector<int> send_ret = canfd_read_cmd(this->MOTOR_LOCK_STATE_ADDRESS, 6);
            if (!send_ret.empty()) {
                return send_ret;
            }
            else {
                printf("Failed to get motor locked state!\n");
                return std::vector<int>();
            }
        }
        catch (...) {
            printf("Failed to get motor locked state: Error occurred\n");
            return std::vector<int>();
        }
    }

    // 获取电机实际角度
    std::vector<int> get_motor_real_angle() {
        try {
            std::vector<int> send_ret = canfd_read_cmd(this->MOTOR_ANGLE_ADDRESS, 6);
            if (!send_ret.empty()) {
                return send_ret;
            }
            else {
                printf("Failed to get motor real angle!\n");
                return std::vector<int>();
            }
        }
        catch (...) {
            printf("Failed to get motor real angle: Error occurred\n");
            return std::vector<int>();
        }
    }

    // 获取所有电机速度
    std::vector<int> get_all_motor_speed() {
        try {
            std::vector<int> send_ret = canfd_read_cmd(this->MOTOR_SPEED_ADDRESS, 6);
            if (!send_ret.empty()) {
                return send_ret;
            }
            else {
                printf("Failed to get all motor speed!\n");
                return std::vector<int>();
            }
        }
        catch (...) {
            printf("Failed to get all motor speed: Error occurred\n");
            return std::vector<int>();
        }
    }

    // 获取所有电机电流
    std::vector<int> get_all_motor_current() {
        try {
            std::vector<int> send_ret = canfd_read_cmd(this->MOTOR_CURRENT_ADDRESS, 6);
            if (!send_ret.empty()) {
                return send_ret;
            }
            else {
                printf("Failed to get all motor current!\n");
                return std::vector<int>();
            }
        }
        catch (...) {
            printf("Failed to get all motor current: Error occurred\n");
            return std::vector<int>();
        }
    }

    // 获取相位丢失故障
    std::vector<int> get_phase_loss_fault() {
        try {
            std::vector<int> send_ret = canfd_read_cmd(this->PHASE_LOSS_FAULT_ADDRESS, 6);
            if (!send_ret.empty()) {
                return send_ret;
            }
            else {
                printf("Failed to get phase loss fault!\n");
                return std::vector<int>();
            }
        }
        catch (...) {
            printf("Failed to get phase loss fault: Error occurred\n");
            return std::vector<int>();
        }
    }

    // 获取电流采样故障
    std::vector<int> get_cur_samp_fault() {
        try {
            std::vector<int> send_ret = canfd_read_cmd(this->CUR_SAMP_ERROR_ADDRESS, 6);
            if (!send_ret.empty()) {
                return send_ret;
            }
            else {
                printf("Failed to get cur samp fault!\n");
                return std::vector<int>();
            }
        }
        catch (...) {
            printf("Failed to get cur samp fault: Error occurred\n");
            return std::vector<int>();
        }
    }

    // 获取客户编号
    std::string get_cust_number() {
        try {
            std::vector<int> send_ret = canfd_read_cmd(this->CUST_NUMBER_ADDRESS, 1);
            if (!send_ret.empty()) {
                int raw_value = send_ret[0];
                char cust_char = (char)(raw_value & 0xFF);
                std::string cust_map;
                switch (cust_char) {
                case 'A': cust_map = "无固定客户，通用版本"; break;
                case 'B': cust_map = "星海图客户"; break;
                case 'C': cust_map = "国华客户"; break;
                case 'D': cust_map = "自变量客户"; break;
                default:  cust_map = "未知客户"; break;
                }
                printf("客户编码: %c - %s\n", cust_char, cust_map.c_str());
                std::string result(1, cust_char);
                return result;
            }
            else {
                printf("Failed to get cust number!\n");
                return "";
            }
        }
        catch (...) {
            printf("Failed to get cust number: Error occurred\n");
            return "";
        }
    }

    // 获取左右手信息
    int get_hand_type() {
        try {
            std::vector<int> send_ret = canfd_read_cmd(this->HAND_TYPE_ADDRESS, 1);
            if (!send_ret.empty()) {
                // 协议约定：0-右手，1-左手
                return send_ret[0];   // 返回单值
            }
            else {
                printf("Failed to get hand type!\n");
                return -1;            // 失败标识
            }
        }
        catch (...) {
            printf("Failed to get hand type: Error occurred\n");
            return -1;
        }
    }

    std::vector<int> get_motor_abs_max_range() {
        try {
            std::vector<int> ret = canfd_read_cmd(0x0B, this->motor_count);
            if (!ret.empty()) return ret;
            printf("Failed to get motor abs max range!\n");
            return std::vector<int>();
        } catch (...) {
            printf("Failed to get motor abs max range: Error occurred\n");
            return std::vector<int>();
        }
    }    

};


int main() {
    ZWHAND hand(0x01);
    bool open_ret = hand.open_to_start();
    if (!open_ret) {
        printf("Device open failed!\n");
        system("pause");
        return 1;
    }

    // ==================== 1. 基本信息读取 ====================
    printf("\n========== 基本信息 ==========\n");

    int init_state = hand.get_initialize_state();
    printf("初始化状态: %d\n", init_state);
    if (init_state == -1) {
        printf("Device initialization failed!\n");
        hand.close_device();
        system("pause");
        return 1;
    }

    int boot_ver = hand.get_bootloader_version();
    printf("Bootloader版本: %d\n", boot_ver);

    int hw_ver = hand.get_hardware_version();
    printf("硬件版本: %d\n", hw_ver);

    int sw_ver = hand.get_software_version();
    printf("软件版本: %d\n", sw_ver);

    int voltage = hand.get_device_voltage();
    printf("设备电压: %d\n", voltage);

    int hand_type = hand.get_hand_type();
    if (hand_type == 0) printf("手类型: 右手\n");
    else if (hand_type == 1) printf("手类型: 左手\n");
    else printf("手类型: 获取失败\n");

    std::string cust = hand.get_cust_number();
    printf("客户编号: %s\n", cust.c_str());

    // ==================== 2. 错误清除 ====================
    printf("\n========== 清除错误 ==========\n");
    hand.set_error_clear();
    printf("错误已清除\n");

    // ==================== 3. 全电机校准 ====================
    printf("\n========== 全电机校准 ==========\n");
    std::vector<int> cal_ret = hand.set_all_motor_calibration();
    if (!cal_ret.empty()) {
        printf("全电机校准成功！等待3秒...\n");
        Sleep(3000);
    }
    else {
        printf("全电机校准失败！\n");
        hand.close_device();
        system("pause");
        return 1;
    }

    // ==================== 4. 读取初始状态 ====================
    printf("\n========== 校准后状态 ==========\n");

    std::vector<int> angles = hand.get_motor_real_angle();
    printf("当前关节角度: ");
    for (int i = 0; i < angles.size(); ++i) printf("%d ", angles[i]);
    printf("\n");

    std::vector<int> speeds = hand.get_all_motor_speed();
    printf("当前电机速度: ");
    for (int i = 0; i < speeds.size(); ++i) printf("%d ", speeds[i]);
    printf("\n");

    std::vector<int> currents = hand.get_all_motor_current();
    printf("当前电机电流: ");
    for (int i = 0; i < currents.size(); ++i) printf("%d ", currents[i]);
    printf("\n");

    // ==================== 5. 速度设置测试 ====================
    printf("\n========== 速度设置测试 ==========\n");
    printf("设置1号电机速度为50...\n");
    hand.set_single_motor_speed(1, 50);
    Sleep(100);

    std::vector<int> all_speeds = { 30, 30, 30, 30, 30, 30 };
    printf("设置所有电机速度为30...\n");
    hand.set_all_motor_speed(all_speeds);
    Sleep(100);

    // ==================== 6. 电流设置测试 ====================
    printf("\n========== 电流设置测试 ==========\n");
    printf("设置1号电机电流为80...\n");
    hand.set_single_motor_current(1, 80);
    Sleep(100);

    std::vector<int> all_currents = { 50, 50, 50, 50, 50, 50 };
    printf("设置所有电机电流为50...\n");
    hand.set_all_motor_current(all_currents);
    Sleep(100);

    // ==================== 7. 绝对位置控制测试 ====================
    printf("\n========== 绝对位置控制测试 ==========\n");
    printf("设置1号电机到位置500...\n");
    hand.set_single_motor_absolute(1, 500);
    Sleep(2000);

    angles = hand.get_motor_real_angle();
    printf("当前位置: ");
    for (int i = 0; i < angles.size(); ++i) printf("%d ", angles[i]);
    printf("\n");

    std::vector<int> abs_angles = { 200, 300, 500, 500, 500, 500 };
    printf("设置所有电机绝对位置: 200, 300, 500, 500, 500, 500...\n");
    hand.set_all_motor_absolute(abs_angles);
    Sleep(3000);

    angles = hand.get_motor_real_angle();
    printf("当前位置: ");
    for (int i = 0; i < angles.size(); ++i) printf("%d ", angles[i]);
    printf("\n");

    // ==================== 8. 相对位置控制测试 ====================
    printf("\n========== 相对位置控制测试 ==========\n");
    printf("1号电机相对移动+100...\n");
    hand.set_single_motor_relative(1, 100);
    Sleep(2000);

    angles = hand.get_motor_real_angle();
    printf("当前位置: ");
    for (int i = 0; i < angles.size(); ++i) printf("%d ", angles[i]);
    printf("\n");

    std::vector<int> rel_angles = { -50, -50, 0, 0, 0, 0 };
    printf("所有电机相对移动: -50, -50, 0, 0, 0, 0...\n");
    hand.set_all_motor_relative(rel_angles);
    Sleep(2000);

    angles = hand.get_motor_real_angle();
    printf("当前位置: ");
    for (int i = 0; i < angles.size(); ++i) printf("%d ", angles[i]);
    printf("\n");

    // ==================== 9. 故障状态读取 ====================
    printf("\n========== 故障状态读取 ==========\n");

    std::vector<int> hall_fault = hand.get_device_error();
    printf("Hall故障状态: ");
    for (int i = 0; i < hall_fault.size(); ++i) printf("%d ", hall_fault[i]);
    printf("\n");

    std::vector<int> locked = hand.get_motor_locked_state();
    printf("堵转状态: ");
    for (int i = 0; i < locked.size(); ++i) printf("%d ", locked[i]);
    printf("\n");

    std::vector<int> phase_loss = hand.get_phase_loss_fault();
    printf("缺相状态: ");
    for (int i = 0; i < phase_loss.size(); ++i) printf("%d ", phase_loss[i]);
    printf("\n");

    std::vector<int> cur_samp = hand.get_cur_samp_fault();
    printf("电流采样状态: ");
    for (int i = 0; i < cur_samp.size(); ++i) printf("%d ", cur_samp[i]);
    printf("\n");

    // ==================== 10. 弧度读取 ====================
    printf("\n========== 弧度读取 ==========\n");

    std::vector<double> radians = hand.get_all_motor_radian();
    printf("当前弧度: ");
    for (int i = 0; i < radians.size(); ++i) printf("%.1f ", radians[i]);
    printf("\n");

    std::vector<int> max_rad = hand.get_all_motor_max_radian();
    printf("最大弧度原始值: ");
    for (int i = 0; i < max_rad.size(); ++i) printf("%d ", max_rad[i]);
    printf("\n");



        // 电机最大量程位置
    std::vector<int> abs_max_range = hand.get_motor_abs_max_range();
    printf("电机最大量程原始值: ");
    for (int i = 0; i < abs_max_range.size(); ++i) printf("%d ", abs_max_range[i]);
    printf("\n");




    // ==================== 11. 温度和皮肤 ====================
    printf("\n========== 温度和电子皮肤 ==========\n");

    std::vector<int> temps = hand.get_all_motor_temperature();
    printf("电机温度原始值: ");
    for (int i = 0; i < temps.size(); ++i) printf("%d ", temps[i]);
    printf("\n");

    std::vector<int> skin = hand.get_ele_skin_force_infor();
    printf("电子皮肤力原始值: ");
    for (int i = 0; i < skin.size(); ++i) printf("%d ", skin[i]);
    printf("\n");

    // ==================== 12. 控制模式测试 ====================
    printf("\n========== 控制模式测试 ==========\n");

    std::vector<int> modes = { 1, 1, 1, 1, 1, 1 };  // PSC模式
    printf("设置所有手指为PSC模式...\n");
    hand.set_finger_control_mode(modes);
    Sleep(100);

    // ==================== 13. 弧度控制测试 ====================
    printf("\n========== 弧度控制测试 ==========\n");
    std::vector<int> rad_set = { 100, 100, 100, 100, 100, 100 };  // 10.0度
    printf("设置所有电机弧度为100 (10.0度)...\n");
    hand.set_all_motor_radian(rad_set);
    Sleep(3000);

    angles = hand.get_motor_real_angle();
    printf("当前位置: ");
    for (int i = 0; i < angles.size(); ++i) printf("%d ", angles[i]);
    printf("\n");

    // ==================== 14. 整手测试 ====================
    printf("\n========== 整手测试 ==========\n");
    printf("启动整手测试...\n");
    hand.set_hand_test(true);
    Sleep(3000);
    printf("停止整手测试...\n");
    hand.set_hand_test(false);

    // ==================== 15. 急停测试 ====================
    printf("\n========== 急停测试 ==========\n");
    printf("停止1号电机...\n");
    hand.set_single_motor_stop(1);
    Sleep(100);

    std::vector<int> stop_flags = { 0, 0, 0, 0, 0, 0 };
    printf("恢复所有电机(停止标志清零)...\n");
    hand.set_all_motor_stop(stop_flags);
    Sleep(100);

    // ==================== 16. 掉电保存 ====================
    printf("\n========== 掉电保存 ==========\n");
    printf("保存参数配置...\n");
    hand.set_power_off_save(2);  // 参数保存

    printf("测试修改设备ID为0x02...\n");
    hand.set_id(0x02);
    Sleep(100);
    printf("恢复设备ID为0x01...\n");
    hand.set_id(0x01);
    Sleep(100);

    // ==================== 17. PVT/PC参数测试 ====================
    printf("\n========== PVT/PC参数测试 ==========\n");

    std::vector<uint16_t> pvt_params = {
        500, 3000, 0,   // 电机1: P=500, V=3000, T=0 (PV模式)
        500, 0, 2000,   // 电机2: P=500, V=0, T=2000 (PT模式)
        500, 3000, 0,
        500, 0, 2000,
        500, 3000, 0,
        500, 0, 2000
    };
    printf("设置PVT参数...\n");
    hand.set_pvt_control_param(pvt_params);
    Sleep(100);

    std::vector<int> pc_params = { 1000, 1000, 1000, 1000, 1000, 1000 };
    printf("设置PC控制参数...\n");
    hand.set_pc_control_param(pc_params);
    Sleep(100);


    std::vector<int> modes_pvt = { 2, 2, 2, 2, 2, 2 };  // PVT模式
    printf("设置所有手指为PVT模式...\n");
    hand.set_finger_control_mode(modes_pvt);
    Sleep(100);

    // ==================== 完成 ====================
    printf("\n========== 全部测试完成 ==========\n");

    // 回到安全位置
    printf("回到安全位置...\n");
    std::vector<int> safe_angles = { 0, 0, 0, 0, 0, 0 };
    hand.set_all_motor_absolute(safe_angles);
    Sleep(2000);

    hand.close_device();
    printf("设备已关闭\n");
    system("pause");
    return 0;
}


// int main() {
//     ZWHAND hand(0x01);
//     bool open_ret = hand.open_to_start();
//     if (open_ret) {
//         int init_state = hand.get_initialize_state();
//         if (init_state != -1) {
//             printf("Device initialization successful!\n");
//             std::vector<int> ret = hand.set_all_motor_calibration();
//             if (!ret.empty()) {
//                 printf("Calibration successful!\n");

//                 Sleep(20000); // 等待20秒
//                 std::vector<int> angles = { 0, 300, 1000, 1000, 1000, 1000 };
//                 hand.set_all_motor_absolute(angles);
//                 Sleep(1000);
//                 std::vector<int> get_state = hand.get_motor_real_angle();
//                 printf("Joint position: ");
//                 for (int i = 0; i < get_state.size(); ++i) {
//                     printf("%d  ", get_state[i]);
//                 }
//                 printf("\n");
                
//             }
//             else {
//                 printf("Calibration failed!\n");
//             }
//         }
//         else {
//             printf("Device initialization failed!\n");
//         }
//         hand.close_device();
//     }
//     system("pause");
//     return 0;
// }




