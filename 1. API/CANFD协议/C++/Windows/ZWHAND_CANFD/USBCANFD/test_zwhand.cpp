/**
 * test_zwhand.cpp
 * 灵巧手 ZWHAND 类全功能测试程序 (CANFD 通信)
 * 编译时需链接 zlgcan.lib，运行目录下需有 zlgcan.dll
 */

#include <iostream>
#include <windows.h>
#include <vector>
#include <string>
#include <ctime>
#include <chrono>
#include <mutex>
#include <thread>

// ---------- 包含 ZLG CAN 头文件 ----------
#include "zlgcan.h"

// 打印锁 (与原类一致)
CRITICAL_SECTION print_mutex;
#define LOCKED_PRINT(msg) do { EnterCriticalSection(&print_mutex); printf msg; LeaveCriticalSection(&print_mutex); } while(0)

// ---------- 原 ZWHAND 类定义 (从你提供的代码完整复制) ----------
class ZWHAND {
private:
    int lqs_id;
    int initial_lqs_id;
    int canfd_channel;
    std::string arb_baud_rate;
    std::string data_baud_rate;
    DEVICE_HANDLE handle;
    CHANNEL_HANDLE chn_handle;
    int device_index;
    int reserved;
    int chn_index;
    bool can_is_open;
    std::mutex _lock;
    int motor_count;
    int write_single_fun_type;
    int write_multipie_fun_type;
    int read_fun_type;
    int receive_detect_time;
    std::vector<std::string> arb_baud_rate_list;
    std::string initial_arb_baud_rate;
    std::vector<std::string> data_baud_rate_list;
    std::string initial_data_baud_rate;
    std::vector<std::string> BAUD_RATE_LEVELS;

    // 寄存器地址
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

    // 新增地址
    int SET_RADIAN_ADDRESS;
    int SET_FINGER_CTRL_MODE_ADDRESS;
    int SET_PVT_CTRL_PARAM_ADDRESS;
    int SET_PC_CTRL_PARAM_ADDRESS;
    int MOTOR_RADIAN_ADDRESS;
    int JOINT_ABS_MAX_ADDRESS;
    int ELE_SKIN_FORCE_INFOR_ADDRESS;
    int MOTOR_TEMPERATURE_ADDRESS;

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
    }

    bool open_to_start() {
        bool open_ret = open_canfd_device();
        if (open_ret) return start_canfd();
        return false;
    }

    bool open_canfd_device(int device_index = 0, int reserved = 0) {
        this->device_index = device_index;
        this->reserved = reserved;
        UINT ZCAN_USBCANFD;
        if (this->canfd_channel == 1) ZCAN_USBCANFD = ZCAN_USBCANFD_100U;
        else ZCAN_USBCANFD = ZCAN_USBCANFD_200U;
        this->handle = ZCAN_OpenDevice(ZCAN_USBCANFD, this->device_index, this->reserved);
        if (this->handle == INVALID_DEVICE_HANDLE) {
            printf("Open CANFD Device failed!\n");
            return false;
        }
        printf("device handle: %d.\n", this->handle);
        ZCAN_DEVICE_INFO info;
        if (ZCAN_GetDeviceInf(this->handle, &info)) {
            printf("Device Information:\n");
            printf("Hardware Version: %d\n", info.hw_Version);
        }
        return true;
    }

    bool start_canfd(int chn = 0) {
        this->chn_index = chn;
        char path[24];
        sprintf_s(path, 24, "%d/canfd_abit_baud_rate", this->chn_index);
        if (0 == ZCAN_SetValue(this->handle, path, this->arb_baud_rate.c_str())) {
            printf("设置仲裁段波特率失败\n");
            return false;
        }
        sprintf_s(path, 24, "%d/canfd_dbit_baud_rate", this->chn_index);
        if (0 == ZCAN_SetValue(this->handle, path, this->data_baud_rate.c_str())) {
            printf("设置数据段波特率失败\n");
            return false;
        }
        ZCAN_CHANNEL_INIT_CONFIG config;
        memset(&config, 0, sizeof(config));
        config.can_type = 1;
        config.canfd.mode = 0;
        this->chn_handle = ZCAN_InitCAN(this->handle, this->chn_index, &config);
        if (this->chn_handle == INVALID_CHANNEL_HANDLE) {
            printf("初始化通道失败\n");
            return false;
        }
        sprintf_s(path, "%d/initenal_resistance", this->chn_index);
        if (ZCAN_SetValue(this->handle, path, "1") == STATUS_ERR) {
            printf("使能终端电阻失败\n");
            return false;
        }
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
        if (close_ret) printf("Close Device success! \n");
        return close_ret;
    }

    bool clear_buffer() {
        bool clear_ret = ZCAN_ClearBuffer(this->chn_handle) == STATUS_OK;
        // printf("Clear Buffer success! \n");
        return clear_ret;
    }

    bool reset_canfd() {
        return ZCAN_ResetCAN(this->chn_handle) == STATUS_OK;
    }

    std::vector<int> receive_messages(int fun_type) {
        auto start_time = std::chrono::high_resolution_clock::now();
        auto timeout = std::chrono::seconds(this->receive_detect_time);
        while (std::chrono::high_resolution_clock::now() - start_time < timeout) {
            uint32_t rcv_canfd_num = ZCAN_GetReceiveNum(this->chn_handle, 1);
            if (rcv_canfd_num) {
                ZCAN_ReceiveFD_Data* canfdData = new ZCAN_ReceiveFD_Data[rcv_canfd_num];
                uint32_t received_num = ZCAN_ReceiveFD(this->chn_handle, canfdData, rcv_canfd_num, 100);
                for (int i = 0; i < received_num; i++) {
                    int received_id = canfdData[i].frame.can_id & 0xFF;
                    if (received_id == this->lqs_id) {
                        std::vector<BYTE> message_data(canfdData[i].frame.data, canfdData[i].frame.data + canfdData[i].frame.len);
                        if (fun_type == this->read_fun_type && message_data.size() > 1 && message_data[1] == fun_type) {
                            int receive_number = message_data[2] + 5;
                            std::vector<int> int_data;
                            for (int j = 0; j < receive_number; j++) int_data.push_back(message_data[j]);
                            printf("接收报文：0x%X ", received_id);
                            for (auto val : int_data) printf("%02X ", val);
                            printf("\n");
                            std::vector<int> result;
                            for (int j = 3; j < 3 + message_data[2] && j < int_data.size(); j++) result.push_back(int_data[j]);
                            delete[] canfdData;
                            return result;
                        }
                        else if (fun_type == this->write_single_fun_type && message_data.size() > 1 && message_data[1] == fun_type) {
                            std::vector<int> hex_data;
                            for (int j = 0; j < (int)message_data.size(); j++) hex_data.push_back(message_data[j]);
                            printf("接收报文：0x%X ", received_id);
                            for (auto val : hex_data) printf("%02X ", val);
                            printf("\n");
                            delete[] canfdData;
                            return hex_data;
                        }
                        else if (fun_type == this->write_multipie_fun_type && message_data.size() > 1 && message_data[1] == fun_type) {
                            std::vector<int> hex_data;
                            for (int j = 0; j < (int)message_data.size(); j++) hex_data.push_back(message_data[j]);
                            printf("接收报文：0x%X ", received_id);
                            for (auto val : hex_data) printf("%02X ", val);
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
        return std::vector<int>();
    }

    std::vector<BYTE> canfd_crc(std::vector<BYTE> data) {
        WORD crc = 0xFFFF;
        for (BYTE byte : data) {
            crc ^= byte;
            for (int i = 0; i < 8; i++) {
                if (crc & 0x0001) { crc >>= 1; crc ^= 0xA001; }
                else crc >>= 1;
            }
        }
        BYTE crc_high = (crc >> 8) & 0xFF;
        BYTE crc_low = crc & 0xFF;
        data.push_back(crc_low);
        data.push_back(crc_high);
        return data;
    }

    bool int_canfd_cmd(std::vector<BYTE> canFD_cmd) {
        int canFD_cmd_len = canFD_cmd.size();
        if (24 > canFD_cmd_len && canFD_cmd_len > 8) {
            int pad = 4 - canFD_cmd_len % 4;
            for (int i = 0; i < pad; i++) canFD_cmd.push_back(0);
        }
        else if (32 > canFD_cmd_len && canFD_cmd_len > 24) {
            int pad = 8 - canFD_cmd_len % 8;
            for (int i = 0; i < pad; i++) canFD_cmd.push_back(0);
        }
        else if (64 > canFD_cmd_len && canFD_cmd_len > 32) {
            int pad = 16 - canFD_cmd_len % 16;
            for (int i = 0; i < pad; i++) canFD_cmd.push_back(0);
        }
        else if (canFD_cmd_len > 64) canFD_cmd.resize(64);

        int transmit_canfd_num = 1;
        ZCAN_TransmitFD_Data canfd_msgs[1];
        memset(canfd_msgs, 0, sizeof(canfd_msgs));
        for (int i = 0; i < transmit_canfd_num; i++) {
            canfd_msgs[i].transmit_type = 0;
            canfd_msgs[i].frame.flags |= CANFD_BRS;
            canfd_msgs[i].frame.can_id = this->lqs_id;
            canfd_msgs[i].frame.len = canFD_cmd.size();
            for (int j = 0; j < canfd_msgs[i].frame.len && j < 64; j++) canfd_msgs[i].frame.data[j] = canFD_cmd[j];
        }
        clear_buffer();
        uint32_t ret = ZCAN_TransmitFD(this->chn_handle, canfd_msgs, transmit_canfd_num);
        printf("Send one frame of canFD message：0x%X ", this->lqs_id);
        for (BYTE cmd : canFD_cmd) printf("%02X ", cmd);
        printf("\n");
        return ret == transmit_canfd_num;
    }

    std::vector<int> canfd_write_single_address(int start_address, int data) {
        std::lock_guard<std::mutex> lock(this->_lock);
        std::vector<BYTE> cmd_data(6);
        cmd_data[0] = this->lqs_id;
        cmd_data[1] = this->write_single_fun_type;
        cmd_data[2] = 0x0;
        cmd_data[3] = start_address;
        cmd_data[4] = (data >> 8) & 0xFF;
        cmd_data[5] = data & 0xFF;
        std::vector<BYTE> canfd_cmd = canfd_crc(cmd_data);
        bool send_ret = int_canfd_cmd(canfd_cmd);
        if (send_ret) {
            std::vector<int> data = receive_messages(this->write_single_fun_type);
            printf("Receive data：");
            for (int val : data) printf("%d ", val);
            printf("\n");
            return data;
        }
        return std::vector<int>();
    }

    std::vector<int> canfd_write_multiple_address(int start_address, std::vector<int> data) {
        std::lock_guard<std::mutex> lock(this->_lock);
        int address_len = data.size();
        std::vector<BYTE> cmd_data(7 + address_len * 2);
        cmd_data[0] = this->lqs_id;
        cmd_data[1] = this->write_multipie_fun_type;
        cmd_data[2] = 0x0;
        cmd_data[3] = start_address;
        cmd_data[4] = 0x0;
        cmd_data[5] = address_len;
        cmd_data[6] = address_len * 2;
        for (int i = 0; i < data.size(); i++) {
            BYTE high = (data[i] >> 8) & 0xFF;
            BYTE low = data[i] & 0xFF;
            cmd_data[i * 2 + 7] = high;
            cmd_data[i * 2 + 8] = low;
        }
        std::vector<BYTE> canfd_cmd = canfd_crc(cmd_data);
        bool send_ret = int_canfd_cmd(canfd_cmd);
        if (send_ret) {
            std::vector<int> data = receive_messages(this->write_multipie_fun_type);
            printf("Receive data：");
            for (int val : data) printf("%d ", val);
            printf("\n");
            return data;
        }
        return std::vector<int>();
    }

    std::vector<int> canfd_read_cmd(int start_address, int address_len) {
        std::lock_guard<std::mutex> lock(this->_lock);
        std::vector<BYTE> cmd_data(6);
        cmd_data[0] = this->lqs_id;
        cmd_data[1] = this->read_fun_type;
        cmd_data[2] = 0x0;
        cmd_data[3] = start_address;
        cmd_data[4] = 0x0;
        cmd_data[5] = address_len;
        std::vector<BYTE> canfd_cmd = canfd_crc(cmd_data);
        bool send_ret = int_canfd_cmd(canfd_cmd);
        if (send_ret) {
            std::vector<int> data = receive_messages(this->read_fun_type);
            if (!data.empty()) {
                std::vector<int> int_data;
                for (int i = 0; i < data.size(); i += 2) {
                    if (i + 1 < data.size()) {
                        int value = (data[i] << 8) | data[i + 1];
                        if (value > 32767) value -= 65536;
                        int_data.push_back(value);
                    }
                }
                return int_data;
            }
        }
        return std::vector<int>();
    }

    // ---- 以下是所有用户功能方法 (简略标注) ----
    std::vector<int> set_id(int new_id) {
        if (new_id < 1 || new_id > 255) { printf("ID out of range\n"); return {}; }
        auto ret = canfd_write_single_address(SET_ID_ADDRESS, new_id);
        if (!ret.empty()) this->lqs_id = new_id;
        return ret;
    }

    bool set_baud(int baud_order) {
        if (baud_order < 1 || baud_order > BAUD_RATE_LEVELS.size()) return false;
        int index = baud_order - 1;
        auto ret = canfd_write_single_address(SET_BAUD_ADDRESS, baud_order);
        if (!ret.empty()) {
            if (baud_order <= 3) arb_baud_rate = arb_baud_rate_list[0];
            else arb_baud_rate = arb_baud_rate_list[1];
            if (baud_order == 5) data_baud_rate = data_baud_rate_list[1];
            else data_baud_rate = data_baud_rate_list[index];
            reset_canfd();
            close_device();
            return open_to_start();
        }
        return false;
    }

    std::vector<int> set_error_clear() { return canfd_write_single_address(CLEAR_ERROR_ADDRESS, 1); }
    std::vector<int> set_power_off_save(int type) { return canfd_write_single_address(SET_POWER_OFF_SAVE_ADDRESS, type); }
    bool set_factory_data_reset() {
        auto ret = canfd_write_single_address(FACTORY_DATA_RESET_ADDRESS, 1);
        if (!ret.empty()) {
            lqs_id = initial_lqs_id;
            arb_baud_rate = initial_arb_baud_rate;
            data_baud_rate = initial_data_baud_rate;
            close_device();
            return open_to_start();
        }
        return false;
    }
    std::vector<int> set_single_motor_speed(int motor, int speed) {
        if (motor<1||motor>motor_count||speed<1||speed>100) return {};
        return canfd_write_single_address(SET_SPEED_ADDRESS + motor - 1, speed);
    }
    std::vector<int> set_all_motor_speed(std::vector<int> speed_list) { return canfd_write_multiple_address(SET_SPEED_ADDRESS, speed_list); }
    std::vector<int> set_single_motor_current(int motor, int current) {
        if (motor<1||motor>motor_count||current<1||current>100) return {};
        return canfd_write_single_address(SET_CURRENT_ADDRESS + motor - 1, current);
    }
    std::vector<int> set_all_motor_current(std::vector<int> current_list) { return canfd_write_multiple_address(SET_CURRENT_ADDRESS, current_list); }
    bool set_single_motor_stop(int motor) {
        if (motor<1||motor>motor_count) return false;
        auto ret = canfd_write_single_address(SET_MOTOR_STOP_ADDRESS + motor - 1, 1);
        return !ret.empty();
    }
    std::vector<int> set_all_motor_stop(std::vector<int> stop_flags) { return canfd_write_multiple_address(SET_MOTOR_STOP_ADDRESS, stop_flags); }
    std::vector<int> set_single_motor_absolute(int motor, int angle) {
        if (motor<1||motor>motor_count||angle<0||angle>1000) return {};
        return canfd_write_single_address(CONTROL_JOINT_MOTOR_ABSOLUTE_ADDRESS + motor - 1, angle);
    }
    std::vector<int> set_all_motor_absolute(std::vector<int> angles) { return canfd_write_multiple_address(CONTROL_JOINT_MOTOR_ABSOLUTE_ADDRESS, angles); }
    std::vector<int> set_single_motor_relative(int motor, int angle) {
        if (motor<1||motor>motor_count||angle<-1000||angle>1000) return {};
        return canfd_write_single_address(CONTROL_JOINT_MOTOR_RELATIVE_ADDRESS + motor - 1, angle);
    }
    std::vector<int> set_all_motor_relative(std::vector<int> angles) { return canfd_write_multiple_address(CONTROL_JOINT_MOTOR_RELATIVE_ADDRESS, angles); }
    std::vector<int> set_single_motor_calibration(int motor) {
        if (motor<1||motor>motor_count) return {};
        return canfd_write_single_address(SINGLE_MOTOR_CALIBRATION_ADDRESS + motor - 1, 1);
    }
    std::vector<int> set_all_motor_calibration() { return canfd_write_single_address(ALL_MOTOR_CALIBRATION_ADDRESS, 1); }

    int get_initialize_state() { auto r = canfd_read_cmd(INITIALIZE_DATA_ADDRESS, 1); return r.empty() ? -1 : r[0]; }
    int get_bootloader_version() { auto r = canfd_read_cmd(BOOTLOADER_VERSION_ADDRESS, 1); return r.empty() ? -1 : r[0]; }
    int get_hardware_version() { auto r = canfd_read_cmd(HARDWARE_VERSION_ADDRESS, 1); return r.empty() ? -1 : r[0]; }
    int get_software_version() { auto r = canfd_read_cmd(SOFTWARE_VERSION_ADDRESS, 1); return r.empty() ? -1 : r[0]; }
    std::vector<int> get_motor_hall_fault() { return canfd_read_cmd(HALL_ERROR_ADDRESS, 6); }
    int get_device_voltage() { auto r = canfd_read_cmd(DEVICE_VOLTAGE_ADDRESS, 1); return r.empty() ? -1 : r[0]; }
    std::vector<int> get_motor_locked_state() { return canfd_read_cmd(MOTOR_LOCK_STATE_ADDRESS, 6); }
    std::vector<int> get_motor_real_angle() { return canfd_read_cmd(MOTOR_ANGLE_ADDRESS, 6); }
    std::vector<int> get_all_motor_speed() { return canfd_read_cmd(MOTOR_SPEED_ADDRESS, 6); }
    std::vector<int> get_all_motor_current() { return canfd_read_cmd(MOTOR_CURRENT_ADDRESS, 6); }
    std::vector<int> get_phase_loss_fault() { return canfd_read_cmd(PHASE_LOSS_FAULT_ADDRESS, 6); }
    std::vector<int> get_cur_samp_fault() { return canfd_read_cmd(CUR_SAMP_ERROR_ADDRESS, 6); }

    // 新增功能
    bool set_hand_test(bool enable) {
        int val = enable ? 1 : 0;
        auto ret = canfd_write_single_address(SET_HAND_TEST_ADDRESS, val);
        return !ret.empty();
    }
    int get_hand_type() {
        auto ret = canfd_read_cmd(HAND_TYPE_ADDRESS, 1);
        if (!ret.empty()) return ret[0];
        return -1;
    }
    std::vector<int> set_all_motor_radian(std::vector<int> radian_list) {
        if (radian_list.size() != motor_count) return {};
        return canfd_write_multiple_address(SET_RADIAN_ADDRESS, radian_list);
    }
    std::vector<int> set_finger_control_mode(std::vector<int> mode_list) {
        if (mode_list.size() != motor_count) return {};
        return canfd_write_multiple_address(SET_FINGER_CTRL_MODE_ADDRESS, mode_list);
    }
    std::vector<int> set_pvt_control_param(std::vector<int> pvt_list) {
        if (pvt_list.size() != motor_count * 3) return {};
        return canfd_write_multiple_address(SET_PVT_CTRL_PARAM_ADDRESS, pvt_list);
    }
    std::vector<int> set_pc_control_param(std::vector<int> pc_list) {
        if (pc_list.size() != motor_count) return {};
        return canfd_write_multiple_address(SET_PC_CTRL_PARAM_ADDRESS, pc_list);
    }
    std::vector<double> get_all_motor_radian() {
        auto raw = canfd_read_cmd(MOTOR_RADIAN_ADDRESS, motor_count);
        std::vector<double> res;
        for (int v : raw) res.push_back(v * 0.1);
        return res;
    }
    std::vector<int> get_all_motor_max_radian() { return canfd_read_cmd(JOINT_ABS_MAX_ADDRESS, motor_count); }
    std::vector<int> get_ele_skin_force_infor() { return canfd_read_cmd(ELE_SKIN_FORCE_INFOR_ADDRESS, motor_count); }
    std::vector<int> get_all_motor_temperature() { return canfd_read_cmd(MOTOR_TEMPERATURE_ADDRESS, motor_count); }
};

// ---------- 测试主函数 ----------
int main() {
    InitializeCriticalSection(&print_mutex);

    // 创建灵巧手对象 (ID=0x01, 仲裁500k, 数据1M)
    ZWHAND hand(0x01, "500000", "1000000");

    printf("========== 灵巧手全功能测试 ==========\n");

    // 1. 打开设备
    if (!hand.open_to_start()) {
        printf("!!! 打开设备失败，测试终止 !!!\n");
        DeleteCriticalSection(&print_mutex);
        system("pause");
        return -1;
    }

    // 2. 获取初始化状态
    printf("\n[1] 获取初始化状态...\n");
    int init = hand.get_initialize_state();
    if (init >= 0) printf("初始化状态: %d\n", init);
    else printf("获取初始化状态失败\n");

    // 3. 清除错误
    printf("\n[2] 清除错误...\n");
    auto err_ret = hand.set_error_clear();
    if (!err_ret.empty()) printf("清除错误成功\n");
    else printf("清除错误失败\n");

    // 4. 获取版本信息
    printf("\n[3] 版本信息:\n");
    int bl = hand.get_bootloader_version();
    int hw = hand.get_hardware_version();
    int sw = hand.get_software_version();
    printf("Bootloader: %d, Hardware: %d, Software: %d\n", bl, hw, sw);

    // 5. 获取设备电压
    printf("\n[4] 设备电压: ");
    int voltage = hand.get_device_voltage();
    if (voltage >= 0) printf("%d mV\n", voltage);
    else printf("读取失败\n");

    // 6. 获取手型
    printf("\n[5] 手型识别: ");
    int handtype = hand.get_hand_type();
    if (handtype == 0) printf("右手\n");
    else if (handtype == 1) printf("左手\n");
    else printf("获取失败\n");

    // 7. 整手测试 (开启2秒后关闭，请确保安全)
    printf("\n[6] 整手测试...\n");
    if (hand.set_hand_test(true)) {
        printf("整手测试开启成功，等待2秒...\n");
        Sleep(2000);
        hand.set_hand_test(false);
        printf("整手测试关闭\n");
    } else {
        printf("整手测试失败\n");
    }

    // 8. 设置电机速度 (安全范围)
    printf("\n[7] 设置所有电机速度为 30...\n");
    std::vector<int> speeds(6, 30);
    auto speed_ret = hand.set_all_motor_speed(speeds);
    if (!speed_ret.empty()) printf("速度设置成功\n");

    // 9. 设置电流
    printf("\n[8] 设置所有电机电流为 50...\n");
    std::vector<int> currents(6, 50);
    auto cur_ret = hand.set_all_motor_current(currents);
    if (!cur_ret.empty()) printf("电流设置成功\n");

    // 10. 获取实时数据
    printf("\n[9] 获取实时数据:\n");
    auto angles = hand.get_motor_real_angle();
    printf("当前角度: ");
    for (int a : angles) printf("%d ", a);
    printf("\n");

    auto m_speed = hand.get_all_motor_speed();
    printf("当前速度: ");
    for (int s : m_speed) printf("%d ", s);
    printf("\n");

    auto m_current = hand.get_all_motor_current();
    printf("当前电流: ");
    for (int c : m_current) printf("%d ", c);
    printf("\n");

    // 11. 故障状态
    printf("\n[10] 故障状态:\n");
    auto hall = hand.get_motor_hall_fault();
    printf("Hall故障: ");
    for (int v : hall) printf("%d ", v);
    printf("\n");

    auto phase = hand.get_phase_loss_fault();
    printf("缺相故障: ");
    for (int v : phase) printf("%d ", v);
    printf("\n");

    auto samp = hand.get_cur_samp_fault();
    printf("电流采样故障: ");
    for (int v : samp) printf("%d ", v);
    printf("\n");

    auto locked = hand.get_motor_locked_state();
    printf("堵转状态: ");
    for (int v : locked) printf("%d ", v);
    printf("\n");

    // 12. 温度
    printf("\n[11] 电机温度: ");
    auto temps = hand.get_all_motor_temperature();
    for (int t : temps) printf("%d ", t);
    printf("\n");

    // 13. 电子皮肤
    printf("\n[12] 电子皮肤力数据: ");
    auto skin = hand.get_ele_skin_force_infor();
    for (int s : skin) printf("%d ", s);
    printf("\n");

    // 14. 弧度读取
    printf("\n[13] 当前弧度: ");
    auto radian_vals = hand.get_all_motor_radian();
    for (double r : radian_vals) printf("%.1f ", r);
    printf("\n");

    // 15. 最大弧度
    printf("\n[14] 最大弧度: ");
    auto max_rad = hand.get_all_motor_max_radian();
    for (int r : max_rad) printf("%d ", r);
    printf("\n");

    // 16. 设置弧度 (示例，请确认电机可动)
    printf("\n[15] 设置弧度 (示例)...\n");
    std::vector<int> set_rad = {0, 500, 1000, 1500, 2000, 2500};
    auto rad_set_ret = hand.set_all_motor_radian(set_rad);
    if (!rad_set_ret.empty()) printf("弧度设置成功\n");

    // 17. 设置控制模式 (PSC=1)
    printf("\n[16] 设置控制模式为PSC...\n");
    std::vector<int> mod = {1,1,1,1,1,1};
    auto mode_ret = hand.set_finger_control_mode(mod);
    if (!mode_ret.empty()) printf("控制模式设置成功\n");

    // 18. 设置PC参数 (示例)
    printf("\n[17] 设置PC参数...\n");
    std::vector<int> pc_params = {1000, -2000, 3000, -4000, 5000, -6000};
    auto pc_ret = hand.set_pc_control_param(pc_params);
    if (!pc_ret.empty()) printf("PC参数设置成功\n");

    // 19. 设置PVT参数 (全零示例)
    printf("\n[18] 设置PVT参数 (全零示例)...\n");
    std::vector<int> pvt(18, 0);
    auto pvt_ret = hand.set_pvt_control_param(pvt);
    if (!pvt_ret.empty()) printf("PVT参数设置成功\n");

    // 20. 关闭设备
    printf("\n所有测试完成，关闭设备...\n");
    hand.close_device();

    DeleteCriticalSection(&print_mutex);
    system("pause");
    return 0;
}