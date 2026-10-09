#include "ZWHAND.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <algorithm>
#include <cerrno>    // for errno

#if defined(_WIN32)
    #include <windows.h>
    #define INVALID_SERIAL (void*)-1        // Windows 无效句柄
    #define SLEEP_MS(ms) Sleep(ms)
#else
    #include <fcntl.h>
    #include <unistd.h>
    #include <termios.h>
    #include <sys/ioctl.h>
    #include <cstring>
    #define INVALID_SERIAL -1               // Linux 无效文件描述符
    #define SLEEP_MS(ms) usleep((ms) * 1000)
#endif


ZWHAND::ZWHAND(int lqs_id, const std::string& port, int baud) 
    : hSerial(INVALID_SERIAL),
      port(port),
      baudrate(baud),
      lqs_id(lqs_id),
      motor_count(6), // 修改为6DOF
      initial_lqs_id(1),
      initial_baud(115200),
      BAUD_RATE_LEVELS({9600, 115200, 921600, 2000000}) {
}

ZWHAND::~ZWHAND() {
    if (hSerial != INVALID_SERIAL) {
#if defined(_WIN32)
        CloseHandle(hSerial);
#else
        close(hSerial);
#endif
    }
}

bool ZWHAND::open_start_device() {
#if defined(_WIN32)
    std::string fullPort = "\\\\.\\" + port;
    hSerial = CreateFileA(fullPort.c_str(), GENERIC_READ | GENERIC_WRITE,
                          0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (hSerial == INVALID_SERIAL) {
        std::cerr << "[串口异常] 无法打开端口 " << port << std::endl;
        return false;
    }
    DCB dcb = {0};
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(hSerial, &dcb)) { CloseHandle(hSerial); hSerial = INVALID_SERIAL; return false; }
    dcb.BaudRate = baudrate;
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity = NOPARITY;
    if (!SetCommState(hSerial, &dcb)) { CloseHandle(hSerial); hSerial = INVALID_SERIAL; return false; }
    COMMTIMEOUTS to = {0};
    to.ReadIntervalTimeout = 50;
    to.ReadTotalTimeoutConstant = 200;
    to.ReadTotalTimeoutMultiplier = 10;
    to.WriteTotalTimeoutConstant = 50;
    to.WriteTotalTimeoutMultiplier = 10;
    if (!SetCommTimeouts(hSerial, &to)) { CloseHandle(hSerial); hSerial = INVALID_SERIAL; return false; }
#else
    hSerial = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (hSerial < 0) {
        std::cerr << "[串口异常] 无法打开端口 " << port << std::endl;
        return false;
    }
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(hSerial, &tty)) { close(hSerial); hSerial = INVALID_SERIAL; return false; }
    speed_t speed;
    switch (baudrate) {
        case 9600:   speed = B9600; break;
        case 115200: speed = B115200; break;
        case 921600: speed = B921600; break;
        case 2000000: speed = B2000000; break;
        default:     speed = B115200; break;
    }
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;               // 无硬件流控
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 2;                  // 200ms 超时（单位0.1s）
    if (tcsetattr(hSerial, TCSANOW, &tty)) { close(hSerial); hSerial = INVALID_SERIAL; return false; }
#endif
    std::cout << "端口<" << port << ">初始化成功" << std::endl;
    return true;
}

bool ZWHAND::close_device() {
    try {
        if (hSerial != INVALID_SERIAL) {
#if defined(_WIN32)
            if (!CloseHandle(hSerial)) {
                std::cerr << "[串口异常] Windows 关闭句柄失败, GetLastError = " 
                          << GetLastError() << std::endl;
            }
#else
            if (close(hSerial) != 0) {
                std::cerr << "[串口异常] Linux 关闭串口失败, errno = " 
                          << errno << " (" << strerror(errno) << ")" << std::endl;
            }
#endif
            hSerial = INVALID_SERIAL;
        }
        return true;
    } catch (...) {
        std::cerr << "[串口异常] 串口关闭时发生未知异常" << std::endl;
        return false;
    }
}


std::vector<unsigned char> ZWHAND::modbus_data_receive(const std::vector<unsigned char>& detect_data, int receive_data_len) {
    auto start_time = std::chrono::steady_clock::now();
    size_t available_num = 0;
    std::vector<unsigned char> data(receive_data_len);
    
    while (true) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count();
        
        if (elapsed > 200) {
            std::cout << "超时 数据接收失败" << std::endl;
            return {};
        }
        
        if (hSerial != INVALID_SERIAL) {
        #if defined(_WIN32)
            COMSTAT comStat;
            DWORD dwErrors;
            if (ClearCommError(hSerial, &dwErrors, &comStat)) {
                available_num = comStat.cbInQue;
            }
        #else
            int bytes_avail = 0;
            ioctl(hSerial, FIONREAD, &bytes_avail);
            available_num = bytes_avail;
        #endif
        } else {
            std::cout << "串口未打开" << std::endl;
            return {};
        }
        
        if (available_num >= receive_data_len) {
            #if defined(_WIN32)
                DWORD bytesRead;
                bool readOk = ReadFile(hSerial, data.data(), receive_data_len, &bytesRead, NULL);
            #else
                ssize_t bytesRead = read(hSerial, data.data(), receive_data_len);
                bool readOk = (bytesRead >= 0);
            #endif
            if (readOk) {
                
                // // ========== 新增打印 ==========
                // std::cout << "[接收原始] ";
                // for (DWORD i = 0; i < bytesRead; ++i) {
                //     printf("%02X ", data[i]);
                // }
                // std::cout << std::endl;
                // // ========== 新增结束 ==========

                // 校验数据
                bool valid = true;
                for (size_t i = 0; i < std::min(detect_data.size(), static_cast<size_t>(bytesRead)); ++i) {
                    if (data[i] != detect_data[i]) {
                        std::cout << "异常数据校验失败" << std::endl;
                        valid = false;
                        break;
                    }
                }
                
                if (!valid) return {};
                
                // 提取数据段
                std::vector<unsigned char> need_data(data.begin() + detect_data.size(), 
                                                    data.end() - 2); // 去掉CRC
                
                if (!need_data.empty()) {
                    
                    std::vector<short> decimal_list;
                    for (size_t i = 0; i < need_data.size(); i += 2) {
                        if (i + 1 >= need_data.size()) break;
                        short decimal_value = (static_cast<short>(need_data[i]) << 8) + 
                                             static_cast<short>(need_data[i + 1]);
                        decimal_list.push_back(decimal_value);
                    }
                    
                    // 转换为unsigned char vector返回
                    std::vector<unsigned char> result(decimal_list.size() * 2);
                    for (size_t i = 0; i < decimal_list.size(); ++i) {
                        result[i*2] = (decimal_list[i] >> 8) & 0xFF;
                        result[i*2 + 1] = decimal_list[i] & 0xFF;
                    }
                    std::cout << "成功接收数据" << result.size() << std::endl;
                    return result;
                } else {
                    return {1}; // 表示成功接收数据
                    std::cout << "成功接收数据" << std::endl;
                }
            }
        }
        
        SLEEP_MS(10);
    }
    
    return {};
}

std::vector<unsigned char> ZWHAND::modbus_crc(const std::vector<unsigned char>& data) {
    unsigned short crc = 0xFFFF;
    for (unsigned char byte : data) {
        crc ^= byte;
        for (int i = 0; i < 8; ++i) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    
    unsigned char crc_high = (crc >> 8) & 0xFF;
    unsigned char crc_low = crc & 0xFF;
    
    std::vector<unsigned char> result = data;
    result.push_back(crc_low);
    result.push_back(crc_high);
    
    return result;
}


bool ZWHAND::send_single_data(unsigned char address, short data) {
    if (hSerial == INVALID_SERIAL) return false;
    
#if defined(_WIN32)
    PurgeComm(hSerial, PURGE_RXCLEAR);
#else
    tcflush(hSerial, TCIFLUSH);
#endif
    
    std::vector<unsigned char> step_list = {
        static_cast<unsigned char>((data >> 8) & 0xFF),
        static_cast<unsigned char>(data & 0xFF)
    };
    
    std::vector<unsigned char> single_motor_cmd = {
        static_cast<unsigned char>(lqs_id), 0x06, 0x00, address, step_list[0], step_list[1]
    };
    
    std::vector<unsigned char> cmd = modbus_crc(single_motor_cmd);
    
    try {
#if defined(_WIN32)
        DWORD bytesWritten;
        bool writeOk = (WriteFile(hSerial, cmd.data(), cmd.size(), &bytesWritten, NULL) && 
                        bytesWritten == cmd.size());
#else
        ssize_t bytesWritten = write(hSerial, cmd.data(), cmd.size());
        bool writeOk = (bytesWritten == static_cast<ssize_t>(cmd.size()));
#endif
        if (writeOk) {
            std::vector<unsigned char> detect_data(cmd.begin(), cmd.begin() + 6);
            std::vector<unsigned char> receive_data = modbus_data_receive(detect_data, 8);
            return !receive_data.empty();
        } else {
            std::cout << "发送未完成" << std::endl;
        }
    } catch (...) {
        std::cout << "发送失败: 异常" << std::endl;
    }
    
    return false;
}



bool ZWHAND::send_multiple_data(unsigned char start_address, const std::vector<short>& data) {
    if (hSerial == INVALID_SERIAL) return false;

#if defined(_WIN32)
    PurgeComm(hSerial, PURGE_RXCLEAR);
#else
    tcflush(hSerial, TCIFLUSH);
#endif

    int address_len = data.size();
    std::vector<unsigned char> cmd_data(7 + address_len * 2);

    cmd_data[0] = lqs_id;
    cmd_data[1] = 0x10;
    cmd_data[2] = 0x00;
    cmd_data[3] = start_address;
    cmd_data[4] = 0x00;
    cmd_data[5] = address_len;
    cmd_data[6] = address_len * 2;

    for (size_t i = 0; i < data.size(); ++i) {
        cmd_data[i * 2 + 7] = (data[i] >> 8) & 0xFF;
        cmd_data[i * 2 + 8] = data[i] & 0xFF;
    }

    std::vector<unsigned char> cmd = modbus_crc(cmd_data);

    try {
#if defined(_WIN32)
        DWORD bytesWritten;
        bool writeOk = (WriteFile(hSerial, cmd.data(), cmd.size(), &bytesWritten, NULL) &&
                        bytesWritten == cmd.size());
#else
        ssize_t bytesWritten = write(hSerial, cmd.data(), cmd.size());
        bool writeOk = (bytesWritten == static_cast<ssize_t>(cmd.size()));
#endif
        if (writeOk) {
            std::vector<unsigned char> detect_data(cmd.begin(), cmd.begin() + 6);
            std::vector<unsigned char> receive_data = modbus_data_receive(detect_data, 8);
            return !receive_data.empty();
        } else {
            std::cout << "发送未完成" << std::endl;
        }
    } catch (...) {
        std::cout << "发送失败: 异常" << std::endl;
    }

    return false;
}



std::vector<short> ZWHAND::read_multiple_data(unsigned char start_address, unsigned char address_len) {
    if (hSerial == INVALID_SERIAL) return {};

#if defined(_WIN32)
    PurgeComm(hSerial, PURGE_RXCLEAR);
#else
    tcflush(hSerial, TCIFLUSH);
#endif

    std::vector<unsigned char> cmd_data = {
        static_cast<unsigned char>(lqs_id), 0x04, 0x00, start_address, 0x00, address_len
    };

    std::vector<unsigned char> cmd = modbus_crc(cmd_data);

    try {
#if defined(_WIN32)
        DWORD bytesWritten;
        bool writeOk = (WriteFile(hSerial, cmd.data(), cmd.size(), &bytesWritten, NULL) && bytesWritten == cmd.size());
#else
        ssize_t bytesWritten = write(hSerial, cmd.data(), cmd.size());
        bool writeOk = (bytesWritten == static_cast<ssize_t>(cmd.size()));
#endif
        if (writeOk) {
            std::vector<unsigned char> detect_data = {
                static_cast<unsigned char>(lqs_id), 0x04, static_cast<unsigned char>(address_len * 2)
            };

            std::vector<unsigned char> received_data = modbus_data_receive(detect_data, address_len * 2 + 5);

            if (!received_data.empty()) {
                std::vector<short> result;
                for (size_t i = 0; i < received_data.size(); i += 2) {
                    if (i + 1 < received_data.size()) {
                        short value = (static_cast<short>(received_data[i]) << 8) + 
                                     static_cast<short>(received_data[i + 1]);
                        result.push_back(value);
                    }
                }
                return result;
            }
        } else {
            std::cout << "发送未完成" << std::endl;
            std::cout << "bytesWritten: " << bytesWritten << " cmd.size(): " << cmd.size() << std::endl;
        }
    } catch (...) {
        std::cout << "发送失败: 异常" << std::endl;
    }

    return {};
}

// 设备配置函数实现
bool ZWHAND::set_id(int lqs_id) {
    if (lqs_id < 1 || lqs_id > 255) {
        std::cout << "设置ID失败: " << lqs_id << "参数超出有效范围1-255" << std::endl;
        return false;
    }
    
    try {
        bool send_ret = send_single_data(SET_ID_ADDRESS_6DOF, static_cast<short>(lqs_id));
        if (send_ret) {
            this->lqs_id = lqs_id;
            std::cout << "设置ID成功: " << lqs_id << std::endl;
            return true;
        } else {
            std::cout << "设置ID失败: " << lqs_id << std::endl;
            return false;
        }
    } catch (...) {
        std::cout << "设置ID失败: " << lqs_id << "数据传输异常" << std::endl;
        return false;
    }
}

bool ZWHAND::set_baud(int baud_order, bool is_restart) {
    if (baud_order < 1 || baud_order > static_cast<int>(BAUD_RATE_LEVELS.size())) {
        std::cout << "设置波特率失败: " << baud_order << " - 参数超出有效范围" << std::endl;
        return false;
    }
    
    try {
        bool send_ret = send_single_data(SET_BAUD_ADDRESS_6DOF, static_cast<short>(baud_order));
        if (send_ret) {
            baudrate = BAUD_RATE_LEVELS[baud_order - 1];
            
            if (is_restart) {
                close_device();
                SLEEP_MS(100);
                open_start_device();
            }
            
            std::cout << "设置波特率成功: " << BAUD_RATE_LEVELS[baud_order - 1] << std::endl;
            return true;
        } else {
            std::cout << "设置波特率失败: " << BAUD_RATE_LEVELS[baud_order - 1] << std::endl;
            return false;
        }
    } catch (...) {
        std::cout << "设置波特率异常: " << BAUD_RATE_LEVELS[baud_order - 1] << std::endl;
        return false;
    }
}

bool ZWHAND::set_error_clear() {
    return send_single_data(CLEAR_ERROR_ADDRESS_6DOF, 1);
}

bool ZWHAND::set_power_off_save(int save_type) {
    if (save_type < 1 || save_type > 2) {
        std::cout << "设置掉电保存失败: " << save_type << " - 参数超出有效范围" << std::endl;
        return false;
    }
    
    try {
        return send_single_data(SET_POWER_OFF_SAVE_ADDRESS_6DOF, static_cast<short>(save_type));
    } catch (...) {
        std::cout << "设置掉电保存异常" << std::endl;
        return false;
    }
}

bool ZWHAND::set_factory_data_reset() {
    try {
        bool send_ret = send_single_data(FACTORY_DATA_RESET_ADDRESS_6DOF, 1);
        if (send_ret) {
            int original_lqs_id = lqs_id;
            int original_baudrate = baudrate;
            
            try {
                lqs_id = initial_lqs_id;
                
                if (baudrate != initial_baud) {
                    close_device();
                    baudrate = initial_baud;
                    open_start_device();
                }
                
                std::cout << "恢复出厂设置成功" << std::endl;
                return true;
            } catch (...) {
                lqs_id = original_lqs_id;
                baudrate = original_baudrate;
                close_device();
                open_start_device();
                std::cout << "恢复出厂设置时发生错误" << std::endl;
                return false;
            }
        } else {
            std::cout << "恢复出厂设置失败" << std::endl;
            return false;
        }
    } catch (...) {
        std::cout << "发送恢复出厂设置指令时发生错误" << std::endl;
        std::cout << "恢复出厂设置失败" << std::endl;
        return false;
    }
}

// 电机控制函数实现
bool ZWHAND::set_single_motor_speed(int motor_number, int speed) {
    if (motor_number < 1 || motor_number > motor_count) {
        std::cout << "设置电机速度失败: " << motor_number << " - 参数超出有效范围" << std::endl;
        return false;
    }
    
    if (speed < 1 || speed > 100) {
        std::cout << "设置电机速度失败: " << speed << " - 速度超出有效范围" << std::endl;
        return false;
    }
    
    unsigned char address = SET_SPEED_ADDRESS_6DOF + motor_number - 1;
    try {
        return send_single_data(address, static_cast<short>(speed));
    } catch (...) {
        std::cout << "设置电机速度异常" << std::endl;
        return false;
    }
}

bool ZWHAND::set_all_motor_speed(int speed) {
    if (speed < 1 || speed > 100) {
        std::cout << "设置所有电机速度失败: " << speed << " - 速度超出有效范围" << std::endl;
        return false;
    }
    
    std::vector<short> speeds(motor_count, static_cast<short>(speed));
    return send_multiple_data(SET_SPEED_ADDRESS_6DOF, speeds);
}

bool ZWHAND::set_single_motor_current(int motor_number, int current) {
    if (motor_number < 1 || motor_number > motor_count) {
        std::cout << "设置电机电流失败: " << motor_number << " - 参数超出有效范围" << std::endl;
        return false;
    }
    
    if (current < 1 || current > 100) {
        std::cout << "设置电机电流失败: " << current << " - 参数超出有效范围" << std::endl;
        return false;
    }
    
    unsigned char address = SET_CURRENT_ADDRESS_6DOF + motor_number - 1;
    try {
        return send_single_data(address, static_cast<short>(current));
    } catch (...) {
        std::cout << "设置电机电流失败" << std::endl;
        return false;
    }
}

bool ZWHAND::set_all_motor_current(int current) {
    if (current < 1 || current > 100) {
        std::cout << "设置所有电机电流失败: " << current << " - 参数超出有效范围" << std::endl;
        return false;
    }
    
    std::vector<short> currents(motor_count, static_cast<short>(current));
    return send_multiple_data(SET_CURRENT_ADDRESS_6DOF, currents);
}

bool ZWHAND::set_single_motor_stop(int motor_number) {
    if (motor_number < 1 || motor_number > motor_count) {
        std::cout << "设置电机停止失败: " << motor_number << " - 参数超出有效范围" << std::endl;
        return false;
    }
    
    unsigned char address = SET_MOTOR_STOP_ADDRESS_6DOF + motor_number - 1;
    try {
        return send_single_data(address, 1);
    } catch (...) {
        std::cout << "设置电机停止失败" << std::endl;
        return false;
    }
}

bool ZWHAND::set_all_motor_stop() {
    std::vector<short> stops(motor_count, 1);
    return send_multiple_data(SET_MOTOR_STOP_ADDRESS_6DOF, stops);
}

bool ZWHAND::set_single_motor_absolute(int motor_number, int joint_angle) {
    if (motor_number < 1 || motor_number > motor_count) {
        std::cout << "设置电机角度失败: " << motor_number << " - 参数超出有效范围" << std::endl;
        return false;
    }
    
    if (joint_angle < 0 || joint_angle > 1000) {
        std::cout << "设置电机角度失败: " << joint_angle << " - 参数超出有效范围" << std::endl;
        return false;
    }
    
    unsigned char address = CONTROL_JOINT_MOTOR_ABSOLUTE_ADDRESS_6DOF + motor_number - 1;
    try {
        return send_single_data(address, static_cast<short>(joint_angle));
    } catch (...) {
        std::cout << "设置电机角度失败" << std::endl;
        return false;
    }
}

bool ZWHAND::set_all_motor_absolute(const std::vector<int>& joint_angle_list) {
    if (static_cast<int>(joint_angle_list.size()) != motor_count) {
        std::cout << "设置所有电机角度失败: 数组长度不匹配!" << std::endl;
        return false;
    }
    
    for (int item : joint_angle_list) {
        if (item < 0 || item > 1000) {
            std::cout << "设置所有电机角度失败: " << item << " - 参数超出有效范围" << std::endl;
            return false;
        }
    }
    
    std::vector<short> angles;
    for (int item : joint_angle_list) {
        angles.push_back(static_cast<short>(item));
    }
    
    return send_multiple_data(CONTROL_JOINT_MOTOR_ABSOLUTE_ADDRESS_6DOF, angles);
}

bool ZWHAND::set_single_motor_relative(int motor_number, int joint_angle) {
    if (motor_number < 1 || motor_number > motor_count) {
        std::cout << "设置电机角度失败: " << motor_number << " - 参数超出有效范围" << std::endl;
        return false;
    }
    
    if (joint_angle < -1000 || joint_angle > 1000) {
        std::cout << "设置电机角度失败: " << joint_angle << " - 参数超出有效范围" << std::endl;
        return false;
    }
    
    unsigned char address = CONTROL_JOINT_MOTOR_RELATIVE_ADDRESS_6DOF + motor_number - 1;
    try {
        return send_single_data(address, static_cast<short>(joint_angle));
    } catch (...) {
        std::cout << "设置电机角度失败" << std::endl;
        return false;
    }
}

bool ZWHAND::set_all_motor_relative(const std::vector<int>& joint_angle_list) {
    if (static_cast<int>(joint_angle_list.size()) != motor_count) {
        std::cout << "设置所有电机角度失败: 数组长度不匹配!" << std::endl;
        return false;
    }
    
    for (int item : joint_angle_list) {
        if (item < -1000 || item > 1000) {
            std::cout << "设置所有电机角度失败: " << item << " - 参数超出有效范围" << std::endl;
            return false;
        }
    }
    
    std::vector<short> angles;
    for (int item : joint_angle_list) {
        angles.push_back(static_cast<short>(item));
    }
    
    return send_multiple_data(CONTROL_JOINT_MOTOR_RELATIVE_ADDRESS_6DOF, angles);
}

bool ZWHAND::set_single_motor_calibration(int motor_number) {
    if (motor_number < 1 || motor_number > motor_count) {
        std::cout << "单电机校准失败: " << motor_number << " - 参数超出有效范围" << std::endl;
        return false;
    }
    
    unsigned char address = SINGLE_MOTOR_CALIBRATION_ADDRESS_6DOF + motor_number - 1;
    try {
        return send_single_data(address, 1);
    } catch (...) {
        std::cout << "单电机校准失败" << std::endl;
        return false;
    }
}

bool ZWHAND::set_all_motor_calibration() {
    return send_multiple_data(ALL_MOTOR_CALIBRATION_ADDRESS_6DOF, {1});
}

bool ZWHAND::set_hand_test(int enable){
    try {
        // 参数校验（很重要）
        if (enable != 0 && enable != 1) {
            std::cout << "参数错误：enable 只能是 0 或 1" << std::endl;
            return false;
        }

        return send_single_data(HAND_TEST_ADDRESS_6DOF, enable);

    } catch (...) {
        std::cout << "整手测试失败" << std::endl;
        return false;
    }
}

//
bool ZWHAND::set_finger_control_mode(int mode) {
    if (mode < 0 || mode > 2) {
        std::cout << "控制模式设置失败: " << mode << " - 参数超出有效范围 (0:PC, 1:PSC, 2:PVT)" << std::endl;
        return false;
    }
    try {
        std::vector<short> mode_list(motor_count, mode);
        return send_multiple_data(FINGER_CTRL_MODE_ADDRESS_6DOF, mode_list);
    } catch (...) {
        std::cout << "控制模式设置失败" << std::endl;
        return false;
    }
}

bool ZWHAND::set_all_motor_radian(const std::vector<int>& radian_list) {
    if (radian_list.size() != motor_count) {
        std::cout << "角度设置失败: 需要" << motor_count << "个参数，实际" << radian_list.size() << std::endl;
        return false;
    }
    try {
        std::vector<short> short_radian(radian_list.begin(), radian_list.end());
        return send_multiple_data(JOINT_LOCATION_ADDRESS_6DOF, short_radian);
    } catch (...) {
        std::cout << "角度设置失败" << std::endl;
        return false;
    }
}

bool ZWHAND::set_pvt_control_param(const std::vector<uint16_t>& pvt_params) {
    // 参数数量校验（6个电机 × 3个参数 = 18个）
    if (pvt_params.size() != motor_count * 3) {
        std::cout << "PVT参数设置失败: 需要" << motor_count * 3 << "个参数，实际" << pvt_params.size() << std::endl;
        return false;
    }
    
    // 参数范围校验
    for (size_t i = 0; i < pvt_params.size(); i++) {
        uint16_t val = pvt_params[i];
        uint16_t param_type = i % 3;  // 0:位置P, 1:速度V, 2:时间T
        
        if (param_type == 0) {  // 位置 P: 0-1000
            if (val < 0 || val > 1000) {
                std::cout << "PVT参数设置失败: 位置P[" << i/3 << "] = " << val << " 超出范围[0,1000]" << std::endl;
                return false;
            }
        } else if (param_type == 1) {  // 速度 V: 0-33000
            if (val < 0 || val > 33000) {
                std::cout << "PVT参数设置失败: 速度V[" << i/3 << "] = " << val << " 超出范围[0,33000]" << std::endl;
                return false;
            }
        } else {  // 时间 T: 0 或 450-5000
            if (val != 0 && (val < 450 || val > 5000)) {
                std::cout << "PVT参数设置失败: 时间T[" << i/3 << "] = " << val << " 超出范围[450,5000]或0" << std::endl;
                return false;
            }
        }
    }
    
    // V和T互斥警告（不阻止执行，因为设备会自动处理）
    for (uint16_t i = 0; i < motor_count; i++) {
        uint16_t v = pvt_params[i * 3 + 1];
        uint16_t t = pvt_params[i * 3 + 2];
        if (v != 0 && t != 0) {
            std::cout << "PVT参数警告: 电机" << i+1 << "的V和T同时非0，将按PV模式执行" << std::endl;
        }
    }
    
    try {
        // 参数校验中用 uint16_t val
        // ...
        std::vector<short> short_params;
        for (uint16_t val : pvt_params) {
            short_params.push_back(static_cast<short>(val));
        }
        return send_multiple_data(PVT_CTRL_PARAM_ADDRESS_6DOF, short_params);
    } catch (...) {
        std::cout << "PVT参数设置失败: 通信异常" << std::endl;
        return false;
    }
}

bool ZWHAND::set_pc_control_param(const std::vector<int>& current_params) {
    // 参数数量校验（6个电机）
    if (current_params.size() != motor_count) {
        std::cout << "PC参数设置失败: 需要" << motor_count << "个参数，实际" << current_params.size() << std::endl;
        return false;
    }
    
    // 参数范围校验：-7500 ~ 7500
    for (size_t i = 0; i < current_params.size(); i++) {
        int val = current_params[i];
        if (val < -7500 || val > 7500) {
            std::cout << "PC参数设置失败: 电机" << i+1 << "电流=" << val << " 超出范围[-7500,7500]" << std::endl;
            return false;
        }
    }
    
    try {
        std::vector<short> short_params(current_params.begin(), current_params.end());
        return send_multiple_data(PC_CTRL_PARAM_ADDRESS_6DOF, short_params);
    } catch (...) {
        std::cout << "PC参数设置失败: 通信异常" << std::endl;
        return false;
    }
}



// 数据读取函数实现
short ZWHAND::get_initialize_state() {
    std::vector<short> result = read_multiple_data(INITIALIZE_DATA_ADDRESS_6DOF, 1);
    return result.empty() ? -1 : result[0];
}

short ZWHAND::get_bootloader_version() {
    std::vector<short> result = read_multiple_data(BOOTLOADER_VERSION_ADDRESS_6DOF, 1);
    return result.empty() ? -1 : result[0];
}

short ZWHAND::get_hardware_version() {
    std::vector<short> result = read_multiple_data(HARDWARE_VERSION_ADDRESS_6DOF, 1);
    return result.empty() ? -1 : result[0];
}

short ZWHAND::get_software_version() {
    std::vector<short> result = read_multiple_data(SOFTWARE_VERSION_ADDRESS_6DOF, 1);
    return result.empty() ? -1 : result[0];
}

std::vector<short> ZWHAND::get_device_error() {
    return read_multiple_data(HALL_ERROR_ADDRESS_6DOF, 6);
}

short ZWHAND::get_device_voltage() {
    std::vector<short> result = read_multiple_data(DEVICE_VOLTAGE_ADDRESS_6DOF, 1);
    return result.empty() ? -1 : result[0];
}

std::vector<short> ZWHAND::get_motor_locked_state() {
    return read_multiple_data(MOTOR_LOCK_STATE_ADDRESS_6DOF, motor_count);
}

std::vector<short> ZWHAND::get_motor_real_angle() {
    return read_multiple_data(MOTOR_ANGLE_ADDRESS_6DOF, motor_count);
}

std::vector<short> ZWHAND::get_joint_skin_moment() {
    return read_multiple_data(JOINT_SKIN_ADDRESS, 5);
}

// 从Python代码添加的函数实现
std::vector<short> ZWHAND::get_phase_loss_fault() {
    return read_multiple_data(PHASE_LOSS_FAULT_ADDRESS_6DOF, motor_count);
}

std::vector<short> ZWHAND::get_cur_samp_fault() {
    return read_multiple_data(CUR_SAMP_ERROR_ADDRESS_6DOF, motor_count);
}

std::vector<short> ZWHAND::get_all_motor_speed() {
    return read_multiple_data(MOTOR_SPEED_ADDRESS_6DOF, motor_count);
}

std::vector<short> ZWHAND::get_all_motor_current() {
    return read_multiple_data(MOTOR_CURRENT_ADDRESS_6DOF, motor_count);
}

std::vector<short> ZWHAND::get_all_motor_radian() {
    return read_multiple_data(MOTOR_RADIAN_ADDRESS_6DOF, motor_count);
}

std::vector<short> ZWHAND::get_all_motor_max_radian() {
    return read_multiple_data(JOINT_ABS_MAX_ADDRESS_6DOF, motor_count);
}

short ZWHAND::get_cust_number() {
    std::vector<short> result = read_multiple_data(CUST_NUMBER_ADDRESS_6DOF, 1);
    return result.empty() ? -1 : result[0];
}

std::vector<short> ZWHAND::get_ele_skin_force_infor() {
    return read_multiple_data(ELE_SKIN_FORCE_INFOR_ADDRESS_6DOF, 6);  // 6个力传感器
}

std::vector<short> ZWHAND::get_all_motor_temperature() {
    return read_multiple_data(MOTOR_TEMPERATURE_ADDRESS_6DOF, motor_count);
}

std::vector<short> ZWHAND::get_motor_hall_fault() {
    return read_multiple_data(HALL_ERROR_ADDRESS_6DOF, motor_count);
}

short ZWHAND::get_hand_type()
{
    std::vector<short> result = read_multiple_data(HAND_TYPE_ADDRESS, 1);
    if (result.empty()) {
        return -1;   // 错误返回值，可根据实际约定调整
    }
    return result[0];   // 返回 0 或 1
}

