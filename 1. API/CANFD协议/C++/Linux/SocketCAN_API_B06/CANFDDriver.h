#ifndef CANFD_DRIVER_H
#define CANFD_DRIVER_H

#include <chrono>
#include <cstring>
#include <iostream>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <mutex>
#include <net/if.h>
#include <string>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

class CANFDDriver {
private:
    int socket_fd;
    std::string channel;
    uint32_t arb_baud_rate;
    uint32_t data_baud_rate;
    uint8_t lqs_id;
    bool is_connected;
    std::mutex mtx;

    static const uint8_t INITIAL_LQS_ID = 0x01;
    static constexpr double RECEIVE_DETECT_TIME = 0.05;
    static constexpr double TIMEOUT = 0.1;
    static const uint8_t MOTOR_COUNT = 6;
    static const uint8_t WRITE_SINGLE_FUN_TYPE = 0x06;
    static const uint8_t WRITE_MULTIPLE_FUN_TYPE = 0x10;
    static const uint8_t READ_FUN_TYPE = 0x04;

    // 保持寄存器地址
    static const uint8_t SET_ID_ADDRESS = 0x00;
    static const uint8_t SET_BAUD_ADDRESS = 0x01;
    static const uint8_t CLEAR_ERROR_ADDRESS = 0x02;
    static const uint8_t SET_POWER_OFF_SAVE_ADDRESS = 0x03;
    static const uint8_t FACTORY_DATA_RESET_ADDRESS = 0x04;
    static const uint8_t SINGLE_MOTOR_CALIBRATION_ADDRESS = 0x05;
    static const uint8_t CONTROL_JOINT_MOTOR_ABSOLUTE_ADDRESS = 0x0B;
    static const uint8_t CONTROL_JOINT_MOTOR_RELATIVE_ADDRESS = 0x11;
    static const uint8_t SET_MOTOR_STOP_ADDRESS = 0x17;
    static const uint8_t SET_SPEED_ADDRESS = 0x1D;
    static const uint8_t SET_CURRENT_ADDRESS = 0x23;
    static const uint8_t ALL_MOTOR_CALIBRATION_ADDRESS = 0x29;
    static const uint8_t HAND_TEST_ADDRESS = 0x2A;
    static const uint8_t SET_JOINT_LOCATION_ADDRESS = 0x2B;
    static const uint8_t SET_FINGER_CTRL_MODE_ADDRESS = 0x31;
    static const uint8_t SET_PVT_CTRL_PARAM_ADDRESS = 0x37;
    static const uint8_t SET_PC_CTRL_PARAM_ADDRESS = 0x49;

    // 输入寄存器地址
    static const uint8_t INITIALIZE_DATA_ADDRESS = 0x00;
    static const uint8_t BOOTLOADER_VERSION_ADDRESS = 0x01;
    static const uint8_t HARDWARE_VERSION_ADDRESS = 0x02;
    static const uint8_t SOFTWARE_VERSION_ADDRESS = 0x03;
    static const uint8_t HALL_ERROR_ADDRESS = 0x04;
    static const uint8_t DEVICE_VOLTAGE_ADDRESS = 0x0A;
    static const uint8_t ABS_MAX_POSITION_ADDRESS = 0x0B;
    static const uint8_t MOTOR_LOCK_STATE_ADDRESS = 0x11;
    static const uint8_t MOTOR_POSITION_ADDRESS = 0x17;
    static const uint8_t MOTOR_SPEED_ADDRESS = 0x1D;
    static const uint8_t MOTOR_CURRENT_ADDRESS = 0x23;
    static const uint8_t PHASE_LOSS_FAULT_ADDRESS = 0x29;
    static const uint8_t CUR_SAMP_ERROR_ADDRESS = 0x2F;
    static const uint8_t MOTOR_ANGLE_ADDRESS = 0x35;
    static const uint8_t JOINT_ABS_MAX_ADDRESS = 0x3B;
    static const uint8_t CUST_NUMBER_ADDRESS = 0x41;
    static const uint8_t SKIN_FORCE_ADDRESS = 0x42;
    static const uint8_t MOTOR_TEMPERATURE_ADDRESS = 0x48;
    static const uint8_t HAND_TYPE_ADDRESS = 0x4E;

public:
    CANFDDriver(const std::string& channel = "can0",
                uint32_t arb_baud_rate = 500000,
                uint32_t data_baud_rate = 1000000,
                uint8_t lqs_id = 0x01);
    ~CANFDDriver();

    bool connect();
    void disconnect();
    bool isConnected() const { return is_connected; }

    bool sendMessage(const std::vector<uint8_t>& data, bool is_extended_id = false);
    std::vector<uint8_t> receiveMessage(uint8_t fun_type);
    void clearReceiveBuffer();

    static std::vector<uint8_t> calculateCRC(const std::vector<uint8_t>& data);

    bool writeSingleAddress(uint8_t start_address, int16_t data);
    bool writeMultipleAddress(uint8_t start_address, const std::vector<int16_t>& data);
    // unsigned multi-write path for unsigned register payloads
    bool writeMultipleAddress1(uint8_t start_address, const std::vector<uint16_t>& data);

    std::vector<int16_t> readCommand(uint8_t start_address, uint8_t address_len);

    bool setId(int16_t new_id);
    bool setBaud(int16_t baud_order);
    bool setErrorClear();
    bool setPowerOffSave(int16_t save_type);
    bool setFactoryDataReset();

    bool setSingleMotorSpeed(uint8_t motor_number, int16_t speed);
    bool setAllMotorSpeed(const std::vector<int16_t>& speed_list);
    bool setSingleMotorCurrent(uint8_t motor_number, int16_t current);
    bool setAllMotorCurrent(const std::vector<int16_t>& current_list);
    bool setSingleMotorStop(uint8_t motor_number);
    bool setAllMotorStop(const std::vector<int16_t>& stop_flag_list);
    bool setAllMotorAngle(const std::vector<int16_t>& joint_location_list);
    bool setSingleMotorAbsolute(uint8_t motor_number, int16_t joint_angle);
    bool setAllMotorAbsolute(const std::vector<int16_t>& joint_angle_list);
    bool setSingleMotorRelative(uint8_t motor_number, int16_t joint_angle);
    bool setAllMotorRelative(const std::vector<int16_t>& joint_angle_list);
    bool setSingleMotorCalibration(uint8_t motor_number);
    bool setAllMotorCalibration();
    bool setFingerControlMode(const std::vector<int16_t>& mode_list);
    bool setPvtControlParam(const std::vector<uint16_t>& param_list);
    bool setPcControlParam(const std::vector<int16_t>& param_list);

    int16_t getInitializeState();
    int16_t getBootloaderVersion();
    int16_t getHardwareVersion();
    int16_t getSoftwareVersion();
    std::vector<int16_t> getDeviceError();
    float getDeviceVoltage();
    std::vector<int16_t> getMotorLockedState();
    std::vector<int16_t> getMotorRealPosition();
    std::vector<int16_t> getAllMotorSpeed();
    std::vector<int16_t> getAllMotorCurrent();
    std::vector<int16_t> getPhaseLossFault();
    std::vector<int16_t> getCurSampFault();
    std::vector<float> getAllMotorAngle();

    std::vector<int16_t> getAbsMaxPosition();
    std::vector<float> getJointAbsMax();
    int16_t getCustomerNumber();
    std::vector<float> getSkinForce();
    std::vector<float> getMotorTemperature();
    int16_t getHandType();
    bool setHandTest(int16_t enable);

private:
    std::vector<uint8_t> uintToBytes(uint16_t value);
    std::vector<uint8_t> intToBytes(int16_t value);
    int16_t bytesToInt(const std::vector<uint8_t>& bytes, size_t offset);
};

#endif // CANFD_DRIVER_H
