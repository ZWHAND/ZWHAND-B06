#include "CANFDDriver.h"

#include <algorithm>
#include <cerrno>
#include <iomanip>

CANFDDriver::CANFDDriver(const std::string& channel, uint32_t arb_baud_rate,
                         uint32_t data_baud_rate, uint8_t lqs_id)
    : socket_fd(-1),
      channel(channel),
      arb_baud_rate(arb_baud_rate),
      data_baud_rate(data_baud_rate),
      lqs_id(lqs_id),
      is_connected(false) {}

CANFDDriver::~CANFDDriver() {
    disconnect();
}

bool CANFDDriver::connect() {
    socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd < 0) {
        std::cerr << "Failed to create socket" << std::endl;
        return false;
    }

    int enable_canfd = 1;
    if (setsockopt(socket_fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
                   &enable_canfd, sizeof(enable_canfd)) < 0) {
        std::cerr << "Failed to enable CAN FD" << std::endl;
        close(socket_fd);
        socket_fd = -1;
        return false;
    }

    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, channel.c_str(), IFNAMSIZ - 1);
    if (ioctl(socket_fd, SIOCGIFINDEX, &ifr) < 0) {
        std::cerr << "Failed to get interface index" << std::endl;
        close(socket_fd);
        socket_fd = -1;
        return false;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::cerr << "Failed to bind socket" << std::endl;
        close(socket_fd);
        socket_fd = -1;
        return false;
    }

    is_connected = true;
    std::cout << "Connected to CAN FD bus on " << channel << std::endl;
    return true;
}

void CANFDDriver::disconnect() {
    if (socket_fd >= 0) {
        close(socket_fd);
        socket_fd = -1;
    }
    is_connected = false;
}

void CANFDDriver::clearReceiveBuffer() {
    if (!is_connected) {
        std::cerr << "Not connected to CAN bus" << std::endl;
        return;
    }

    struct canfd_frame frame;
    socklen_t len = sizeof(struct sockaddr_can);
    struct sockaddr_can addr;
    int cleared_count = 0;

    while (true) {
        ssize_t nbytes = recvfrom(socket_fd, &frame, sizeof(frame), MSG_DONTWAIT,
                                  reinterpret_cast<struct sockaddr*>(&addr), &len);
        if (nbytes < 0) {
            break;
        }
        ++cleared_count;
    }

    if (cleared_count > 0) {
        std::cout << "Cleared " << cleared_count << " messages from receive buffer" << std::endl;
    }
}

bool CANFDDriver::sendMessage(const std::vector<uint8_t>& data, bool is_extended_id) {
    (void)is_extended_id;

    if (!is_connected) {
        std::cerr << "Not connected to CAN bus" << std::endl;
        return false;
    }

    if (data.size() > 64) {
        std::cerr << "Data length exceeds maximum CAN FD payload (64 bytes)" << std::endl;
        return false;
    }

    std::vector<uint8_t> padded_data = data;
    size_t data_len = padded_data.size();

    if (data_len > 8 && data_len <= 24) {
        while (padded_data.size() % 4 != 0) {
            padded_data.push_back(0);
        }
    } else if (data_len > 24 && data_len <= 32) {
        while (padded_data.size() % 8 != 0) {
            padded_data.push_back(0);
        }
    } else if (data_len > 32 && data_len <= 64) {
        while (padded_data.size() % 16 != 0) {
            padded_data.push_back(0);
        }
    }

    struct canfd_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = lqs_id;
    frame.len = static_cast<__u8>(padded_data.size());
    frame.flags |= CANFD_BRS;
    std::memcpy(frame.data, padded_data.data(),
                std::min(padded_data.size(), sizeof(frame.data)));

    clearReceiveBuffer();
    ssize_t nbytes = write(socket_fd, &frame, sizeof(frame));
    if (nbytes != sizeof(frame)) {
        std::cerr << "Failed to send CAN FD message"
                  << ", errno=" << errno
                  << " (" << std::strerror(errno) << ")"
                  << ", can_id=0x" << std::hex << static_cast<int>(frame.can_id)
                  << ", len=" << std::dec << static_cast<int>(frame.len)
                  << std::endl;
        return false;
    }

    return true;
}

std::vector<uint8_t> CANFDDriver::receiveMessage(uint8_t fun_type) {
    if (!is_connected) {
        std::cerr << "Not connected to CAN bus" << std::endl;
        return {};
    }

    auto start_time = std::chrono::steady_clock::now();
    struct canfd_frame frame;
    socklen_t len = sizeof(struct sockaddr_can);
    struct sockaddr_can addr;

    while (std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now() - start_time).count() <
           RECEIVE_DETECT_TIME * 1000) {
        ssize_t nbytes = recvfrom(socket_fd, &frame, sizeof(frame), MSG_DONTWAIT,
                                  reinterpret_cast<struct sockaddr*>(&addr), &len);
        if (nbytes > 0 && (frame.can_id & CAN_EFF_MASK) == lqs_id) {
            std::vector<uint8_t> message_data(frame.data, frame.data + frame.len);

            if (fun_type == READ_FUN_TYPE && message_data.size() > 2 && message_data[1] == fun_type) {
                size_t receive_number = static_cast<size_t>(message_data[2]) + 5;
                if (receive_number <= message_data.size()) {
                    return std::vector<uint8_t>(message_data.begin() + 3,
                                                message_data.begin() + 3 + message_data[2]);
                }
            } else if ((fun_type == WRITE_SINGLE_FUN_TYPE || fun_type == WRITE_MULTIPLE_FUN_TYPE) &&
                       message_data.size() > 1 && message_data[1] == fun_type) {
                return std::vector<uint8_t>(message_data.begin(),
                                            message_data.begin() + std::min<size_t>(8, message_data.size()));
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    return {};
}

std::vector<uint8_t> CANFDDriver::calculateCRC(const std::vector<uint8_t>& data) {
    uint16_t crc = 0xFFFF;
    std::vector<uint8_t> result = data;

    for (uint8_t byte : data) {
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

    result.push_back(static_cast<uint8_t>(crc & 0xFF));
    result.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
    return result;
}

std::vector<uint8_t> CANFDDriver::uintToBytes(uint16_t value) {
    return {
        static_cast<uint8_t>((value >> 8) & 0xFF),
        static_cast<uint8_t>(value & 0xFF)
    };
}

std::vector<uint8_t> CANFDDriver::intToBytes(int16_t value) {
    return {
        static_cast<uint8_t>((static_cast<uint16_t>(value) >> 8) & 0xFF),
        static_cast<uint8_t>(static_cast<uint16_t>(value) & 0xFF)
    };
}

int16_t CANFDDriver::bytesToInt(const std::vector<uint8_t>& bytes, size_t offset) {
    if (offset + 1 >= bytes.size()) {
        return 0;
    }
    uint16_t value = static_cast<uint16_t>(bytes[offset]) << 8;
    value |= static_cast<uint16_t>(bytes[offset + 1]);
    return static_cast<int16_t>(value);
}

bool CANFDDriver::writeSingleAddress(uint8_t start_address, int16_t data) {
    std::lock_guard<std::mutex> lock(mtx);

    std::vector<uint8_t> cmd_data(6);
    cmd_data[0] = lqs_id;
    cmd_data[1] = WRITE_SINGLE_FUN_TYPE;
    cmd_data[2] = 0x00;
    cmd_data[3] = start_address;

    std::vector<uint8_t> bytes = intToBytes(data);
    cmd_data[4] = bytes[0];
    cmd_data[5] = bytes[1];

    return sendMessage(calculateCRC(cmd_data));
}

bool CANFDDriver::writeMultipleAddress(uint8_t start_address, const std::vector<int16_t>& data) {
    std::lock_guard<std::mutex> lock(mtx);

    size_t address_len = data.size();
    std::vector<uint8_t> cmd_data(7 + address_len * 2);
    cmd_data[0] = lqs_id;
    cmd_data[1] = WRITE_MULTIPLE_FUN_TYPE;
    cmd_data[2] = 0x00;
    cmd_data[3] = start_address;
    cmd_data[4] = 0x00;
    cmd_data[5] = static_cast<uint8_t>(address_len);
    cmd_data[6] = static_cast<uint8_t>(address_len * 2);

    for (size_t i = 0; i < address_len; ++i) {
        std::vector<uint8_t> bytes = intToBytes(data[i]);
        cmd_data[i * 2 + 7] = bytes[0];
        cmd_data[i * 2 + 8] = bytes[1];
    }

    return sendMessage(calculateCRC(cmd_data));
}

bool CANFDDriver::writeMultipleAddress1(uint8_t start_address, const std::vector<uint16_t>& data) {
    std::lock_guard<std::mutex> lock(mtx);

    size_t address_len = data.size();
    std::vector<uint8_t> cmd_data(7 + address_len * 2);
    cmd_data[0] = lqs_id;
    cmd_data[1] = WRITE_MULTIPLE_FUN_TYPE;
    cmd_data[2] = 0x00;
    cmd_data[3] = start_address;
    cmd_data[4] = 0x00;
    cmd_data[5] = static_cast<uint8_t>(address_len);
    cmd_data[6] = static_cast<uint8_t>(address_len * 2);

    for (size_t i = 0; i < address_len; ++i) {
        std::vector<uint8_t> bytes = uintToBytes(data[i]);
        cmd_data[i * 2 + 7] = bytes[0];
        cmd_data[i * 2 + 8] = bytes[1];
    }

    return sendMessage(calculateCRC(cmd_data));
}

std::vector<int16_t> CANFDDriver::readCommand(uint8_t start_address, uint8_t address_len) {
    std::lock_guard<std::mutex> lock(mtx);

    std::vector<uint8_t> cmd_data(6);
    cmd_data[0] = lqs_id;
    cmd_data[1] = READ_FUN_TYPE;
    cmd_data[2] = 0x00;
    cmd_data[3] = start_address;
    cmd_data[4] = 0x00;
    cmd_data[5] = address_len;

    if (!sendMessage(calculateCRC(cmd_data))) {
        return {};
    }

    std::vector<uint8_t> received_data = receiveMessage(READ_FUN_TYPE);
    if (received_data.empty()) {
        return {};
    }

    std::vector<int16_t> result;
    for (size_t i = 0; i + 1 < received_data.size(); i += 2) {
        result.push_back(bytesToInt(received_data, i));
    }
    return result;
}

bool CANFDDriver::setId(int16_t new_id) {
    if (new_id < 1 || new_id > 255) {
        return false;
    }
    if (writeSingleAddress(SET_ID_ADDRESS, new_id)) {
        lqs_id = static_cast<uint8_t>(new_id);
        return true;
    }
    return false;
}

bool CANFDDriver::setBaud(int16_t baud_order) {
    return baud_order >= 1 && baud_order <= 5 &&
           writeSingleAddress(SET_BAUD_ADDRESS, baud_order);
}

bool CANFDDriver::setErrorClear() {
    return writeSingleAddress(CLEAR_ERROR_ADDRESS, 1);
}

bool CANFDDriver::setPowerOffSave(int16_t save_type) {
    return save_type >= 1 && save_type <= 2 &&
           writeSingleAddress(SET_POWER_OFF_SAVE_ADDRESS, save_type);
}

bool CANFDDriver::setFactoryDataReset() {
    if (!writeSingleAddress(FACTORY_DATA_RESET_ADDRESS, 1)) {
        return false;
    }
    lqs_id = INITIAL_LQS_ID;
    disconnect();
    return connect();
}

bool CANFDDriver::setSingleMotorSpeed(uint8_t motor_number, int16_t speed) {
    if (motor_number < 1 || motor_number > MOTOR_COUNT || speed < 1 || speed > 100) {
        return false;
    }
    return writeSingleAddress(static_cast<uint8_t>(SET_SPEED_ADDRESS + motor_number - 1), speed);
}

bool CANFDDriver::setAllMotorSpeed(const std::vector<int16_t>& speed_list) {
    if (speed_list.size() != MOTOR_COUNT) {
        return false;
    }
    for (int16_t speed : speed_list) {
        if (speed < 1 || speed > 100) {
            return false;
        }
    }
    return writeMultipleAddress(SET_SPEED_ADDRESS, speed_list);
}

bool CANFDDriver::setSingleMotorCurrent(uint8_t motor_number, int16_t current) {
    if (motor_number < 1 || motor_number > MOTOR_COUNT || current < 1 || current > 100) {
        return false;
    }
    return writeSingleAddress(static_cast<uint8_t>(SET_CURRENT_ADDRESS + motor_number - 1), current);
}

bool CANFDDriver::setAllMotorCurrent(const std::vector<int16_t>& current_list) {
    if (current_list.size() != MOTOR_COUNT) {
        return false;
    }
    for (int16_t current : current_list) {
        if (current < 1 || current > 100) {
            return false;
        }
    }
    return writeMultipleAddress(SET_CURRENT_ADDRESS, current_list);
}

bool CANFDDriver::setSingleMotorStop(uint8_t motor_number) {
    if (motor_number < 1 || motor_number > MOTOR_COUNT) {
        return false;
    }
    return writeSingleAddress(static_cast<uint8_t>(SET_MOTOR_STOP_ADDRESS + motor_number - 1), 1);
}

bool CANFDDriver::setAllMotorStop(const std::vector<int16_t>& stop_flag_list) {
    if (stop_flag_list.size() != MOTOR_COUNT) {
        return false;
    }
    for (int16_t flag : stop_flag_list) {
        if (flag != 0 && flag != 1) {
            return false;
        }
    }
    return writeMultipleAddress(SET_MOTOR_STOP_ADDRESS, stop_flag_list);
}

bool CANFDDriver::setAllMotorAngle(const std::vector<int16_t>& joint_location_list) {
    if (joint_location_list.size() != MOTOR_COUNT) {
        return false;
    }
    return writeMultipleAddress(SET_JOINT_LOCATION_ADDRESS, joint_location_list);
}

bool CANFDDriver::setSingleMotorAbsolute(uint8_t motor_number, int16_t joint_angle) {
    if (motor_number < 1 || motor_number > MOTOR_COUNT) {
        return false;
    }
    return writeSingleAddress(static_cast<uint8_t>(CONTROL_JOINT_MOTOR_ABSOLUTE_ADDRESS + motor_number - 1),
                              joint_angle);
}

bool CANFDDriver::setAllMotorAbsolute(const std::vector<int16_t>& joint_angle_list) {
    if (joint_angle_list.size() != MOTOR_COUNT) {
        return false;
    }
    return writeMultipleAddress(CONTROL_JOINT_MOTOR_ABSOLUTE_ADDRESS, joint_angle_list);
}

bool CANFDDriver::setSingleMotorRelative(uint8_t motor_number, int16_t joint_angle) {
    if (motor_number < 1 || motor_number > MOTOR_COUNT) {
        return false;
    }
    return writeSingleAddress(static_cast<uint8_t>(CONTROL_JOINT_MOTOR_RELATIVE_ADDRESS + motor_number - 1),
                              joint_angle);
}

bool CANFDDriver::setAllMotorRelative(const std::vector<int16_t>& joint_angle_list) {
    if (joint_angle_list.size() != MOTOR_COUNT) {
        return false;
    }
    return writeMultipleAddress(CONTROL_JOINT_MOTOR_RELATIVE_ADDRESS, joint_angle_list);
}

bool CANFDDriver::setSingleMotorCalibration(uint8_t motor_number) {
    if (motor_number < 1 || motor_number > MOTOR_COUNT) {
        return false;
    }
    return writeSingleAddress(static_cast<uint8_t>(SINGLE_MOTOR_CALIBRATION_ADDRESS + motor_number - 1), 1);
}

bool CANFDDriver::setAllMotorCalibration() {
    return writeSingleAddress(ALL_MOTOR_CALIBRATION_ADDRESS, 1);
}

bool CANFDDriver::setFingerControlMode(const std::vector<int16_t>& mode_list) {
    if (mode_list.size() != MOTOR_COUNT) {
        return false;
    }
    for (int16_t mode : mode_list) {
        if (mode < 0 || mode > 2) {
            return false;
        }
    }
    return writeMultipleAddress(SET_FINGER_CTRL_MODE_ADDRESS, mode_list);
}

bool CANFDDriver::setPvtControlParam(const std::vector<uint16_t>& param_list) {
    if (param_list.size() != 18) {
        return false;
    }
    return writeMultipleAddress1(SET_PVT_CTRL_PARAM_ADDRESS, param_list);
}

bool CANFDDriver::setPcControlParam(const std::vector<int16_t>& param_list) {
    if (param_list.size() != MOTOR_COUNT) {
        return false;
    }
    for (int16_t param : param_list) {
        if (param < -7500 || param > 7500) {
            return false;
        }
    }
    return writeMultipleAddress(SET_PC_CTRL_PARAM_ADDRESS, param_list);
}

int16_t CANFDDriver::getInitializeState() {
    auto result = readCommand(INITIALIZE_DATA_ADDRESS, 1);
    return result.empty() ? 0 : result[0];
}

int16_t CANFDDriver::getBootloaderVersion() {
    auto result = readCommand(BOOTLOADER_VERSION_ADDRESS, 1);
    return result.empty() ? 0 : result[0];
}

int16_t CANFDDriver::getHardwareVersion() {
    auto result = readCommand(HARDWARE_VERSION_ADDRESS, 1);
    return result.empty() ? 0 : result[0];
}

int16_t CANFDDriver::getSoftwareVersion() {
    auto result = readCommand(SOFTWARE_VERSION_ADDRESS, 1);
    return result.empty() ? 0 : result[0];
}

std::vector<int16_t> CANFDDriver::getDeviceError() {
    return readCommand(HALL_ERROR_ADDRESS, 6);
}

float CANFDDriver::getDeviceVoltage() {
    auto result = readCommand(DEVICE_VOLTAGE_ADDRESS, 1);
    return result.empty() ? 0.0f : static_cast<float>(result[0]) * 0.001f;
}

std::vector<int16_t> CANFDDriver::getMotorLockedState() {
    return readCommand(MOTOR_LOCK_STATE_ADDRESS, MOTOR_COUNT);
}

std::vector<int16_t> CANFDDriver::getMotorRealPosition() {
    return readCommand(MOTOR_POSITION_ADDRESS, MOTOR_COUNT);
}

std::vector<int16_t> CANFDDriver::getAllMotorSpeed() {
    return readCommand(MOTOR_SPEED_ADDRESS, MOTOR_COUNT);
}

std::vector<int16_t> CANFDDriver::getAllMotorCurrent() {
    return readCommand(MOTOR_CURRENT_ADDRESS, MOTOR_COUNT);
}

std::vector<int16_t> CANFDDriver::getPhaseLossFault() {
    return readCommand(PHASE_LOSS_FAULT_ADDRESS, MOTOR_COUNT);
}

std::vector<int16_t> CANFDDriver::getCurSampFault() {
    return readCommand(CUR_SAMP_ERROR_ADDRESS, MOTOR_COUNT);
}

std::vector<float> CANFDDriver::getAllMotorAngle() {
    auto raw = readCommand(MOTOR_ANGLE_ADDRESS, MOTOR_COUNT);
    std::vector<float> result;
    result.reserve(raw.size());
    for (int16_t value : raw) {
        result.push_back(static_cast<float>(value) * 0.1f);
    }
    return result;
}

std::vector<int16_t> CANFDDriver::getAbsMaxPosition() {
    return readCommand(ABS_MAX_POSITION_ADDRESS, MOTOR_COUNT);
}

std::vector<float> CANFDDriver::getJointAbsMax() {
    auto raw = readCommand(JOINT_ABS_MAX_ADDRESS, MOTOR_COUNT);
    std::vector<float> result;
    result.reserve(raw.size());
    for (int16_t value : raw) {
        result.push_back(static_cast<float>(value) * 0.1f);
    }
    return result;
}

int16_t CANFDDriver::getCustomerNumber() {
    auto result = readCommand(CUST_NUMBER_ADDRESS, 1);
    return result.empty() ? 0 : result[0];
}

std::vector<float> CANFDDriver::getSkinForce() {
    auto raw = readCommand(SKIN_FORCE_ADDRESS, MOTOR_COUNT);
    std::vector<float> result;
    result.reserve(raw.size());
    for (int16_t value : raw) {
        result.push_back(static_cast<float>(value) * 0.01f);
    }
    return result;
}

std::vector<float> CANFDDriver::getMotorTemperature() {
    auto raw = readCommand(MOTOR_TEMPERATURE_ADDRESS, MOTOR_COUNT);
    std::vector<float> result;
    result.reserve(raw.size());
    for (int16_t value : raw) {
        result.push_back(static_cast<float>(value) * 0.1f);
    }
    return result;
}

int16_t CANFDDriver::getHandType() {
    auto result = readCommand(HAND_TYPE_ADDRESS, 1);
    return result.empty() ? 0 : result[0];
}

bool CANFDDriver::setHandTest(int16_t enable) {
    return writeSingleAddress(HAND_TEST_ADDRESS, enable);
}
