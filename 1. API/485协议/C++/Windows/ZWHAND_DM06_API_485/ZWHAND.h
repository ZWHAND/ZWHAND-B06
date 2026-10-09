#ifndef ZWHAND_H
#define ZWHAND_H

#include <windows.h>
#include <vector>
#include <string>
#include <cstdint>

class ZWHAND {
public:
    // 构造函数
    ZWHAND(int lqs_id, const std::string& port, int baud);
    
    // 析构函数
    ~ZWHAND();
    
    // 串口操作
    bool open_start_device();
    bool close_device();
    
    // 通信协议函数
    std::vector<unsigned char> modbus_data_receive(const std::vector<unsigned char>& detect_data, int receive_data_len);
    static std::vector<unsigned char> modbus_crc(const std::vector<unsigned char>& data);
    bool send_single_data(unsigned char address, short data);
    bool send_multiple_data(unsigned char start_address, const std::vector<short>& data);
    std::vector<short> read_multiple_data(unsigned char start_address, unsigned char address_len);
    
    // 设备配置函数
    bool set_id(int lqs_id);                                                    //设置ID
    bool set_baud(int baud_order, bool is_restart = true);                      //设置波特率
    bool set_error_clear();                                                     //清除错误
    bool set_power_off_save(int save_type);                                     //设置掉电保存
    bool set_factory_data_reset();                                              //恢复出厂设置
    
    // 电机控制函数
    bool set_single_motor_speed(int motor_number, int speed);                   //设置单电机速度
    bool set_all_motor_speed(int speed);                                        //设置所有电机速度
    bool set_single_motor_current(int motor_number, int current);               //设置单电机电流
    bool set_all_motor_current(int current);                                    //设置所有电机电流
    bool set_single_motor_stop(int motor_number);                               //设置单电机停止
    bool set_all_motor_stop();                                                  //设置所有电机停止
    bool set_single_motor_absolute(int motor_number, int joint_angle);          //设置单电机绝对关节角度
    bool set_all_motor_absolute(const std::vector<int>& joint_angle_list);      //设置所有关节电机绝对角度
    bool set_single_motor_relative(int motor_number, int joint_angle);          //设置单电机相对关节角度
    bool set_all_motor_relative(const std::vector<int>& joint_angle_list);      //设置所有关节电机相对角度
    bool set_single_motor_calibration(int motor_number);                        //设置单电机校准
    bool set_all_motor_calibration();                                           //设置所有电机校准
    bool set_all_step_motor_calibration();                                      //设置所有步进电机校准
    bool set_hand_test(int enable);                                                   // 整手测试
    bool set_finger_control_mode(int mode);                                 // 控制模式
    bool set_all_motor_radian(const std::vector<int>& radian_list);         // 角度制位置
    bool set_pvt_control_param(const std::vector<uint16_t>& pvt_params);         // PVT参数
    bool set_pc_control_param(const std::vector<int>& current_params);      // PC参数





    
    // 数据读取函数
    short get_initialize_state();                                               //获取初始化状态
    short get_bootloader_version();                                             //获取bootloader版本
    short get_hardware_version();                                               //获取硬件版本
    short get_software_version();                                               //获取软件版本
    std::vector<short> get_device_error();                                      //获取设备错误
    short get_device_voltage();                                                 //获取设备电压
    std::vector<short> get_motor_locked_state();                                //获取电机堵转状态
    std::vector<short> get_motor_real_angle();                                  //获取电机实际角度
    std::vector<short> get_joint_skin_moment();                                 //获取指尖力矩
    std::vector<short> get_all_motor_radian();                              // 获取电机弧度
    std::vector<short> get_all_motor_max_radian();                          // 获取关节最大角度
    short get_cust_number();                                                // 获取客户编号
    std::vector<short> get_ele_skin_force_infor();                          // 获取电子皮肤力
    std::vector<short> get_all_motor_temperature();                         // 获取电机温度
    std::vector<short> get_motor_hall_fault();                                  // 获取霍尔故障状态
    short get_hand_type();                                                  // 获取手类型，返回 0（右手）或 1（左手）

    // 从Python代码添加的函数
    bool set_single_motor_absolute_6dof(int motor_number, int joint_angle);     // 设置单个电机绝对位置角度挡位 (6DOF)
    bool set_all_motor_absolute_6dof(const std::vector<int>& joint_angle_list); // 设置所有关节电机绝对位置角度挡位 (6DOF)
    bool set_single_motor_relative_6dof(int motor_number, int joint_angle);     // 设置单个关节电机相对位置角度挡位 (6DOF)
    bool set_all_motor_relative_6dof(const std::vector<int>& joint_angle_list); // 设置所有关节电机相对位置角度挡位 (6DOF)
    bool set_single_motor_calibration_6dof(int motor_number);                   // 单个关节电机零位校准 (6DOF)
    bool set_all_motor_calibration_6dof();                                      // 设置全关节电机(整手)零位校准 (6DOF)
    std::vector<short> get_phase_loss_fault();                                  // 获取所有电机缺相状态
    std::vector<short> get_cur_samp_fault();                                    // 获取所有电机电流采样错误状态
    std::vector<short> get_all_motor_speed();                                   // 获取所有关节电机实际速度
    std::vector<short> get_all_motor_current();                                 // 获取所有关节电机实际电流

private:
    HANDLE hSerial;
    std::string port;
    int baudrate;
    int lqs_id;
    int motor_count;
    int initial_lqs_id;
    int initial_baud;
    
    // 波特率等级
    std::vector<int> BAUD_RATE_LEVELS;
    
    // 寄存器地址定义
    static const unsigned char SET_ID_ADDRESS = 0x00;
    static const unsigned char SET_BAUD_ADDRESS = 0x01;
    static const unsigned char CLEAR_ERROR_ADDRESS = 0x02;
    static const unsigned char SET_POWER_OFF_SAVE_ADDRESS = 0x03;
    static const unsigned char FACTORY_DATA_RESET_ADDRESS = 0x04;
    static const unsigned char SINGLE_MOTOR_CALIBRATION_ADDRESS = 0x05;
    static const unsigned char CONTROL_JOINT_MOTOR_ABSOLUTE_ADDRESS = 0x0B;
    static const unsigned char CONTROL_JOINT_MOTOR_RELATIVE_ADDRESS = 0x11;
    static const unsigned char SET_MOTOR_STOP_ADDRESS = 0x17;
    static const unsigned char SET_SPEED_ADDRESS = 0x1D;
    static const unsigned char SET_CURRENT_ADDRESS = 0x23;
    static const unsigned char ALL_MOTOR_CALIBRATION_ADDRESS = 0x29;
    static const unsigned char HAND_TEST_ADDRESS = 0x2A;                   //  整手测试
    static const unsigned char JOINT_LOCATION_ADDRESS = 0x2B;              //  角度制位置
    static const unsigned char FINGER_CTRL_MODE_ADDRESS = 0x31;            //  控制模式
    static const unsigned char PVT_CTRL_PARAM_ADDRESS = 0x37;              //  PVT参数
    static const unsigned char PC_CTRL_PARAM_ADDRESS = 0x49;               //  PC参数
    static const unsigned char ALL_STEP_MOTOR_CALIBRATION_ADDRESS = 0x6B;
    static const unsigned char INITIALIZE_DATA_ADDRESS = 0x00;
    static const unsigned char BOOTLOADER_VERSION_ADDRESS = 0x01;
    static const unsigned char HARDWARE_VERSION_ADDRESS = 0x02;
    static const unsigned char SOFTWARE_VERSION_ADDRESS = 0x03;
    static const unsigned char HALL_ERROR_ADDRESS = 0x04;
    static const unsigned char DEVICE_VOLTAGE_ADDRESS = 0x0A;
    static const unsigned char MOVING_RANGE_ADDRESS = 0x0B;
    static const unsigned char MOTOR_LOCK_STATE_ADDRESS = 0x11;
    static const unsigned char MOTOR_ANGLE_ADDRESS = 0x17;
    static const unsigned char MOTOR_SPEED_ADDRESS = 0x1D;
    static const unsigned char MOTOR_CURRENT_ADDRESS = 0x23;
    static const unsigned char PHASE_LOSS_FAULT_ADDRESS = 0x29;
    static const unsigned char CUR_SAMP_ERROR_ADDRESS = 0x2F;
    static const unsigned char JOINT_SKIN_ADDRESS = 0x41;
    
    // 6DOF相关地址定义
    static const unsigned char SET_ID_ADDRESS_6DOF = 0x00;
    static const unsigned char SET_BAUD_ADDRESS_6DOF = 0x01;
    static const unsigned char CLEAR_ERROR_ADDRESS_6DOF = 0x02;
    static const unsigned char SET_POWER_OFF_SAVE_ADDRESS_6DOF = 0x03;
    static const unsigned char FACTORY_DATA_RESET_ADDRESS_6DOF = 0x04;
    static const unsigned char SINGLE_MOTOR_CALIBRATION_ADDRESS_6DOF = 0x05;
    static const unsigned char CONTROL_JOINT_MOTOR_ABSOLUTE_ADDRESS_6DOF = 0x0B;
    static const unsigned char CONTROL_JOINT_MOTOR_RELATIVE_ADDRESS_6DOF = 0x11;
    static const unsigned char SET_MOTOR_STOP_ADDRESS_6DOF = 0x17;
    static const unsigned char SET_SPEED_ADDRESS_6DOF = 0x1D;
    static const unsigned char SET_CURRENT_ADDRESS_6DOF = 0x23;
    static const unsigned char ALL_MOTOR_CALIBRATION_ADDRESS_6DOF = 0x29;
    
    static const unsigned char HAND_TEST_ADDRESS_6DOF = 0x2A;                   //  整手测试
    static const unsigned char JOINT_LOCATION_ADDRESS_6DOF = 0x2B;              //  角度制位置
    static const unsigned char FINGER_CTRL_MODE_ADDRESS_6DOF = 0x31;            //  控制模式
    static const unsigned char PVT_CTRL_PARAM_ADDRESS_6DOF = 0x37;              //  PVT参数
    static const unsigned char PC_CTRL_PARAM_ADDRESS_6DOF = 0x49;               //  PC参数

    static const unsigned char INITIALIZE_DATA_ADDRESS_6DOF = 0x00;
    static const unsigned char BOOTLOADER_VERSION_ADDRESS_6DOF = 0x01;
    static const unsigned char HARDWARE_VERSION_ADDRESS_6DOF = 0x02;
    static const unsigned char SOFTWARE_VERSION_ADDRESS_6DOF = 0x03;
    static const unsigned char HALL_ERROR_ADDRESS_6DOF = 0x04;
    static const unsigned char DEVICE_VOLTAGE_ADDRESS_6DOF = 0x0A;
    static const unsigned char MOTOR_LOCK_STATE_ADDRESS_6DOF = 0x11;
    static const unsigned char MOTOR_ANGLE_ADDRESS_6DOF = 0x17;
    static const unsigned char MOTOR_SPEED_ADDRESS_6DOF = 0x1D;
    static const unsigned char MOTOR_CURRENT_ADDRESS_6DOF = 0x23;
    static const unsigned char PHASE_LOSS_FAULT_ADDRESS_6DOF = 0x29;
    static const unsigned char CUR_SAMP_ERROR_ADDRESS_6DOF = 0x2F;
    static const unsigned char MOTOR_RADIAN_ADDRESS_6DOF = 0x35;
    static const unsigned char JOINT_ABS_MAX_ADDRESS_6DOF = 0x3B;
    static const unsigned char CUST_NUMBER_ADDRESS_6DOF = 0x41;
    static const unsigned char ELE_SKIN_FORCE_INFOR_ADDRESS_6DOF = 0x42;
    static const unsigned char MOTOR_TEMPERATURE_ADDRESS_6DOF = 0x48;
    static const unsigned char HAND_TYPE_ADDRESS = 0x4E;        // 手类型读取地址

};

#endif // ZWHAND_H
