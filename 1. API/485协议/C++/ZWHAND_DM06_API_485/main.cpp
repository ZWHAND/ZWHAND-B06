#include "ZWHAND.h"
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    ZWHAND hand(1, "COM1", 115200);
    bool open_com_state = hand.open_zwhand();
    
    if (open_com_state) {
        std::cout << "串口打开成功" << std::endl;
        short initialize_state = hand.get_initialize_state();
        
        if (initialize_state != -1) {
            std::cout << "Device initialization successful!" << std::endl;
            bool set_ret = hand.set_all_motor_calibration_6dof();
            
            if (set_ret) {
                std::cout << "Calibration successful!" << std::endl;
            } else {
                std::cout << "Calibration failed!" << std::endl;
            }
            
            Sleep(20000); // 20秒等待校准完成
            
            std::vector<int> angles = {0, 1000, 1000, 1000, 1000, 1000};
            set_ret = hand.set_all_motor_absolute_6dof(angles);
            
            if (set_ret) {
                std::cout << "设置所有电机绝对角度成功" << std::endl;
            } else {
                std::cout << "设置所有电机绝对角度失败" << std::endl;
            }
            
            // 测试读取数据
            std::vector<short> motor_angles = hand.get_motor_real_angle();
            if (!motor_angles.empty()) {
                std::cout << "电机实际角度: ";
                for (size_t i = 0; i < motor_angles.size(); ++i) {
                    std::cout << motor_angles[i];
                    if (i < motor_angles.size() - 1) std::cout << ", ";
                }
                std::cout << std::endl;
            }
            
            std::vector<short> motor_speeds = hand.get_all_motor_speed();
            if (!motor_speeds.empty()) {
                std::cout << "电机实际速度: ";
                for (size_t i = 0; i < motor_speeds.size(); ++i) {
                    std::cout << motor_speeds[i];
                    if (i < motor_speeds.size() - 1) std::cout << ", ";
                }
                std::cout << std::endl;
            }
            
            std::vector<short> motor_currents = hand.get_all_motor_current();
            if (!motor_currents.empty()) {
                std::cout << "电机实际电流: ";
                for (size_t i = 0; i < motor_currents.size(); ++i) {
                    std::cout << motor_currents[i];
                    if (i < motor_currents.size() - 1) std::cout << ", ";
                }
                std::cout << std::endl;
            }
            
            std::vector<short> phase_loss_faults = hand.get_phase_loss_fault();
            if (!phase_loss_faults.empty()) {
                std::cout << "电机相位丢失状态: ";
                for (size_t i = 0; i < phase_loss_faults.size(); ++i) {
                    std::cout << phase_loss_faults[i];
                    if (i < phase_loss_faults.size() - 1) std::cout << ", ";
                }
                std::cout << std::endl;
            }
            
            std::vector<short> cur_samp_faults = hand.get_cur_samp_fault();
            if (!cur_samp_faults.empty()) {
                std::cout << "电机电流采样错误状态: ";
                for (size_t i = 0; i < cur_samp_faults.size(); ++i) {
                    std::cout << cur_samp_faults[i];
                    if (i < cur_samp_faults.size() - 1) std::cout << ", ";
                }
                std::cout << std::endl;
            }
        }
    } else {
        std::cout << "串口打开失败" << std::endl;
    }
    
    Sleep(5000);
    return 0;
}
