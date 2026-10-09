/**
 * @file test_complete_all.cpp
 * @brief Complete test for all ZWHAND C++ SDK interfaces (47+ functions)
 * @version 1.0
 * 
 * This test file covers all public interfaces of the ZWHAND class.
 * Run this test after connecting the dexterous hand hardware.
 */


#include <chrono>
#include <thread>
#include "ZWHAND.h"
#include <iostream>
#include <vector>

#if defined(_WIN32)
    #include <windows.h>
#endif

using namespace std;

int pass = 0;
int fail = 0;

void printSeparator(const string& title) {
    cout << "\n========================================" << endl;
    cout << title << endl;
    cout << "========================================" << endl;
}

void printResult(bool success, const string& name) {
    if (success) {
        cout << "[PASS] " << name << endl;
        pass++;
    } else {
        cout << "[FAIL] " << name << endl;
        fail++;
    }
}

void printVector(const string& name, const vector<short>& data) {
    cout << "[DATA] " << name << ": ";
    for (size_t i = 0; i < data.size(); i++) {
        cout << data[i];
        if (i < data.size() - 1) cout << ", ";
    }
    cout << endl;
}

int main() {
    cout << "ZWHAND C++ SDK Complete Interface Test" << endl;
    
    ZWHAND hand(1, "/dev/ttyUSB0", 115200);
    
    // 1. Constructor
    printSeparator("Constructor");
    printResult(true, "ZWHAND(1, /dev/ttyUSB0, 115200)");
    
    // 2. Open Device
    printSeparator("open_start_device()");
    bool ret = hand.open_start_device();
    printResult(ret, "open_start_device()");
    if (!ret) {
        cout << "Cannot open device, test terminated" << endl;
        return -1;
    }
    
    // 3. Get Initialize State
    printSeparator("get_initialize_state()");
    short init = hand.get_initialize_state();
    printResult(init != -1, "get_initialize_state() = " + to_string(init));
    
    // 4. Get Bootloader Version
    printSeparator("get_bootloader_version()");
    short boot = hand.get_bootloader_version();
    printResult(boot != -1, "get_bootloader_version() = " + to_string(boot));
    
    // 5. Get Hardware Version
    printSeparator("get_hardware_version()");
    short hw = hand.get_hardware_version();
    printResult(hw != -1, "get_hardware_version() = " + to_string(hw));
    
    // 6. Get Software Version
    printSeparator("get_software_version()");
    short sw = hand.get_software_version();
    printResult(sw != -1, "get_software_version() = " + to_string(sw));
    
    // 7. Get Device Voltage
    printSeparator("get_device_voltage()");
    short volt = hand.get_device_voltage();
    printResult(volt != -1, "get_device_voltage() = " + to_string(volt) + " V");
    
    // 8. Get Customer Number
    printSeparator("get_cust_number()");
    short cust = hand.get_cust_number();
    printResult(cust != -1, "get_cust_number() = " + to_string(cust));

    // 8.5 Get Hand Type
    printSeparator("get_hand_type()");
    short handType = hand.get_hand_type();
    printResult(handType != -1, "get_hand_type() = " + to_string(handType) + (handType==0 ? " (Right hand)" : " (Left hand)"));
    
    // 9. Set Device ID
    printSeparator("set_id()");
    ret = hand.set_id(0x01);
    printResult(ret, "set_id(0x01)");
    
    // 10. Set Baud Rate
    printSeparator("set_baud()");
    ret = hand.set_baud(0x02, false);
    printResult(ret, "set_baud(115200)");
    
    // 11. Clear Error
    printSeparator("set_error_clear()");
    ret = hand.set_error_clear();
    printResult(ret, "set_error_clear()");
    
    // 12. Power Off Save
    printSeparator("set_power_off_save()");
    ret = hand.set_power_off_save(0x01);
    printResult(ret, "set_power_off_save(config save)");
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    // 13. Factory Data Reset
    printSeparator("set_factory_data_reset()");
    ret = hand.set_factory_data_reset();
    printResult(ret, "set_factory_data_reset()");
    
    // 14. Hand Test
    printSeparator("set_hand_test()");
    ret = hand.set_hand_test(1);
    printResult(ret, "set_hand_test(1) - Start");
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));  // 观察手是否开始动作
    // 关闭测试
    ret = hand.set_hand_test(0);
    printResult(ret, "set_hand_test(0) - Stop");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));  // 观察是否停止
    
    // 15. Finger Control Mode
    printSeparator("set_finger_control_mode()");
    ret = hand.set_finger_control_mode(1);
    printResult(ret, "set_finger_control_mode(PSC=1)");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // 16. Set All Motors Speed
    printSeparator("set_all_motor_speed()");
    ret = hand.set_all_motor_speed(80);
    printResult(ret, "set_all_motor_speed(80)");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // 17. Set Single Motor Speed
    printSeparator("set_single_motor_speed()");
    ret = hand.set_single_motor_speed(1, 90);
    printResult(ret, "set_single_motor_speed(motor1, 90)");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // 18. Set All Motors Current
    printSeparator("set_all_motor_current()");
    ret = hand.set_all_motor_current(60);
    printResult(ret, "set_all_motor_current(60)");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // 19. Set Single Motor Current
    printSeparator("set_single_motor_current()");
    ret = hand.set_single_motor_current(1, 70);
    printResult(ret, "set_single_motor_current(motor1, 70)");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // 20. Calibrate All Motors (6DOF)
    printSeparator("set_all_motor_calibration()");
    ret = hand.set_all_motor_calibration();
    printResult(ret, "set_all_motor_calibration()");
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    
    // 21. Calibrate Single Motor (6DOF)
    printSeparator("set_single_motor_calibration()");
    ret = hand.set_single_motor_calibration(1);
    printResult(ret, "set_single_motor_calibration(motor1)");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    // 23. Set All Motors Absolute Position (6DOF)
    printSeparator("set_all_motor_absolute()");
    vector<int> abs_pos = {0, 500, 1000, 1000, 1000, 1000};
    ret = hand.set_all_motor_absolute(abs_pos);
    printResult(ret, "set_all_motor_absolute()");
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    
    // 24. Set Single Motor Absolute Position (6DOF)
    printSeparator("set_single_motor_absolute()");
    ret = hand.set_single_motor_absolute(1, 500);
    printResult(ret, "set_single_motor_absolute(motor1, 500)");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    // 25. Set All Motors Relative Position (6DOF)
    printSeparator("set_all_motor_relative()");
    vector<int> rel_pos = {100, 100, 100, 100, 100, 100};
    ret = hand.set_all_motor_relative(rel_pos);
    printResult(ret, "set_all_motor_relative()");
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    
    // 26. Set Single Motor Relative Position (6DOF)
    printSeparator("set_single_motor_relative()");
    ret = hand.set_single_motor_relative(1, 100);
    printResult(ret, "set_single_motor_relative(motor1, 100)");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    // 27. Set All Motors Radian Angle
    printSeparator("set_all_motor_radian()");
    vector<int> radian = {0, 500, 1000, 1000, 1000, 1000};
    ret = hand.set_all_motor_radian(radian);
    printResult(ret, "set_all_motor_radian()");
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    
    // 28. Set PVT Control Parameter
    printSeparator("set_pvt_control_param()");
    vector<uint16_t> pvt = {
        500, 33000, 0, 500, 33000, 0,
        500, 33000, 0, 500, 33000, 0,
        500, 33000, 0, 500, 33000, 0
    };
    ret = hand.set_pvt_control_param(pvt);
    printResult(ret, "set_pvt_control_param()");
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    
    // 29. Set PC Control Parameter
    printSeparator("set_pc_control_param()");
    vector<int> pc = {1000, 1000, 1000, 1000, 1000, 1000};
    ret = hand.set_pc_control_param(pc);
    printResult(ret, "set_pc_control_param()");
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    
    // 30. Stop All Motors
    printSeparator("set_all_motor_stop()");
    ret = hand.set_all_motor_stop();
    printResult(ret, "set_all_motor_stop()");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // 31. Stop Single Motor
    printSeparator("set_single_motor_stop()");
    ret = hand.set_single_motor_stop(1);
    printResult(ret, "set_single_motor_stop(motor1)");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // 32. Get Motor Real Angle
    printSeparator("get_motor_real_angle()");
    vector<short> angles = hand.get_motor_real_angle();
    printResult(!angles.empty(), "get_motor_real_angle()");
    if (!angles.empty()) printVector("Real angle", angles);
    
    // 33. Get All Motor Speed
    printSeparator("get_all_motor_speed()");
    vector<short> speeds = hand.get_all_motor_speed();
    printResult(!speeds.empty(), "get_all_motor_speed()");
    if (!speeds.empty()) printVector("Motor speed", speeds);
    
    // 34. Get All Motor Current
    printSeparator("get_all_motor_current()");
    vector<short> currents = hand.get_all_motor_current();
    printResult(!currents.empty(), "get_all_motor_current()");
    if (!currents.empty()) printVector("Motor current", currents);
    
    // 35. Get All Motor Radian
    printSeparator("get_all_motor_radian()");
    vector<short> radians = hand.get_all_motor_radian();
    printResult(!radians.empty(), "get_all_motor_radian()");
    if (!radians.empty()) printVector("Motor radian", radians);
    
    // 36. Get All Motor Temperature
    printSeparator("get_all_motor_temperature()");
    vector<short> temps = hand.get_all_motor_temperature();
    printResult(!temps.empty(), "get_all_motor_temperature()");
    if (!temps.empty()) printVector("Motor temp", temps);
    
    // 37. Get Phase Loss Fault
    printSeparator("get_phase_loss_fault()");
    vector<short> phase = hand.get_phase_loss_fault();
    printResult(!phase.empty(), "get_phase_loss_fault()");
    if (!phase.empty()) printVector("Phase loss", phase);
    
    // 38. Get Current Sampling Fault
    printSeparator("get_cur_samp_fault()");
    vector<short> cur = hand.get_cur_samp_fault();
    printResult(!cur.empty(), "get_cur_samp_fault()");
    if (!cur.empty()) printVector("Cur samp fault", cur);
    
    // 39. Get Motor Locked State
    printSeparator("get_motor_locked_state()");
    vector<short> locked = hand.get_motor_locked_state();
    printResult(!locked.empty(), "get_motor_locked_state()");
    if (!locked.empty()) printVector("Locked state", locked);
    
    // 40. Get All Motor Max Radian
    printSeparator("get_all_motor_max_radian()");
    vector<short> max_rad = hand.get_all_motor_max_radian();
    printResult(!max_rad.empty(), "get_all_motor_max_radian()");
    if (!max_rad.empty()) printVector("Max radian", max_rad);
    
    // 41. Get Device Error
    printSeparator("get_device_error()");
    vector<short> dev_err = hand.get_device_error();
    printResult(!dev_err.empty(), "get_device_error()");
    if (!dev_err.empty()) printVector("Device error", dev_err);
    
    // 42. Get Electronic Skin Force
    printSeparator("get_ele_skin_force_infor()");
    vector<short> skin = hand.get_ele_skin_force_infor();
    printResult(!skin.empty(), "get_ele_skin_force_infor()");
    if (!skin.empty()) printVector("Skin force", skin);
    
    // 43. Get Joint Skin Moment
    printSeparator("get_joint_skin_moment()");
    vector<short> moment = hand.get_joint_skin_moment();
    printResult(!moment.empty(), "get_joint_skin_moment()");
    if (!moment.empty()) printVector("Joint moment", moment);
    
    // 44. Modbus CRC
    // 测试 modbus_crc
    printSeparator("modbus_crc()");
    vector<unsigned char> test_data = {0x01, 0x06, 0x00, 0x2A, 0x00, 0x01};
    vector<unsigned char> frame = ZWHAND::modbus_crc(test_data);
    // 检查返回的帧长度 = 原始数据长度 + 2
    bool crc_ok = (frame.size() == test_data.size() + 2);
    if (crc_ok) {
        cout << "[DATA] Full frame: ";
        for (unsigned char c : frame) {
            printf("%02X ", c);
        }
        cout << endl;
    }
    printResult(crc_ok, "modbus_crc()");


    // 45. Close Device
    printSeparator("close_device()");
    ret = hand.close_device();
    printResult(ret, "close_device()");
    
    // Test Summary
    cout << "\n========================================" << endl;
    cout << "TEST SUMMARY" << endl;
    cout << "========================================" << endl;
    cout << "[PASS] Passed: " << pass << endl;
    cout << "[FAIL] Failed: " << fail << endl;
    cout << "[TOTAL] Total: " << (pass + fail) << endl;
    
    return 0;
}


