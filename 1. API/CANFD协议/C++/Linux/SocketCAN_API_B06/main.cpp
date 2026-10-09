#include "CANFDDriver.h"

#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <atomic>

namespace {

template <typename T>
void printVector(const std::string& title, const std::vector<T>& values) {
    std::cout << title << ": [";
    for (size_t i = 0; i < values.size(); ++i) {
        if (i > 0) {
            std::cout << ", ";
        }
        std::cout << values[i];
    }
    std::cout << "]" << std::endl;
}

struct TestResult {
    std::string name;
    bool ok;
};

std::vector<TestResult> g_testResults;

void printStepResult(const std::string& title, bool ok) {
    std::cout << title << ": " << (ok ? "OK" : "FAIL") << std::endl;
    g_testResults.push_back({title, ok});
}

void printSummary() {
    int passCount = 0;
    std::cout << "\n===== Summary =====" << std::endl;
    for (const auto& r : g_testResults) {
        std::cout << "  " << r.name << ": " << (r.ok ? "OK" : "FAIL") << std::endl;
        if (r.ok) ++passCount;
    }
    std::cout << "-----" << std::endl;
    std::cout << "Total: " << passCount << "/" << g_testResults.size()
              << " passed" << std::endl;
}
 
} // namespace

int main() {
    CANFDDriver hand("can0", 500000, 1000000, 0x01);

    if (!hand.connect()) {
        std::cout << "Connect failed!" << std::endl;
        return -1;
    }
    std::cout << "Connect success!" << std::endl;

    // 设备ID1
    // 波特率等级1
    // 故障清除1
    // 数据保存1
    // 恢复出厂设置1
    // 手指校准1
    // 绝对位置控制1
    // 增量位置控制1
    // 停机标志1
    // 最大转速设置1
    // 最大运行电流设置1
    // 整手所有关节零位全校准1
    // 整手测试1
    // 关节位置（角度制）控制1
    // 手指控制模式选择1
    // PVT模式下的位置-速度-时间控制参数(共18个数据)1
    // PC模式下的位置-电流控制参数1
    // 初始化完成标志1
    // boot版本号1
    // 硬件版本号1
    // 软件版本号1
    // 霍尔信号故障标志1
    // 系统电压1
    // 最大量程位置1
    // 堵转故障标志1
    // 当前位置1
    // 当前转速1
    // 当前电流1
    // 缺相故障1
    // 电流采样异常1
    // 当前关节位置（角度制）1
    // 最大关节位置（角度制）1
    // 客户编码1
    // 电子皮肤力数据1
    // 电机温度1
    // 手类型定义1

    // std::cout << "\n===== 6. Optional config tests =====" << std::endl;
    // printStepResult("Set device ID", hand.setId(1));
    // printStepResult("Set baud rate", hand.setBaud(1));
    // printStepResult("Power-off save", hand.setPowerOffSave(1));
    // printStepResult("Factory reset", hand.setFactoryDataReset());

    //先初始化
    std::cout << "\n===== 1. Hand initialization =====" << std::endl;
    std::cout << "Init state before: " << hand.getInitializeState() << std::endl;
    printStepResult("Full motor calibration", hand.setAllMotorCalibration());
    std::cout << "Waiting 20 seconds for initialization..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(22)); //初始化需要20s

    printStepResult("Finger control mode", hand.setFingerControlMode(std::vector<int16_t>{1,1,1,1,1,1}));
    //测试是否能设置电流
    // printStepResult("Single motor current", hand.setAllMotorCurrent({20,20,20,20,20,20}));
    // printStepResult("All motor angle", hand.setAllMotorAngle(std::vector<int16_t>{900, 900, 900, 900, 900, 900}));
    // printVector("Motor position", hand.getMotorRealPosition());
    // printVector("Motor speed", hand.getAllMotorSpeed());
    // printVector("Motor current", hand.getAllMotorCurrent());
    // printVector("Motor angle", hand.getAllMotorAngle());
    // std::this_thread::sleep_for(std::chrono::seconds(1));


    // printStepResult("Single motor current", hand.setAllMotorCurrent({50,50,50,50,50,50}));
    // printStepResult("All motor angle", hand.setAllMotorAngle(std::vector<int16_t>{900, 900, 900, 900, 900, 900}));
    // printVector("Motor speed", hand.getAllMotorSpeed());
    // printVector("Motor current", hand.getAllMotorCurrent());
    // printVector("Motor angle", hand.getAllMotorAngle());
    // std::this_thread::sleep_for(std::chrono::seconds(1));
    // printVector("Motor position", hand.getMotorRealPosition());
    // std::this_thread::sleep_for(std::chrono::seconds(1));


    hand.setAllMotorCurrent({50,50,50,50,50,50});
    hand.setAllMotorAngle(std::vector<int16_t>{900, 900, 900, 900, 900, 900});

    std::atomic<bool> stop_read{false};

    std::thread reader([&]() {
        while (!stop_read) {
            auto speeds = hand.getAllMotorSpeed();
            auto currents = hand.getAllMotorCurrent();
            auto angles = hand.getAllMotorAngle();
            auto positions = hand.getMotorRealPosition();
            printVector("Motor speed", speeds);
            printVector("Motor current", currents);
            printVector("Motor angle", angles);
            printVector("Motor position", positions);
            std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 读取间隔
        }
    });

    // 让电机运行一段时间，或等到你认为运动完成
    std::this_thread::sleep_for(std::chrono::seconds(3));

    stop_read = true;
    reader.join();   // 等待读取线程结束



    printStepResult("Single motor current", hand.setAllMotorCurrent({80,80,80,80,80,80}));
    printStepResult("All motor angle", hand.setAllMotorAngle(std::vector<int16_t>{300, 300, 300, 300, 300, 300}));
    printVector("Motor speed", hand.getAllMotorSpeed());
    printVector("Motor current", hand.getAllMotorCurrent());
    printVector("Motor angle", hand.getAllMotorAngle());
    std::this_thread::sleep_for(std::chrono::seconds(1));
    printVector("Motor position", hand.getMotorRealPosition());
    std::this_thread::sleep_for(std::chrono::seconds(3));

    //测试设置速度

    // printStepResult("Single motor speed", hand.setAllMotorSpeed({20,20,20,20,20,20}));
    // printStepResult("Single motor current", hand.setAllMotorCurrent({80,80,80,80,80,80}));
    // printStepResult("All motor angle", hand.setAllMotorAngle(std::vector<int16_t>{900, 900, 900, 900, 900, 900}));
    // printVector("Motor speed", hand.getAllMotorSpeed());
    // std::this_thread::sleep_for(std::chrono::seconds(1));

    printStepResult("Finger control mode", hand.setFingerControlMode(std::vector<int16_t>{1,1,1,1,1,1}));
    std::this_thread::sleep_for(std::chrono::seconds(1));

    std::cout << "Init state before: " << hand.getInitializeState() << std::endl;
    printStepResult("Single motor speed", hand.setAllMotorSpeed({50,50,50,50,50,50}));
    printStepResult("All motor angle", hand.setAllMotorAngle(std::vector<int16_t>{900, 900, 900, 900, 900, 900}));
    printVector("Motor speed", hand.getAllMotorSpeed());
    printVector("Motor current", hand.getAllMotorCurrent());
    printVector("Motor angle", hand.getAllMotorAngle());
    std::this_thread::sleep_for(std::chrono::seconds(1));
    printVector("Motor position", hand.getMotorRealPosition());
    std::this_thread::sleep_for(std::chrono::seconds(1));


    std::cout << "Init state before: " << hand.getInitializeState() << std::endl;
    printStepResult("Single motor speed", hand.setAllMotorSpeed({100,100,100,100,100,100}));
    printStepResult("All motor angle", hand.setAllMotorAngle(std::vector<int16_t>{300, 300, 300, 300, 300, 300}));
    printVector("Motor speed", hand.getAllMotorSpeed());
    printVector("Motor current", hand.getAllMotorCurrent());
    printVector("Motor angle", hand.getAllMotorAngle());
    std::this_thread::sleep_for(std::chrono::seconds(1));
    printVector("Motor position", hand.getMotorRealPosition());
    std::this_thread::sleep_for(std::chrono::seconds(1));


    std::cout << "\n===== 2. Basic setup =====" << std::endl;
    std::vector<int16_t> speed_100(6, 100);
    std::vector<int16_t> current_80(6, 80);
    printStepResult("Set all motor speed to 100", hand.setAllMotorSpeed(speed_100));
    printStepResult("Set all motor current to 80", hand.setAllMotorCurrent(current_80));
    printStepResult("Clear errors", hand.setErrorClear());

    std::cout << "\n===== 3. Readback =====" << std::endl;
    printStepResult("Finger control mode", hand.setFingerControlMode(std::vector<int16_t>{1,1,1,1,1,1}));
    // printStepResult("All motor angle", hand.setAllMotorAngle(std::vector<int16_t>{500, 500, 500, 500, 500, 500}));
    // printVector("Motor speed", hand.getAllMotorSpeed());
    std::this_thread::sleep_for(std::chrono::seconds(2));
    std::cout << "Bootloader version: " << hand.getBootloaderVersion() << std::endl;
    std::cout << "Hardware version: " << hand.getHardwareVersion() << std::endl;
    std::cout << "Software version: " << hand.getSoftwareVersion() << std::endl;
    printVector("Device error", hand.getDeviceError());
    std::cout << "System voltage(raw): " << hand.getDeviceVoltage() << std::endl;
    printVector("Motor locked state", hand.getMotorLockedState());
    printVector("Motor position", hand.getMotorRealPosition());
    printVector("Motor current", hand.getAllMotorCurrent());
    printVector("Phase loss fault", hand.getPhaseLossFault());
    printVector("Current sample error", hand.getCurSampFault());
    printVector("Motor angle", hand.getAllMotorAngle());
    printVector("Abs max position", hand.getAbsMaxPosition());
    printVector("Joint abs max", hand.getJointAbsMax());
    std::cout << "Customer number: " << hand.getCustomerNumber() << std::endl;
    printVector("Skin force", hand.getSkinForce());
    printVector("Motor temperature", hand.getMotorTemperature());
    std::cout << "Hand type: " << hand.getHandType() << std::endl;

    std::cout << "\n===== 4. Write tests =====" << std::endl;
    // printStepResult("Single motor stop", hand.setSingleMotorStop(1));
    printStepResult("Finger control mode", hand.setFingerControlMode(std::vector<int16_t>{1,1,1,1,1,1}));
    std::this_thread::sleep_for(std::chrono::seconds(1));
    // printStepResult("All motor stop", hand.setAllMotorStop(std::vector<int16_t>{0, 0, 0, 0, 0, 0}));
    // std::this_thread::sleep_for(std::chrono::seconds(2));
    printStepResult("All motor angle", hand.setAllMotorAngle(std::vector<int16_t>{500, 500, 500, 500, 500, 500}));
    std::this_thread::sleep_for(std::chrono::seconds(2));
    printStepResult("Single motor absolute", hand.setSingleMotorAbsolute(1, 300));
    std::this_thread::sleep_for(std::chrono::seconds(2));
    printStepResult("All motor absolute", hand.setAllMotorAbsolute(std::vector<int16_t>{300, 300, 300, 300, 300, 300}));
    std::this_thread::sleep_for(std::chrono::seconds(2));
    printStepResult("Single motor relative", hand.setSingleMotorRelative(1, -100));
    std::this_thread::sleep_for(std::chrono::seconds(1));
    printStepResult("Single motor relative", hand.setSingleMotorRelative(2, -100));
    std::this_thread::sleep_for(std::chrono::seconds(1));
    printStepResult("Single motor relative", hand.setSingleMotorRelative(3, -100));
    std::this_thread::sleep_for(std::chrono::seconds(1));
    printStepResult("Single motor relative", hand.setSingleMotorRelative(4, -100));
    std::this_thread::sleep_for(std::chrono::seconds(1));
    printStepResult("Single motor relative", hand.setSingleMotorRelative(5, -100));
    std::this_thread::sleep_for(std::chrono::seconds(1));
    printStepResult("Single motor relative", hand.setSingleMotorRelative(6, -100));
    std::this_thread::sleep_for(std::chrono::seconds(2));
    printStepResult("All motor relative", hand.setAllMotorRelative(std::vector<int16_t>{200, 200, 200, 200, 200, 200}));
    std::this_thread::sleep_for(std::chrono::seconds(2));
    printStepResult("Finger control mode", hand.setFingerControlMode(std::vector<int16_t>{0,0,0,0,0,0}));
    std::this_thread::sleep_for(std::chrono::seconds(1));
    printStepResult("PC params", hand.setPcControlParam(std::vector<int16_t>{5000, -5000, 5000, -5000, -5000, -5000}));
    std::this_thread::sleep_for(std::chrono::seconds(2));
    printStepResult("Finger control mode", hand.setFingerControlMode(std::vector<int16_t>{2,2,2,2,2,2}));
    std::this_thread::sleep_for(std::chrono::seconds(1));
    printStepResult("PVT params", hand.setPvtControlParam(std::vector<uint16_t>{10, 0, 1000,
                                                                                10, 0, 1000,
                                                                                10, 0, 1000,
                                                                                1000, 0, 1000,
                                                                                1000, 0, 1000,
                                                                                1000, 0, 1000}
                                                                                ));
    std::this_thread::sleep_for(std::chrono::seconds(2));
    printStepResult("PVT params", hand.setPvtControlParam(std::vector<uint16_t>{10, 0, 1000,
                                                                                10, 0, 1000,
                                                                                10, 0, 1000,
                                                                                10, 0, 1000,
                                                                                10, 0, 1000,
                                                                                10, 0, 1000}
                                                                                ));
    std::this_thread::sleep_for(std::chrono::seconds(2));
    printStepResult("Hand test", hand.setHandTest(1));
    std::this_thread::sleep_for(std::chrono::seconds(15));
    printStepResult("Hand test", hand.setHandTest(0));
    // printStepResult("Single motor stop", hand.setAllMotorStop{1,1,1,1,1,1,1});

    std::cout << "\nTest finished" << std::endl;
    printSummary();
    return 0;
}
