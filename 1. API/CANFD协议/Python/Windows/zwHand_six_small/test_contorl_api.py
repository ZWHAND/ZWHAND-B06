import time
from B06_ZWHAND_CANFD_API import ZWHAND


def print_section(title):
    """打印测试章节标题"""
    print(f"\n{'=' * 60}")
    print(f"测试：{title}")
    print(f"{'=' * 60}")


class TestZWHAND:
    """ZWHAND 灵巧手控制接口测试类"""

    def __init__(self, lqs_id=0x01):
        """初始化测试环境"""
        self.hand = ZWHAND(lqs_id)
        self.test_results = {}
        self.test_count = 0
        self.pass_count = 0
        self.fail_count = 0
        self.current_section = ""

    def record_result(self, test_name, result, data=None):
        """记录测试结果"""
        self.test_count += 1
        if result:
            self.pass_count += 1
        else:
            self.fail_count += 1

        if self.current_section not in self.test_results:
            self.test_results[self.current_section] = []

        self.test_results[self.current_section].append({
            'test_name': test_name,
            'result': result,
            'data': data
        })

    def print_result(self, test_name, result, data=None):
        """打印测试结果"""
        status = "✓ 成功" if result else "✗ 失败"
        print(f"[{status}] {test_name}")
        if data is not None:
            print(f"    返回数据：{data}")

        # 记录测试结果
        self.record_result(test_name, result, data)

        return result

    def run_all_tests(self):
        """运行所有测试"""
        print("\n" + "=" * 60)
        print("ZWHAND 灵巧手控制接口完整测试")
        print("=" * 60)

        # 1. 基础配置测试
        self.current_section = "1. 基础配置测试"
        self.test_basic_configuration()

        # 2. 错误管理测试
        self.current_section = "2. 错误管理测试"
        self.test_error_management()

        # 3. 电机校准测试
        self.current_section = "3. 电机校准测试"
        self.test_motor_calibration()

        # 4. 电机速度控制测试
        self.current_section = "4. 电机速度控制测试"
        self.test_motor_speed_control()

        # 5. 电机电流控制测试
        self.current_section = "5. 电机电流控制测试"
        self.test_motor_current_control()

        # 6. 电机停止控制测试
        # self.current_section = "6. 电机停止控制测试"
        # self.test_motor_stop_control()

        # 7. 电机位置控制测试
        self.current_section = "7. 电机位置控制测试"
        self.test_motor_position_control()

        # 8. 电机弧度控制测试
        self.current_section = "8. 电机弧度控制测试"
        self.test_motor_radian_control()

        # 9. 高级控制模式测试
        self.current_section = "9. 高级控制模式测试"
        self.test_advanced_control_mode()

        # 10. 设备状态读取测试
        self.current_section = "10. 设备状态读取测试"
        self.test_device_status_read()

        # 11. 电机实时数据读取测试
        self.current_section = "11. 电机实时数据读取测试"
        self.test_motor_realtime_data()

        # 12. 其他信息读取测试
        self.current_section = "12. 其他信息读取测试"
        self.test_other_information()

        # 打印测试总结
        self.print_summary()

    def test_basic_configuration(self):
        """1. 基础配置测试"""
        print_section("1. 基础配置测试")
        self.hand.open_start_device()

        # 测试设备 ID 设置 (谨慎使用，会改变设备 ID)
        # print("\n注意：以下测试会改变设备 ID，请确认是否需要测试")
        # new_id = 0x02
        # ret = self.hand.set_id(new_id)
        # self.print_result(f"设置设备 ID 为 {new_id}", ret)

        # 测试波特率设置 (谨慎使用)
        # print("\n注意：以下测试会改变波特率，请确认是否需要测试")
        # ret = self.hand.set_baud(2)  # 设置为 115200
        # self.print_result("设置波特率为 115200", ret)

    def test_error_management(self):
        """2. 错误管理测试"""
        print_section("2. 错误管理测试")

        # 清除错误
        ret = self.hand.set_error_clear()
        self.print_result("清除设备错误", ret)

        # 掉电保存设置
        ret = self.hand.set_power_off_save(1)
        self.print_result("设置掉电保存 - 配置保存", ret)
        time.sleep(0.5)

        ret = self.hand.set_power_off_save(2)
        self.print_result("设置掉电保存 - 参数保存", ret)
        time.sleep(0.5)

        # 恢复出厂设置 (谨慎使用)
        # print("\n警告：以下测试将恢复出厂设置!")
        ret = self.hand.set_factory_data_reset()
        self.print_result("恢复出厂设置", ret)

    def test_motor_speed_control(self):
        """3. 电机速度控制测试"""
        print_section("3. 电机速度控制测试")

        # 单个电机速度设置
        ret = self.hand.set_single_motor_speed(1, 50)
        self.print_result("设置 1 号电机速度为 50", ret)

        ret = self.hand.set_single_motor_speed(3, 75)
        self.print_result("设置 3 号电机速度为 75", ret)

        # 所有电机速度设置
        speed_list = [50, 60, 70, 80, 90, 100]
        ret = self.hand.set_all_motor_speed(speed_list)
        self.print_result(f"设置所有电机速度为 {speed_list}", ret)

        # 整数形式设置所有电机速度
        ret = self.hand.set_all_motor_speed(100)
        self.print_result("设置所有电机速度为 100(统一档位)", ret)

        # 读取所有电机速度
        speeds = self.hand.get_all_motor_speed()
        self.print_result("读取所有电机实际速度", speeds is not False, speeds)

    def test_motor_current_control(self):
        """4. 电机电流控制测试"""
        print_section("4. 电机电流控制测试")

        # 单个电机电流设置
        ret = self.hand.set_single_motor_current(1, 50)
        self.print_result("设置 1 号电机电流为 50", ret)

        ret = self.hand.set_single_motor_current(2, 60)
        self.print_result("设置 2 号电机电流为 60", ret)

        # 所有电机电流设置
        current_list = [40, 50, 60, 70, 80, 90]
        ret = self.hand.set_all_motor_current(current_list)
        self.print_result(f"设置所有电机电流为 {current_list}", ret)

        # 整数形式设置所有电机电流
        ret = self.hand.set_all_motor_current(50)
        self.print_result("设置所有电机电流为 50(统一档位)", ret)

        # 读取所有电机电流
        currents = self.hand.get_all_motor_current()
        self.print_result("读取所有电机实际电流", currents is not False, currents)

    def test_motor_stop_control(self):
        """5. 电机停止控制测试"""
        print_section("5. 电机停止控制测试")

        # 单个电机停止
        ret = self.hand.set_single_motor_stop(1)
        self.print_result("停止 1 号电机", ret)

        ret = self.hand.set_single_motor_stop(3)
        self.print_result("停止 3 号电机", ret)

        # 所有电机停止
        ret = self.hand.set_all_motor_stop()
        self.print_result("停止所有电机 (默认全部停止)", ret)

        # 自定义停止标志
        stop_flags = [1, 0, 1, 0, 0, 0]
        ret = self.hand.set_all_motor_stop(stop_flags)
        self.print_result(f"停止指定电机 {stop_flags}", ret)

    def test_motor_position_control(self):
        """6. 电机位置控制测试"""
        print_section("6. 电机位置控制测试")

        # 单个电机绝对位置
        ret = self.hand.set_single_motor_absolute(1, 500)
        self.print_result("设置 1 号电机绝对位置为 500", ret)

        ret = self.hand.set_single_motor_absolute(2, 750)
        self.print_result("设置 2 号电机绝对位置为 750", ret)

        # 所有电机绝对位置
        angle_list = [0, 200, 400, 600, 800, 1000]
        ret = self.hand.set_all_motor_absolute(angle_list)
        self.print_result(f"设置所有电机绝对位置为 {angle_list}", ret)

        time.sleep(2)  # 等待电机运动到位

        # 单个电机相对位置
        ret = self.hand.set_single_motor_relative(1, 100)
        self.print_result("设置 1 号电机相对位置为 +100", ret)

        ret = self.hand.set_single_motor_relative(2, -100)
        self.print_result("设置 2 号电机相对位置为 -100", ret)

        # 所有电机相对位置
        relative_angles = [50, -50, 100, -100, 150, -150]
        ret = self.hand.set_all_motor_relative(relative_angles)
        self.print_result(f"设置所有电机相对位置为 {relative_angles}", ret)

        time.sleep(2)

        # 读取电机实际角度
        angles = self.hand.get_motor_real_angle()
        self.print_result("读取所有电机实际角度", angles is not False, angles)

    def test_motor_calibration(self):
        """7. 电机校准测试"""
        print_section("7. 电机校准测试")

        # 单个电机校准
        ret = self.hand.set_single_motor_calibration(1)
        self.print_result("校准 1 号电机零位", ret)
        time.sleep(2)

        ret = self.hand.set_single_motor_calibration(3)
        self.print_result("校准 3 号电机零位", ret)
        time.sleep(2)

        # 所有电机校准
        print("\n注意：整手校准需要约 20 秒时间...")
        ret = self.hand.set_all_motor_calibration()
        self.print_result("校准所有电机零位 (整手)", ret)

        if ret:
            print("等待校准完成...")
            time.sleep(20)
            print("校准完成!")

    def test_motor_radian_control(self):
        """8. 电机弧度控制测试"""
        print_section("8. 电机弧度控制测试")

        # 设置所有电机弧度 (系数 0.1，范围 0-3600)
        radian_list = [0, 360, 720, 1080, 1440, 1800]
        ret = self.hand.set_all_motor_radian(radian_list)
        self.print_result(f"设置所有电机弧度为 {radian_list}", ret)

        time.sleep(2)

        # 读取所有电机弧度
        radians = self.hand.get_all_motor_radian()
        self.print_result("读取所有电机实际弧度", radians is not False, radians)

        # 读取所有电机最大弧度
        max_radians = self.hand.get_all_motor_max_radian()
        self.print_result("读取所有电机最大弧度", max_radians is not False, max_radians)

    def test_advanced_control_mode(self):
        """9. 高级控制模式测试"""
        print_section("9. 高级控制模式测试")

        # 设置手指控制模式
        # 0:PC 模式，1:PSC 模式，2:PVT 模式
        mode_list = [0, 0, 0, 0, 0, 0]
        ret = self.hand.set_finger_control_mode(mode_list)
        self.print_result(f"设置手指控制模式为 PC 模式 {mode_list}", ret)

        # 统一设置
        ret = self.hand.set_finger_control_mode(0)
        self.print_result("设置所有手指为 PC 模式 (统一设置)", ret)

        # 设置 PVT 控制参数 (18 个参数，6 个电机*3 个参数)
        # P、V、T 参数，V 与 T 只能一个非 0
        pvt_params = [500, 30000, 0] * 6  # PV 模式示例
        ret = self.hand.set_pvt_control_param(pvt_params)
        self.print_result(f"设置 PVT 控制参数 (PV 模式)", ret)

        pvt_params = [500, 0, 1000] * 6  # PT 模式示例
        ret = self.hand.set_pvt_control_param(pvt_params)
        self.print_result(f"设置 PVT 控制参数 (PT 模式)", ret)

        # 设置 PC 控制参数 (范围 -8000 到 8000)
        pc_params = [1000, 2000, 3000, 4000, 5000, 6000]
        ret = self.hand.set_pc_control_param(pc_params)
        self.print_result(f"设置 PC 控制参数为 {pc_params}", ret)

        pc_params = [-1000, -2000, 0, 2000, 4000, 6000]
        ret = self.hand.set_pc_control_param(pc_params)
        self.print_result(f"设置 PC 控制参数为 {pc_params}(含负值)", ret)

    def test_device_status_read(self):
        """10. 设备状态读取测试"""
        print_section("10. 设备状态读取测试")

        # 初始化状态
        init_state = self.hand.get_initialize_state()
        self.print_result("获取设备初始化状态", init_state is not False, f"初始化状态={init_state}")

        # Bootloader 版本
        boot_ver = self.hand.get_bootloader_version()
        self.print_result("获取 Bootloader 版本", boot_ver is not False, f"版本={boot_ver}")

        # 硬件版本
        hw_ver = self.hand.get_hardware_version()
        self.print_result("获取硬件版本", hw_ver is not False, f"版本={hw_ver}")

        # 软件版本
        sw_ver = self.hand.get_software_version()
        self.print_result("获取软件版本", sw_ver is not False, f"版本={sw_ver}")

        # 设备错误码
        errors = self.hand.get_motor_hall_fault()
        self.print_result("获取电机hall故障状态", errors is not False, errors)

        # 设备电压
        voltage = self.hand.get_device_voltage()
        self.print_result("获取设备电压", voltage is not False,
                     f"电压值={voltage} (实际电压={voltage * 0.001 if voltage else 'N/A'}V)")

        # 电机堵转状态
        locked_states = self.hand.get_motor_locked_state()
        self.print_result("获取电机堵转状态", locked_states is not False, locked_states)

    def test_motor_realtime_data(self):
        """11. 电机实时数据读取测试"""
        print_section("11. 电机实时数据读取测试")

        # 电机实际角度
        angles = self.hand.get_motor_real_angle()
        self.print_result("读取电机实际角度挡位", angles is not False, angles)

        # 电机实际速度
        speeds = self.hand.get_all_motor_speed()
        self.print_result("读取电机实际速度", speeds is not False, speeds)

        # 电机实际电流
        currents = self.hand.get_all_motor_current()
        self.print_result("读取电机实际电流", currents is not False, currents)

        # 相位丢失故障
        phase_faults = self.hand.get_phase_loss_fault()
        self.print_result("读取电机相位丢失状态", phase_faults is not False, phase_faults)

        # 电流采样错误
        samp_errors = self.hand.get_cur_samp_fault()
        self.print_result("读取电流采样错误状态", samp_errors is not False, samp_errors)

        # 电机实际弧度
        radians = self.hand.get_all_motor_radian()
        self.print_result("读取电机实际弧度", radians is not False, radians)

        # 电机温度
        temperatures = self.hand.get_all_motor_temperature()
        self.print_result("读取电机实际温度", temperatures is not False,
                     f"{temperatures} (实际温度={[t * 0.1 for t in temperatures] if temperatures else 'N/A'}°C)")

    def test_other_information(self):
        """12. 其他信息读取测试"""
        print_section("12. 其他信息读取测试")

        # 客户编号
        cust_num = self.hand.get_cust_number()
        self.print_result("获取客户编号", cust_num is not False, cust_num)

        # 电子皮肤力信息
        skin_force = self.hand.get_ele_skin_force_infor()
        self.print_result("获取电子皮肤力信息", skin_force is not False,
                     f"{skin_force} (实际力值={[f * 0.01 for f in skin_force] if skin_force else 'N/A'}N)")

    def print_summary(self):
        """打印测试总结"""
        print("\n" + "=" * 80)
        print("测试结果汇总报告")
        print("=" * 80)

        # 总体统计
        print(f"\n总体统计:")
        print(f"  总测试数: {self.test_count}")
        print(f"  通过数量: {self.pass_count} ✓")
        print(f"  失败数量: {self.fail_count} ✗")
        print(f"  通过率:   {(self.pass_count / self.test_count * 100) if self.test_count > 0 else 0:.2f}%")

        # 各章节详细结果
        print(f"\n{'-' * 80}")
        print("各章节详细结果:")
        print(f"{'-' * 80}")

        for section, results in self.test_results.items():
            section_pass = sum(1 for r in results if r['result'])
            section_fail = len(results) - section_pass
            section_total = len(results)

            print(f"\n{section}:")
            print(f"  测试数: {section_total}, 通过: {section_pass} ✓, 失败: {section_fail} ✗")

            # 显示每个测试项的结果
            for i, result in enumerate(results, 1):
                status = "✓" if result['result'] else "✗"
                data_str = f" -> {result['data']}" if result['data'] is not None else ""
                print(f"    {i}. [{status}] {result['test_name']}{data_str}")

        # 失败项汇总（如果有）
        print(f"\n{'-' * 80}")
        if self.fail_count > 0:
            print("失败测试项汇总:")
            print(f"{'-' * 80}")
            fail_index = 1
            for section, results in self.test_results.items():
                for result in results:
                    if not result['result']:
                        data_str = f" (数据: {result['data']})" if result['data'] is not None else ""
                        print(f"  {fail_index}. [{section}] {result['test_name']}{data_str}")
                        fail_index += 1
        else:
            print("恭喜！所有测试项均通过！✓")

        print(f"\n{'=' * 80}")
        print("测试完成!")
        print(f"{'=' * 80}\n")

        # 提示信息
        print("提示:")
        print("1. 部分危险操作 (如修改 ID、波特率) 已被注释")
        print("2. 如需测试这些功能，请取消对应代码的注释")
        print("3. 电机校准需要等待约 20 秒")
        print("4. 请确保设备连接正常且电源稳定")
        print("=" * 80 + "\n")

    def cleanup(self):
        """清理测试环境"""
        print("\n正在关闭设备...")
        self.hand.close_device()
        print("设备已关闭，测试结束!")


def main():
    """主测试函数"""
    try:
        # 创建测试对象
        # 参数：端口号，波特率，设备 ID
        tester = TestZWHAND(lqs_id=0x01)

        # 运行所有测试
        tester.run_all_tests()

    except KeyboardInterrupt:
        print("\n\n测试被用户中断!")
    except Exception as e:
        print(f"\n测试过程中发生错误：{e}")
        import traceback
        traceback.print_exc()
    finally:
        # 清理资源
        try:
            tester.cleanup()
        except Exception:
            pass


if __name__ == '__main__':
    main()
