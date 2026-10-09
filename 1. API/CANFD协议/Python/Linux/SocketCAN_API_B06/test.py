import time
import threading
from CANFDPY import PCANFD


results = []


def print_vector(title, values):
    print(f"{title}: {values}")


def print_step(title, ok):
    results.append((title, ok))
    print(f"{title}: {'OK' if ok else 'FAIL'}")


def print_summary():
    passed = [name for name, ok in results if ok]
    failed = [name for name, ok in results if not ok]
    print("\n" + "=" * 60)
    print(f"  TOTAL: {len(results)}  |  PASS: {len(passed)}  |  FAIL: {len(failed)}")
    print("=" * 60)
    if passed:
        print("[PASSED]")
        for name in passed:
            print(f"  OK    {name}")
    if failed:
        print("[FAILED]")
        for name in failed:
            print(f"  FAIL  {name}")
    print("=" * 60)


def main():
    hand = PCANFD(channel='can0', arb_baud_rate=500000, data_baud_rate=1000000, lqs_id=0x01)

    if not hand.connect():
        print("Connect failed!")
        return
    print("Connect success!")

    # ===== 1. 手初始化 =====
    print("\n===== 1. Hand initialization =====")
    print(f"Init state before: {hand.get_initialize_state()}")
    print_step("Full motor calibration", hand.set_all_motor_calibration())
    print("Waiting 20 seconds for initialization...")
    time.sleep(20)

    print_step("Finger control mode", hand.set_finger_control_mode([1, 1, 1, 1, 1, 1]))

    hand.set_all_motor_current([50, 50, 50, 50, 50, 50])
    hand.set_all_motor_angle([900, 900, 900, 900, 900, 900])

    stop_read = threading.Event()

    def reader():
        while not stop_read.is_set():
            print_vector("Motor speed", hand.get_all_motor_speed())
            print_vector("Motor current", hand.get_all_motor_current())
            print_vector("Motor angle", hand.get_all_motor_angle())
            print_vector("Motor position", hand.get_motor_real_position())
            time.sleep(0.02)

    reader_thread = threading.Thread(target=reader)
    reader_thread.start()
    time.sleep(1.5)
    stop_read.set()
    reader_thread.join()


    print_step("All motor current to 80", hand.set_all_motor_current([80, 80, 80, 80, 80, 80]))
    print_step("All motor angle to 300", hand.set_all_motor_angle([300, 300, 300, 300, 300, 300]))
    print_vector("Motor speed", hand.get_all_motor_speed())
    print_vector("Motor current", hand.get_all_motor_current())
    print_vector("Motor angle", hand.get_all_motor_angle())
    time.sleep(1)
    print_vector("Motor position", hand.get_motor_real_position())
    time.sleep(3)

    print_step("Finger control mode", hand.set_finger_control_mode([1, 1, 1, 1, 1, 1]))
    time.sleep(1)
    print(f"Init state: {hand.get_initialize_state()}")
    print_step("All motor speed to 50", hand.set_all_motor_speed([50, 50, 50, 50, 50, 50]))
    print_step("All motor angle to 900", hand.set_all_motor_angle([900, 900, 900, 900, 900, 900]))
    print_vector("Motor speed", hand.get_all_motor_speed())
    print_vector("Motor current", hand.get_all_motor_current())
    print_vector("Motor angle", hand.get_all_motor_angle())
    time.sleep(1)
    print_vector("Motor position", hand.get_motor_real_position())
    time.sleep(1)

    print(f"Init state: {hand.get_initialize_state()}")
    print_step("All motor speed to 100", hand.set_all_motor_speed([100, 100, 100, 100, 100, 100]))
    print_step("All motor angle to 300", hand.set_all_motor_angle([300, 300, 300, 300, 300, 300]))
    print_vector("Motor speed", hand.get_all_motor_speed())
    print_vector("Motor current", hand.get_all_motor_current())
    print_vector("Motor angle", hand.get_all_motor_angle())
    time.sleep(1)
    print_vector("Motor position", hand.get_motor_real_position())
    time.sleep(1)

    # ===== 2. 基础配置 =====
    print("\n===== 2. Basic setup =====")
    print_step("Set all motor speed to 100", hand.set_all_motor_speed([100, 100, 100, 100, 100, 100]))
    print_step("Set all motor current to 80", hand.set_all_motor_current([80, 80, 80, 80, 80, 80]))
    print_step("Clear errors", hand.set_error_clear())

    # ===== 3. 状态读取 =====
    print("\n===== 3. Readback =====")
    print_step("Finger control mode", hand.set_finger_control_mode([1, 1, 1, 1, 1, 1]))
    time.sleep(2)

    ver = hand.get_bootloader_version()
    print_step("Get bootloader version", ver > 0)
    print(f"  -> {ver}")

    ver = hand.get_hardware_version()
    print_step("Get hardware version", ver > 0)
    print(f"  -> {ver}")

    ver = hand.get_software_version()
    print_step("Get software version", ver > 0)
    print(f"  -> {ver}")

    val = hand.get_device_error()
    print_step("Get device error", len(val) == 6)
    print_vector("  ->", val)

    val = hand.get_device_voltage()
    print_step("Get device voltage", val > 0)
    print(f"  -> {val:.3f} V")

    val = hand.get_motor_locked_state()
    print_step("Get motor locked state", len(val) == 6)
    print_vector("  ->", val)

    val = hand.get_motor_real_position()
    print_step("Get motor real position", len(val) == 6)
    print_vector("  ->", val)

    val = hand.get_all_motor_current()
    print_step("Get all motor current", len(val) == 6)
    print_vector("  ->", val)

    val = hand.get_phase_loss_fault()
    print_step("Get phase loss fault", len(val) == 6)
    print_vector("  ->", val)

    val = hand.get_cur_samp_fault()
    print_step("Get current sample fault", len(val) == 6)
    print_vector("  ->", val)

    val = hand.get_all_motor_angle()
    print_step("Get all motor angle", len(val) == 6)
    print_vector("  ->", val)

    val = hand.get_abs_max_position()
    print_step("Get abs max position", len(val) == 6)
    print_vector("  ->", val)

    val = hand.get_joint_abs_max()
    print_step("Get joint abs max", len(val) == 6)
    print_vector("  ->", val)

    val = hand.get_customer_number()
    print_step("Get customer number", val >= 0)
    print(f"  -> {val}")

    val = hand.get_skin_force()
    print_step("Get skin force", len(val) == 6)
    print_vector("  ->", val)

    val = hand.get_motor_temperature()
    print_step("Get motor temperature", len(val) == 6)
    print_vector("  ->", val)

    val = hand.get_hand_type()
    print_step("Get hand type", val in (0, 1))
    print(f"  -> {val}")

    # ===== 4. 写测试 =====
    print("\n===== 4. Write tests =====")
    print_step("Finger control mode", hand.set_finger_control_mode([1, 1, 1, 1, 1, 1]))
    time.sleep(1)
    print_step("All motor angle to 500", hand.set_all_motor_angle([500, 500, 500, 500, 500, 500]))
    time.sleep(2)
    print_step("Single motor absolute (motor1=300)", hand.set_single_motor_absolute(1, 300))
    time.sleep(2)
    print_step("All motor absolute to 300", hand.set_all_motor_absolute([300, 300, 300, 300, 300, 300]))
    time.sleep(2)
    print_step("Single motor relative (motor1=-100)", hand.set_single_motor_relative(1, -100))
    time.sleep(1)
    print_step("Single motor relative (motor2=-100)", hand.set_single_motor_relative(2, -100))
    time.sleep(1)
    print_step("Single motor relative (motor3=-100)", hand.set_single_motor_relative(3, -100))
    time.sleep(1)
    print_step("Single motor relative (motor4=-100)", hand.set_single_motor_relative(4, -100))
    time.sleep(1)
    print_step("Single motor relative (motor5=-100)", hand.set_single_motor_relative(5, -100))
    time.sleep(1)
    print_step("Single motor relative (motor6=-100)", hand.set_single_motor_relative(6, -100))
    time.sleep(2)
    print_step("All motor relative to 200", hand.set_all_motor_relative([200, 200, 200, 200, 200, 200]))
    time.sleep(2)
    print_step("Finger control mode to 0", hand.set_finger_control_mode([0, 0, 0, 0, 0, 0]))
    time.sleep(1)
    print_step("PC params", hand.set_pc_control_param([5000, -5000, 5000, -5000, -5000, -5000]))
    time.sleep(2)
    print_step("Finger control mode to 2 (PVT)", hand.set_finger_control_mode([2, 2, 2, 2, 2, 2]))
    time.sleep(1)
    print_step("PVT params", hand.set_pvt_control_param([10, 0, 1000,
                                                         10, 0, 1000,
                                                         10, 0, 1000,
                                                         1000, 0, 1000,
                                                         1000, 0, 1000,
                                                         1000, 0, 1000]))
    time.sleep(2)
    print_step("PVT params again", hand.set_pvt_control_param([10, 0, 1000,
                                                               10, 0, 1000,
                                                               10, 0, 1000,
                                                               10, 0, 1000,
                                                               10, 0, 1000,
                                                               10, 0, 1000]))
    time.sleep(2)
    print_step("Hand test ON", hand.set_hand_test(1))
    time.sleep(15)
    print_step("Hand test OFF", hand.set_hand_test(0))

    print("\nTest finished")
    print_summary()
    hand.disconnect()







if __name__ == '__main__':
    main()
