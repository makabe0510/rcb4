from rcb4.armh7interface import ARMH7Interface
import numpy as np
import time

def read_voltage_test(interface):
    val = interface.read_imu_data()
    print(val)
    return val

def free_gripper(interface):
    interface.angle_vector([0, 0], servo_ids=[5, 7])

def open_gripper_init():
    interface.hold()
    interface.angle_vector([-125], servo_ids=[3])
    interface.angle_vector([5, 5], servo_ids=[5, 7])
    time.sleep(10)
    interface.angle_vector([0, 0], servo_ids=[5, 7])

def loosen_stopper():
    interface.angle_vector([-125], servo_ids=[3])
    interface.angle_vector([-15, -15], servo_ids=[5, 7])
    time.sleep(2)
    interface.angle_vector([0, 5], servo_ids=[5, 7])
    time.sleep(1.5)
    interface.angle_vector([-10, -10], servo_ids=[5, 7])
    time.sleep(9)
    print("grasp")
    interface.angle_vector([0, -4.5], servo_ids=[5, 7])
    time.sleep(5)
    interface.angle_vector([20, 13], servo_ids=[5, 7])
    time.sleep(8)
    # todo 
    interface.angle_vector([0, -4], servo_ids=[5, 7])
    time.sleep(1)
    # # interface.angle_vector([0, 0], servo_ids=[5, 7])
    interface.angle_vector([-90], servo_ids=[3])
    interface.angle_vector([0, 0], servo_ids=[5, 7])
    time.sleep(3)

def insert_stopper():
    # interface.angle_vector([0, -10], servo_ids=[5, 7])
    # time.sleep(1)
    interface.angle_vector([-125], servo_ids=[3])
    time.sleep(3)
    interface.angle_vector([0, -4], servo_ids=[5, 7])
    time.sleep(1)
    interface.angle_vector([-10, -10], servo_ids=[5, 7])
    time.sleep(12)
    interface.angle_vector([0, 2], servo_ids=[5, 7])
    time.sleep(1)
    interface.angle_vector([10, 10], servo_ids=[5, 7])
    time.sleep(12)
    interface.angle_vector([0, 0], servo_ids=[5, 7])
    interface.angle_vector([-90], servo_ids=[3])


if __name__ == "__main__":
    interface = ARMH7Interface()
    try:
        print(interface.auto_open())
        open_gripper_init()
    except Exception as e:
        if "LIBUSB_ERROR_ACCESS" in str(e):
            # Error already handled and printed by the class method
            pass
        else:
            print(f"Error: {e}")
