from flight import *


if __name__ == '__main__':
    q = get_quaternion_from_euler(1, 2, 3)
    roll, yaw, pitch = get_euler_from_quaternion(q)

    print(roll, yaw, pitch)