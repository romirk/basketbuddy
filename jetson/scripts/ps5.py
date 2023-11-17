import argparse
import socket

from pydualsense import *

parser = argparse.ArgumentParser(description="PS5 controller test")
parser.add_argument("--ip", type=str, default="192.168.1.100", help="UDP IP address")
parser.add_argument("--port", type=int, default=5000, help="UDP port")
parser.add_argument("--deadzone", type=int, default=10, help="Joystick deadzone")
args = parser.parse_args()

UDP_IP = args.ip
UDP_PORT = args.port
DEADZONE = args.deadzone

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def send_cmd(cmd):
    sock.sendto(cmd, (UDP_IP, UDP_PORT))


def cmd_vel(linear, angular):
    if linear < -100 or linear > 100 or angular < -100 or angular > 100:
        print("Error: velocity must be between -100 and 100")
        exit(1)
    cmd = bytes([ord("V"), linear + 100, angular + 100])
    # print(f"{linear:04d}, {angular:04d}", end="\r")
    print(f"[{' ' * ((linear + 100) // 10)}#{' ' * ((100 - linear) // 10)}]" +
            f"[{' ' * ((angular + 100) // 10)}#{' ' * ((100 - angular) // 10)}]", end="\r")
    send_cmd(cmd)


def stop():
    cmd = bytes([ord("S"), 0, 0])
    send_cmd(cmd)


def brake(state):
    if state:
        cmd = bytes([ord("B"), 1, 1])
        send_cmd(cmd)


def led_toggle(state):
    if state:
        cmd = bytes([ord("L"), 3, 3])
        send_cmd(cmd)


def lift_up(state):
    if state:
        cmd = bytes([ord("L"), ord("U")])
        send_cmd(cmd)


def lift_down(state):
    if state:
        cmd = bytes([ord("L"), ord("D")])
        send_cmd(cmd)


def lift_stop(state):
    if state:
        cmd = bytes([ord("L"), ord("S")])
        send_cmd(cmd)


velocity = [0, 0]


def linear_stick(x, y):
    global velocity
    # map y from [-127, 128] to [-100, 100]
    y = -int(y * (100 / 127))
    # interpolate between red and purple
    color = [int(255 * (y + 100) / 200), 0, int(255 * (100 - y) / 200)]
    dualsense.light.setColorI(color[0], color[1], color[2])
    if abs(y) > DEADZONE:
        velocity[0] = y
        cmd_vel(velocity[0], velocity[1])
    elif velocity[0] != 0:
        velocity[0] = 0
        cmd_vel(velocity[0], velocity[1])


def angular_stick(x, y):
    global velocity
    # map x from [-127, 128] to [-100, 100]
    x = -int(x * (100 / 127))
    if abs(x) > DEADZONE:
        velocity[1] = x
        cmd_vel(velocity[0], velocity[1])
    elif velocity[1] != 0:
        velocity[1] = 0
        cmd_vel(velocity[0], velocity[1])


# def gyro_changed(pitch, yaw, roll):
#     print(f'{pitch}, {yaw}, {roll}')

if __name__ == "__main__":
    # create dualsense
    dualsense = pydualsense()
    # find device and initialize
    dualsense.init()

    dualsense.light.setColorI(0, 0, 255)

    # add events handler functions
    dualsense.left_joystick_changed += linear_stick
    dualsense.right_joystick_changed += angular_stick
    dualsense.square_pressed += brake
    dualsense.dpad_up += lift_up
    dualsense.dpad_down += lift_down
    dualsense.dpad_left += lift_stop

    # dualsense.gyro_changed += gyro_changed

    # read controller state until options is pressed
    while not dualsense.state.options:
        ...

    # set light to blue
    dualsense.light.setColorI(0, 0, 255)

    # close device
    dualsense.close()
