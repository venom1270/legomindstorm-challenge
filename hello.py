#!/usr/bin/env python3
'''Hello to the world from ev3dev.org'''

import os
import sys
import time
from ev3dev.ev3 import LargeMotor, Sound, ColorSensor, GyroSensor

# state constants
ON = True
OFF = False


def debug_print(*args, **kwargs):
    '''Print debug messages to stderr.

    This shows up in the output panel in VS Code.
    '''
    print(*args, **kwargs, file=sys.stderr)


def reset_console():
    '''Resets the console to the default state'''
    print('\x1Bc', end='')


def set_cursor(state):
    '''Turn the cursor on or off'''
    if state:
        print('\x1B[?25h', end='')
    else:
        print('\x1B[?25l', end='')


def set_font(name):
    '''Sets the console font

    A full list of fonts can be found with `ls /usr/share/consolefonts`
    '''
    os.system('setfont ' + name)

def get_angle(gyro, readings_to_average):
    angle = 0.0
    for _ in range(readings_to_average):
        angle += gyro.value()
    return angle/readings_to_average

def rotate_to_angle(angle, mL, mR, gyro):
    debug_print("Rotate to ", angle, "degrees")
    mL.position = mR.position = 0
    e_old = integral = 0
    t_beg = t_old = time.time()
    e = angle - get_angle(gyro, 3)
    k_p = 2
    k_i = 3
    k_d = 0
    errors = 0xff

    while errors: 
        t = time.time()
         # increase to 100 over 1.5 seconds
        max_power = max(min((t - t_beg)* 100/1.5, 100), -100)

        dT = t - t_old
        e = angle - get_angle(gyro, 3)
        errors = ((errors << 1) & 0xff) | (e != 0)
        dE = e - e_old
        u = k_p * e + max(min(k_i * integral, 10), -10) + k_d * dE / dT

        u = max(min(u, max_power), -max_power)
        debug_print(t, e, dE, integral, u, sep=', ')
        mR.run_direct(duty_cycle_sp=-u)
        mL.run_direct(duty_cycle_sp=u)

        integral += e * dT
        integral = max(min(integral, 10), -10)
        e_old = e
        t_old = t

    mR.run_direct(duty_cycle_sp=0)
    mL.run_direct(duty_cycle_sp=0)

def drive_for_centimeters(distance, mL, mR, gyro, angle):
    distance_count = (distance*mR.count_per_rot)/17.6
    distance_count = int(round(distance_count))
    debug_print("Move for ", distance_count, "degrees")
    mL.position = mR.position = 0

    errors = 0xff
    e_old = integral = 0
    t_beg = t_old = time.time()
    e = distance_count
    k_p = 0.4
    k_i = 0.7; max_i = 15
    k_d = 0

    while errors: 
        t = time.time()
         # increase to 90 over 1.5 seconds
        max_power = max(min((t - t_beg)* 90/1.5, 90), -90)

        dT = t - t_old
        e = distance_count - int((mL.position + mR.position)/2)
        errors = ((errors << 1) & 0xff) | (e != 0)
        dE = e - e_old
        u = k_p * e +  max(min(k_i * integral, max_i), -max_i) + k_d * dE / dT

        u = max(min(u, max_power), -max_power)
        debug_print(t, e, dE, integral, u , sep=', ')
        mrsnod(u, u, mL, mR, gyro, angle)

        integral += e * dT
        integral = max(min(integral, max_i), -max_i)
        e_old = e
        t_old = t

    mR.run_direct(duty_cycle_sp=0)
    mL.run_direct(duty_cycle_sp=0)

# motor_rotate_same_number_of_degrees
def mrsnod(duty_cycle_l, duty_cycle_r, mL, mR, gyro, angle):
    # Not the best way to do things ^^', in fact it probably wont work...
    k_p = 0.5
    e = (mL.position - mR.position) + (angle - get_angle(gyro, 2))
    u = max(min(e * k_p, 10), -10)

    mL.run_direct(duty_cycle_sp=duty_cycle_l - u)
    mR.run_direct(duty_cycle_sp=duty_cycle_r + u)



def main():
    '''The main function of our program'''

    os.system('setfont Lat15-TerminusBold14')
    mL = LargeMotor('outB'); mL.stop_action = 'hold'
    mR = LargeMotor('outC'); mR.stop_action = 'hold'
    cl = ColorSensor()
    cl.mode = 'COL-COLOR'
    print(cl.value())
    debug_print(cl.value())
    print('Hello, my name is EV3!')

    gy = GyroSensor()
    gy.mode = 'GYRO-ANG'

    debug_print("Gyro: " + str(gy.value()))


    drive_for_centimeters(10, mL, mR, gy, 0)
    rotate_to_angle(90, mL, mR, gy)
    drive_for_centimeters(20, mL, mR, gy, 90)
    rotate_to_angle(270, mL, mR, gy, )
    drive_for_centimeters(20, mL, mR, gy, 270)
    rotate_to_angle(45, mL, mR, gy)
    rotate_to_angle(0, mL, mR, gy)
    drive_for_centimeters(-10, mL, mR, gy, 0)

    



    #Sound.speak('Hello, my name is EV3! Govorim tudi slovensko!').wait()
    #mL.run_to_rel_pos(position_sp= 840, speed_sp = 250)
    #mR.run_to_rel_pos(position_sp=-840, speed_sp = 250)
    """ mL.wait_while('running')
    mR.wait_while('running')


    mL.run_timed(time_sp=10000, speed_sp=100)
    mR.run_timed(time_sp=10000, speed_sp=100)

    #while cl.value() > 50 or cl.value() < 20:
    while cl.value() != 7:
        debug_print(cl.value())
        time.sleep(0.1)

    mL.stop(stop_action="hold")
    mR.stop(stop_action="hold")



    debug_print("End")
    mR.run_to_rel_pos(position_sp=360, speed_sp=300, stop_action="hold")

    time.sleep(3)

    debug_print("Gyro: " + str(gy.value())) """

    # print something to the screen of the device
    #print('Hello World!')

    # print something to the output panel in VS Code
    #debug_print('Hello VS Code!')


if __name__ == '__main__':
    main()
