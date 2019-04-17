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
    mL.reset() 
    mR.reset()
    e_old = integral = 0
    t_beg = t_old = time.time()
    e = angle - get_angle(gyro, 3)
    k_p = 2.5
    k_i = 1; max_i = 15
    k_d = 0
    errors = 0xff

    while errors: 
        t = time.time()
         # increase to 30 over 1 seconds
        max_power = max(min((t - t_beg)* 30/1, 30), -30)

        dT = t - t_old
        e = angle - get_angle(gyro, 3)
        errors = ((errors << 1) & 0xff) | (e != 0)
        dE = e - e_old
        u = k_p * e + max(min(k_i * integral, max_i), -max_i) + k_d * dE / dT

        u = max(min(u, max_power), -max_power)
        debug_print(t, e, dE, integral, u, sep=', ')
        mR.run_direct(duty_cycle_sp=u)
        mL.run_direct(duty_cycle_sp=-u)

        integral += e * dT
        integral = max(min(integral, max_i), -max_i)
        e_old = e
        t_old = t

    mR.run_direct(duty_cycle_sp=0)
    mL.run_direct(duty_cycle_sp=0)

def drive_for_centimeters(distance, mL, mR, gyro, angle):
    distance_count = (distance*mR.count_per_rot)/17.6
    distance_count = int(round(distance_count))
    debug_print("Move for ", distance_count, "degrees")
    mL.reset()
    mR.reset()

    errors = 0xff
    integral_2 = e_old = integral = 0
    t_beg = t_old = time.time()
    e = distance_count
    k_p = 1
    k_i = 2; max_i = 10
    k_d = 0

    k_p_2 = 0.6
    k_i_2 = 1; max_i_2 = 10

    while errors: 
        t = time.time()
         # increase to 50 over 1.5 seconds
        max_power = max(min((t - t_beg)* 50/1.5, 50), -50)

        dT = t - t_old
        e = distance_count - int((mL.position + mR.position)/2)
        errors = ((errors << 1) & 0xff) | ((e > 1) | (e < -1))
        dE = e - e_old
        u = k_p * e +  max(min(k_i * integral, max_i), -max_i) + k_d * dE / dT

        u = max(min(u, max_power), -max_power)
        # --------------------------------------------------------------------------------------------
        # Not the best way to do things ^^', in fact it probably wont work...
        e_2 = (mL.position - mR.position) + (angle - get_angle(gyro, 2))
        u_2 = k_p_2 * e_2 + k_i_2 * integral_2
        u_2 = max(min(u_2, 30), -30)

        mL.run_direct(duty_cycle_sp=u - u_2)
        mR.run_direct(duty_cycle_sp=u + u_2)

        integral_2 += e_2 * dT
        integral_2 = max(min(integral_2, max_i_2), -max_i_2)
        # --------------------------------------------------------------------------------------------
        debug_print("Time: ", t, "Error: ", e, "Integral: ", integral, "u:", u, "Integral_2: ", integral_2, "u_2: ", u_2, "e_2:", e_2, sep=' ')


        integral += e * dT
        integral = max(min(integral, max_i), -max_i)
        e_old = e
        t_old = t

    mR.run_direct(duty_cycle_sp=0)
    mL.run_direct(duty_cycle_sp=0)


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


    drive_for_centimeters(170, mL, mR, gy, 0)
    # rotate_to_angle(90, mL, mR, gy)
    # drive_for_centimeters(100, mL, mR, gy, 90)
    # rotate_to_angle(180, mL, mR, gy)
    # drive_for_centimeters(100, mL, mR, gy, 180)
    # rotate_to_angle(270, mL, mR, gy)
    # drive_for_centimeters(100, mL, mR, gy, 270)
    # rotate_to_angle(0, mL, mR, gy)
    # rotate_to_angle(45, mL, mR, gy)
    # rotate_to_angle(0, mL, mR, gy)
    # drive_for_centimeters(-10, mL, mR, gy, 0)

    



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
