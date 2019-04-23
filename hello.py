#!/usr/bin/env python3
'''Hello to the world from ev3dev.org'''

import os
import sys
import time
import json
import math
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

def get_angle(gyro, readings_to_average, dt=0, current_angle=0):
    #global start_time
    #delta_time = time.time() - start_time # compensation for gyro drift
    angle = 0.0
    for _ in range(readings_to_average):
        angle += gyro.value()
    return angle/readings_to_average #- int(delta_time/10)
    # Test for calculating angle from rotation rate... didn't seem to work better
    #current_angle += ((gyro.value() - gyro_drift) * dt)
    #return current_angle

rotate_log_number = 0
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

    global rotate_log_number
    file = open("logs/rotate_to_angle_" + str(rotate_log_number) + ".txt", "w")
    file.write("Time\tError\tIntegral\tU\n")
    rotate_log_number += 1

    speed = 40 #35

    start_time = time.time()

    while errors: # or error_history < 10:
        t = time.time()
         # increase to 'speed' over 1 seconds
        max_power = max(min((t - t_beg)* speed/1, speed), -speed)

        dT = t - t_old
        e = angle - get_angle(gyro, 5) # Check if it's better to read the gyro less or even more times
        errors = ((errors << 1) & 0xff) | (e != 0)
        dE = e - e_old
        u = k_p * e + max(min(k_i * integral, max_i), -max_i) + k_d * dE / dT

        u = max(min(u, max_power), -max_power)
        debug_print(t, e, dE, integral, u, sep=', ')
        mR.run_direct(duty_cycle_sp=-u)
        mL.run_direct(duty_cycle_sp=u)

        integral += e * dT
        integral = max(min(integral, max_i), -max_i)
        e_old = e
        t_old = t

        file.write(str(t-start_time) + "\t" + str(e) + "\t" + str(integral) + "\t" + str(u) + "\n")

    mR.run_direct(duty_cycle_sp=0)
    mL.run_direct(duty_cycle_sp=0)

    file.close()

drive_log_number = 0
def drive_for_centimeters(distance, mL, mR, gyro, angle):
    distance_count = (distance*mR.count_per_rot)/17.6
    distance_count = int(round(distance_count))
    debug_print("Move for ", distance_count, "degrees")
    mL.reset()
    mR.reset()

    global gyro_drift
    global drive_log_number

    file = open("logs/drive_for_centimeters_" + str(drive_log_number) + ".txt", "w")
    file2 = open("logs/drive_for_centimeters2_" + str(drive_log_number) + ".txt", "w")
    file.write("Time\tError\tIntegral\tU\n")
    file2.write("Time\tError\tIntegral\tU\n")
    drive_log_number += 1

    gyro_drift = gyro.value()
    measured_angle = angle

    errors = 0xff
    integral_2 = e_old = e_old_2 = integral = 0
    t_beg = t_old = time.time()
    e = distance_count

    k_p = 1
    k_i = 2; max_i = 15
    k_d = 0.01

    k_p_2 = 0.2
    k_i_2 = 1; max_i_2 = 10
    k_d_2 = 0.01
    # these work relatively okay
    #k_p_2 = 0.02
    #k_i_2 = 0.05; max_i_2 = 10 # before 1
    #k_d_2 = 0.025
    #k_p_2 = 0
    #k_i_2 = 0; max_i_2 = 10 # before 1
    #k_d_2 = 0

    speed = 50

    start_error = e
    start_time = time.time()

    while errors:
        t = time.time()
         # increase to 'speed' over 1.5 seconds
        max_power = max(min((t - t_beg)* speed/1.5, speed), -speed)

        dT = t - t_old
        e = distance_count - int((mL.position + mR.position)/2)
        errors = ((errors << 1) & 0xff) | ((e > 1) | (e < -1))
        dE = e - e_old
        u = k_p * e +  max(min(k_i * integral, max_i), -max_i) + k_d * dE / dT

        u = max(min(u, max_power), -max_power)
        #if u < 5:
        #    gyro_drift = gyro.value()
        # --------------------------------------------------------------------------------------------
        # Not the best way to do things ^^', in fact it probably wont work...
        measured_angle = get_angle(gyro, 1, dT, measured_angle) # Check if it's better to read the gyro just one time or even more times
        e_2 = (mL.position - mR.position) + (angle - measured_angle)
        #debug_print("GYRO VAL:" + str(gyro.value()) + " | DRIFT: " + str(gyro_drift))
        dE_2 = e_2 - e_old_2
        u_2 = k_p_2 * e_2 + k_i_2 * integral_2 + k_d_2 * dE_2/dT
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

        file.write(str(t-start_time) + "\t" + str(-e) + "\t" + str(integral) + "\t" + str(u) + "\n")
        file2.write(str(t-start_time) + "\t" + str(-e_2) + "\t" + str(integral_2) + "\t" + str(u_2) + "\n")

    mR.run_direct(duty_cycle_sp=0)
    mL.run_direct(duty_cycle_sp=0)

    file.close()
    file2.close()

def go_to_location(x, y, current_x, current_y, mL, mR, gyro):
    current_angle = get_angle(gyro, 5)
    delta_x = x - current_x
    delta_y = y - current_y
    angle = -math.floor(math.degrees(math.atan2(delta_y, delta_x)))
    debug_print(angle)
    # Not working in all cases
    while abs(angle-current_angle) > 180:
        debug_print(str(angle) + " | current_angle: " + str(current_angle))
        if angle > 0:
            angle = -(360-angle)
        elif angle < 0:
            angle = angle+360

    #a1 = current_angle % 360
    #if current_angle >= 0:
    #    a2 = 360 - a1
    #    a1 *= -1
    #    if abs(angle - a1) < abs(angle - a2):
    #        angle = current_angle - (a1 - angle)
    #    else:
    #        angle = current_angle - (a2 - angle)
    #else:
    #    a2 = 360 - a1
    #    if abs(angle - a1) < abs(angle - a2):
    #        angle = current_angle + (a1 - angle)
    #    else:
    #        angle = current_angle + (a2 - angle)

    debug_print("Angle converted to " + str(angle) + " | Current angle: " + str(current_angle))
    rotate_to_angle(angle, mL, mR, gyro)

    time.sleep(1)

    distance = math.floor(math.sqrt(delta_x**2 + delta_y**2))
    drive_for_centimeters(distance, mL, mR, gyro, angle)

def beep(times, beep_duration=1000):
    for i in range(times):
        Sound.tone(1500, beep_duration).wait()
        time.sleep(0.5)

def check_sensor(c):
    color = c.value()
    if color == 1: # BLACK: START, 2 second beep
        debug_print("Color sensor: START")
        beep(1, 2000)
        # TODO: go to next person
    elif color == 2: # BLUE: good condition, 1 beep
        debug_print("Color sensor: BLUE")
        beep(1)
        # TODO: go to start
    elif color == 4: # YELLOW: critical condition, 2 beeps
        debug_print("Color sensor: YELLOW")
        beep(2)
        # TODO: go to start
    elif color == 5: # RED: passed away, 3 beeps
        debug_print("Color sensor: RED")
        beep(3)
        # TODO: DON'T have to go to start, go to next person
    else:
        debug_print("Color sensor: UNKNOWN (" + str(color) + ")")

gyro_drift = 0
start_time = None

def main():
    '''The main function of our program'''

    data = None
    with open('zemljevid.json') as f:
        data = json.load(f)
    debug_print(data)
    debug_print(data["start"])
    debug_print(data["oseba1"])

    os.system('setfont Lat15-TerminusBold14')
    if os.path.exists("logs"):
        import shutil
        shutil.rmtree("logs")
    os.mkdir("logs")
    mL = LargeMotor('outB'); mL.stop_action = 'hold'
    mR = LargeMotor('outC'); mR.stop_action = 'hold'
    cl = ColorSensor()
    cl.mode = 'COL-COLOR'
    print(cl.value())
    debug_print(cl.value())
    print('Hello, my name is EV3!')

    gy = GyroSensor()
    gy.mode = 'GYRO-ANG'
    #gy.mode = 'GYRO-RATE'

    debug_print("Gyro: " + str(gy.value()))

    global start_time
    start_time = time.time()


    #drive_for_centimeters(100, mL, mR, gy, 0)
    #return

    #test_destinations = [[10, 0], [10, 10], [10, 0], [0, 0]]
    #test_destinations = [[60, 0], [0, 0]]
    #test_destinations = [[100, 0], [100, -30], [0, -30], [0, 0]]
    #test_destinations = [[20, 0], [40, 0], [60, 0], [80, 0], [100, 0]]
    #test_destinations = [[100, 0]]
    #test_destinations = [[96, 24], [0, 0], [60, 60], [0, 0]]
    #test_destinations = [[96, 0], [96, 24], [0, 24], [0, 0], [60, 0], [60, 60], [0, 60], [0, 0]]
    test_destinations = [[180, 0], [0, 0]]

    start = [0, 0]
    for d in test_destinations:
        go_to_location(x=d[0], y=d[1], current_x=start[0], current_y=start[1], mL=mL, mR=mR, gyro=gy)
        #debug_print("GYRO DRIFT: " + str(gyro_drift))
        check_sensor(cl)
        start = d
    rotate_to_angle(0, mL, mR, gy)

    #go_to_location(x=10, y=0, current_x=0, current_y=0, mL=mL, mR=mR, gyro=gy)


    #rotate_to_angle(90, mL, mR, gy)
    #rotate_to_angle(180, mL, mR, gy)
    #rotate_to_angle(0, mL, mR, gy)

    #drive_for_centimeters(170, mL, mR, gy, 0)
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
