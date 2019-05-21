#!/usr/bin/env python3
'''Hello to the world from ev3dev.org'''

import os
import sys
import time
import json
import math
import urllib.request
from ev3dev.ev3 import LargeMotor, Sound, ColorSensor, GyroSensor

# state constants
ON = True
OFF = False


def debug_print(*args, **kwargs):
    '''Print debug messages to stderr.

    This shows up in the output panel in VS Code.
    '''
    print(*args, **kwargs, file=sys.stderr)

def get_angle(gyro, readings_to_average):
    angle = 0.0
    for _ in range(readings_to_average):
        angle += gyro.value()
    return angle/readings_to_average


rotate_log_number = 0
def rotate_to_angle(angle, mL, mR, gyro):
    mL.reset()
    mR.reset()
    e_old = integral = 0
    start_time = t_beg = t_old = time.time()
    errors = 0xffffffff
    check_stuck = 0xffffffff
    stuck_value = 0

    k_p = 0.5 
    k_i = 33; max_i = 0.25 
    k_d = 0
    speed = 30
    
    # global rotate_log_number
    # file = open("logs/rotate_to_angle_" + str(rotate_log_number) + ".txt", "w")
    # file.write("Time\tError\tIntegral\tU\n")
    # rotate_log_number += 1
    
    global left_rotations
    global right_rotations
    
    e = angle - get_angle(gyro, 3)

    if e < -20:
        left_rotations += 1
    if e > 20:
        right_rotations += 1

    global gyro_drift
    gyro_drift = int((right_rotations * -12/8) + (left_rotations * 0/8))
    e += gyro_drift 

    debug_print("Rotate for ", e, " degrees to ", angle, "degrees")

    while errors: # or error_history < 10:
        t = time.time()
         # increase to 'speed' over 1 seconds
        max_power = max(min((t - t_beg)* speed/1, speed), -speed)

        dT = t - t_old
        e = angle - gyro.value() + gyro_drift # Check if it's better to read the gyro less or even more times
        if e == 0:
            integral = 0
        errors = ((errors << 1) & 0xffffffff) | (e != 0)
        dE = e - e_old
        u = k_p * e + k_i * integral + k_d * dE / dT

        u = max(min(u, max_power), -max_power)

        # Bump if stuck on same error (not enough force to move)
        check_stuck = ((check_stuck << 1) & 0xffffffff) | (stuck_value != e)
        stuck_value = e
        if check_stuck == 0:
            u *= 2
            check_stuck = 0xffffffff

        # correction for wheel spin difference so the ev3 stays in same position
        if abs(e) > 4:
            k_ns_p = 1
            e_ns = mR.position + mL.position
            u_ns = k_ns_p * e_ns
            if u_ns > 0:
                u_ns_r = 0
                u_ns_l = u_ns
            else:
                u_ns_r = u_ns
                u_ns_l = 0
        else: 
            u_ns_r = 0
            u_ns_l = 0
            #k_p = 5

        print(e, integral, u, sep=', ')

        mR.run_direct(duty_cycle_sp=-u - u_ns_r)
        mL.run_direct(duty_cycle_sp=u - u_ns_l)

        integral = max(min(integral + e * dT, max_i), -max_i)
        e_old = e
        t_old = t

        # comment out when not needed
        #file.write(str(t-start_time) + "\t" + str(e) + "\t" + str(integral) + "\t" + str(u) + "\n")

    mR.run_direct(duty_cycle_sp=0)
    mL.run_direct(duty_cycle_sp=0)

    #file.close()


drive_log_number = 0
def drive_for_centimeters(distance, mL, mR, gyro, angle):
    distance_count = (distance*mR.count_per_rot)/17.16
    distance_count = int(round(distance_count))
    debug_print("Move for ", distance_count, "degrees")
    mL.reset()
    mR.reset()
    check_stuck = 0xffffffff
    stuck_value = 0
    errors = 0xff
    powr_corr = e_2_ang = integral_2 = e_old = e_old_2 = integral = 0
    start_time = t_beg = t_old = time.time()
    e = distance_count

    # global drive_log_number
    # file = open("logs/drive_for_centimeters_" + str(drive_log_number) + ".txt", "w")
    # file2 = open("logs/drive_for_centimeters2_" + str(drive_log_number) + ".txt", "w")
    # file.write("Time\tError\tIntegral\tU\n")
    # file2.write("Time\tError\tIntegral\tU\n")
    # drive_log_number += 1

    k_p = 0.2
    k_i = 28; max_i = 0.25 # 15 0.5
    k_d = 0

    k_p_2 = 1 # 1
    k_i_2 = 0.0; max_i_2 = 10 
    k_d_2 = 0

    speed = 35 

    if e > 0:
        powr_corr = 1.027
    else:
        powr_corr = 0.998

    global searching_neighbourhood
    global gyro_drift

    while errors:
        t = time.time()
         # increase to 'speed' over 2 seconds
        max_power = max(min((t - t_beg)* speed/2.0, speed), -speed)

        dT = t - t_old
        e = distance_count - int((mL.position + mR.position)/2)
        errors = ((errors << 1) & 0xff) | ((e > 1) | (e < -1))
        dE = e - e_old
        u = k_p * e +  k_i * integral + k_d * dE / dT

        u = max(min(u, max_power), -max_power)
        
        # Bump if stuck on same error (not enough force to move)
        check_stuck = ((check_stuck << 1) & 0xffffffff) | (stuck_value != e)
        stuck_value = e
        if check_stuck == 0:
            u *= 2
            check_stuck = 0xffffffff

        # --------------------------------------------------------------------------------------------
        if abs(e) > 15:
            measured_angle = get_angle(gyro, 1) - gyro_drift # Check if it's better to read the gyro just one time or even more times
            e_2_ang = (measured_angle - angle) # integral of the error is the actual error
            
            dE_2 = e_2_ang - e_old_2
            e_old_2 = e_2_ang
            u_2 = k_p_2 * e_2_ang + k_i_2 * integral_2 + k_d_2 * dE_2/dT
            u_2 = max(min(u_2, 10), -10)

            mL.run_direct(duty_cycle_sp=u*powr_corr - u_2)
            mR.run_direct(duty_cycle_sp=u + u_2)

            integral_2 += e_2_ang * dT
            integral_2 = max(min(integral_2, max_i_2), -max_i_2)
        else:
            mL.run_direct(duty_cycle_sp=u*powr_corr)
            mR.run_direct(duty_cycle_sp=u)
        # --------------------------------------------------------------------------------------------
        #debug_print("Time:", t, "Error:", e, "Integral:", integral, "u:", u, "Integral_2:", integral_2, "u_2:", u_2, "e_2:", e_2, "e_2_ang:", e_2_ang, sep=' ')
        print(e_2_ang, sep=', ')

        integral = max(min(integral + e * dT, max_i), -max_i)
        e_old = e
        t_old = t

        #file.write(str(t-start_time) + "\t" + str(-e) + "\t" + str(integral) + "\t" + str(u) + "\n")
        #file2.write(str(t-start_time) + "\t" + str(-e_2) + "\t" + str(integral_2) + "\t" + str(u_2) + "\n")

        if searching_neighbourhood and check_color():
            return

    mR.run_direct(duty_cycle_sp=0)
    mL.run_direct(duty_cycle_sp=0)

    # file.close()
    # file2.close()

def calculate_angle(angle, current_angle):
    factor = int(current_angle / 360)
    angle = angle + factor * 360

    if angle - current_angle > 180:
       angle = angle - 360
    elif current_angle - angle > 180:
        angle = angle + 360

    return angle

def go_to_location(x, y, current_x, current_y, mL, mR, gyro):
    global gyro_drift
    current_angle = get_angle(gyro, 5) - gyro_drift
    relative_angle = abs(current_angle % 360)
    delta_x = x - current_x
    delta_y = y - current_y

    debug_print("Moving to: ", x, ", ", y)#current_angle, relative_angle)

    if relative_angle == 0:
        if delta_x != 0:
            drive_for_centimeters(delta_x, mL, mR, gyro, current_angle)
        if delta_y != 0:
            angle = calculate_angle(90, current_angle)
            rotate_to_angle(angle, mL, mR, gyro)
            drive_for_centimeters(delta_y, mL, mR, gyro, angle)
            rotate_to_angle(angle, mL, mR, gyro)
        else:
            rotate_to_angle(current_angle, mL, mR, gyro)
    elif relative_angle == 180:
        if delta_x != 0:
            drive_for_centimeters(-delta_x, mL, mR, gyro, current_angle)
        if delta_y != 0:
            angle = calculate_angle(90, current_angle)
            rotate_to_angle(angle, mL, mR, gyro)
            drive_for_centimeters(delta_y, mL, mR, gyro, angle)
            rotate_to_angle(angle, mL, mR, gyro)
        else:
            rotate_to_angle(current_angle, mL, mR, gyro)
    elif relative_angle == 90:
        if delta_y != 0:
            drive_for_centimeters(delta_y, mL, mR, gyro, current_angle)
        if delta_x != 0:
            angle = calculate_angle(0, current_angle)
            rotate_to_angle(angle, mL, mR, gyro)
            drive_for_centimeters(delta_x, mL, mR, gyro, angle)
            rotate_to_angle(angle, mL, mR, gyro)
        else:
            rotate_to_angle(current_angle, mL, mR, gyro)
    elif relative_angle == 270:
        if delta_y != 0:
            drive_for_centimeters(-delta_y, mL, mR, gyro, current_angle)
        if delta_x != 0:
            angle = calculate_angle(0, current_angle)
            rotate_to_angle(angle, mL, mR, gyro)
            drive_for_centimeters(delta_x, mL, mR, gyro, angle)
            rotate_to_angle(angle, mL, mR, gyro)
        else:
            rotate_to_angle(current_angle, mL, mR, gyro)

def beep(times, beep_duration=1000):
    for i in range(times):
        Sound.tone(1500, beep_duration).wait()
        time.sleep(0.5)

def check_color():
    global cl
    color = cl.value()
    if color == 1: # BLACK: START, 2 second beep
        debug_print("Color sensor: START")
        return True
    elif color == 2: # BLUE: good condition, 1 beep
        debug_print("Color sensor: BLUE")
        return True
    elif color == 4: # YELLOW: critical condition, 2 beeps
        debug_print("Color sensor: YELLOW")
        return True
    elif color == 5: # RED: passed away, 3 beeps
        debug_print("Color sensor: RED")
        return True
    else:
        debug_print("Color sensor: UNKNOWN (" + str(color) + ")")
        return False

right_rotations = left_rotations = gyro_drift = 0
start_time = None
cl = None
searching_neighbourhood = False

def main():
    data = None

    if True:
        with open('zemljevid.json') as f:
            data = json.load(f)
    else:
        resource = urllib.request.urlopen('http://192.168.0.200:8080/zemljevid.json')
        content =  resource.read()
        content =  content.decode("utf-8")
        data = json.loads(content)

    os.system('setfont Lat15-TerminusBold14')
    if os.path.exists("logs"):
        import shutil
        shutil.rmtree("logs")
    os.mkdir("logs")

    mL = LargeMotor('outB'); mL.stop_action = 'hold'
    mR = LargeMotor('outC'); mR.stop_action = 'hold'
    global cl
    cl = ColorSensor()
    cl.mode = 'COL-COLOR'
    gy = GyroSensor()
    gy.mode = 'GYRO-RATE'
    gy.mode = 'GYRO-ANG'

    # Give gyro a bit of time to start
    time.sleep(3)

    global start_time
    start_time = time.time()
    debug_print("Start time: ", start_time)

    start = [0, 0]
    locations = []

    for key, item in data.items():
        if key == "start":
            start = item
        else:
            locations.append(item)

    # Sort by distance, TODO might be better to minimize turns by prioritizing victims that are in the same line
    locations = sorted(locations, key=lambda x: abs(start[0] - x[0]) + abs(start[1] - x[1]), reverse=False)
    current_location = start

    def reset_neighbourhood_search():
        global searching_neighbourhood
        nonlocal neighbourhood_locations
        searching_neighbourhood = False
        neighbourhood_locations = []

    global searching_neighbourhood
    searching_neighbourhood = False
    neighbourhood_locations = []
    

    while locations:
        next_location = locations.pop(0)
        go_to_location(x=next_location[0], y=next_location[1], current_x=current_location[0], current_y=current_location[1], mL=mL, mR=mR, gyro=gy)
        current_location = next_location

        color = cl.value()
        if color == 1: # BLACK: START, 2 second beep
            debug_print("Color sensor: START")
            beep(1, 2000)
            reset_neighbourhood_search()
            locations = sorted(locations, key=lambda x: abs(start[0] - x[0]) + abs(start[1] - x[1]), reverse=False)
        elif color == 2: # BLUE: good condition, 1 beep
            debug_print("Color sensor: BLUE")
            beep(1)
            reset_neighbourhood_search()
            locations.insert(0, start)
        elif color == 4: # YELLOW: critical condition, 2 beeps
            debug_print("Color sensor: YELLOW")
            beep(2)
            reset_neighbourhood_search()
            locations.insert(0, start)
        elif color == 5: # RED: passed away, 3 beeps
            debug_print("Color sensor: RED")
            beep(3)
            reset_neighbourhood_search()
            #locations.insert(0, start)
            locations = sorted(locations, key=lambda x: abs(current_location[0] - x[0]) + abs(current_location[1] - x[1]), reverse=False)
        else:
            debug_print("Color sensor: UNKNOWN (" + str(color) + ")")
            #locations.insert(0, start)
            if not searching_neighbourhood:
                searching_neighbourhood = True
                radius = 5
                for area in range(1,20):
                    neighbourhood_locations.append([next_location[0]+radius*area, next_location[1]-radius*area])
                    neighbourhood_locations.append([next_location[0]+radius*area, next_location[1]+radius*area])
                    neighbourhood_locations.append([next_location[0]-radius*area, next_location[1]+radius*area])
                    neighbourhood_locations.append([next_location[0]-radius*area, next_location[1]-radius*area])
            locations.insert(0, neighbourhood_locations[0])
            neighbourhood_locations = neighbourhood_locations[1:]


    # # Rotate back to original orientation
    # angle = calculate_angle(0, gy.value())
    # rotate_to_angle(angle, mL, mR, gy)

    

    # for i in range (16):
    #     rotate_to_angle(90, mL, mR, gy)
    #     rotate_to_angle(0, mL, mR, gy)
        
    
    

    #for _ in range (5):
        #rotate_to_angle(89, mL, mR, gy)
        #rotate_to_angle(178, mL, mR, gy)
        #rotate_to_angle(270, mL, mR, gy)
        #rotate_to_angle(180, mL, mR, gy)
        #rotate_to_angle(90, mL, mR, gy)
        #rotate_to_angle(0, mL, mR, gy)

    #rotate_to_angle(-91, mL, mR, gy)
    #rotate_to_angle(-182, mL, mR, gy)
    #rotate_to_angle(-2, mL, mR, gy)


    #rotate_to_angle(89, mL, mR, gy)
    #rotate_to_angle(178, mL, mR, gy)
    #rotate_to_angle(-2, mL, mR, gy)
    #rotate_to_angle(87, mL, mR, gy)
    #rotate_to_angle(176, mL, mR, gy)

    # for i in range (5):
    #     drive_for_centimeters(50, mL, mR, gy, 0)
    #     rotate_to_angle(0, mL, mR, gy)
    #     drive_for_centimeters(-50, mL, mR, gy, 0)
    #     rotate_to_angle(0, mL, mR, gy)

    debug_print("End time: ", time.time())

if __name__ == '__main__':
    main()
