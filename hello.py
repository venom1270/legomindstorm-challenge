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

    #Sound.speak('Hello, my name is EV3! Govorim tudi slovensko!').wait()
    #mL.run_to_rel_pos(position_sp= 840, speed_sp = 250)
    #mR.run_to_rel_pos(position_sp=-840, speed_sp = 250)
    mL.wait_while('running')
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

    debug_print("Gyro: " + str(gy.value()))

    # print something to the screen of the device
    #print('Hello World!')

    # print something to the output panel in VS Code
    #debug_print('Hello VS Code!')


if __name__ == '__main__':
    main()
