#Run this before running the program.
#runfll - it is an alias to open the virtual environment
#startfll - it is an alias to run the program.

#Once you commit, type " git push -u origin main " into the terminal to publish.

from pybricks.hubs import PrimeHub
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

hub = PrimeHub()

ld = Motor(Port.D, positive_direction=Direction.COUNTERCLOCKWISE)
rd = Motor(Port.C, positive_direction=Direction.CLOCKWISE)

AXLE_TRACK = 135
WHEEL_DIAMETRE = 61

drive_base = DriveBase(ld, rd, wheel_diameter=WHEEL_DIAMETRE, axle_track=AXLE_TRACK)

missions = []

def mission(func_ptr):
    """Register a mission function in menu order."""
    missions.append(func_ptr)
    return func_ptr

drive_base.straight(500)