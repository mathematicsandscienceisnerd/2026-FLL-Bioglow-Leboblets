#Run this before running the program.
#runfll - it is an alias to open the virtual environment

#Once you commit, type " git push -u origin main " into the terminal to publish.

from pybricks.hubs import PrimeHub
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

hub = PrimeHub()

ld = Motor(Port.D, positive_direction=Direction.COUNTERCLOCKWISE)
rd = Motor(Port.C, positive_direction=Direction.CLOCKWISE)

MISSIONS = []