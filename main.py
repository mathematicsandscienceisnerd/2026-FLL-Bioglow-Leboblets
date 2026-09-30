#Run this before running the program.
#runfll - it is an alias to open the virtual environment
#startfll - it is an alias to run the program.
#turn the robot on and press bluetooth button before starting
#Use side buttons to select a mission and central one to run

#Once you commit, type " git push -u origin main " into the terminal to publish.

from pybricks.hubs import PrimeHub
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

hub = PrimeHub()

ld = Motor(Port.E, positive_direction=Direction.COUNTERCLOCKWISE)
rd = Motor(Port.F, positive_direction=Direction.CLOCKWISE)

lbm = Motor(Port.C)
rbm = Motor(Port.D)

AXLE_TRACK = 135
WHEEL_DIAMETRE = 60

class LebobletsBase(DriveBase):
    def _stop_with(self, then):
        self.stop()
        if then == Stop.HOLD:
            ld.hold()
            rd.hold()
        elif then == Stop.BRAKE:
            ld.brake()
            rd.brake()

    def _wait_until_stalled(
        self, speed_tolerance: int = 0, turn_tolerance: int = 0, then=Stop.COAST
    ):
        if speed_tolerance == 0 and turn_tolerance == 0:
            while not self.stalled():
                wait(10)
        else:
            wait(200)
            while True:
                state = self.state()
                if abs(state[1]) <= speed_tolerance and abs(state[3]) <= turn_tolerance:
                    break
                wait(10)
        self._stop_with(then)

    def straight_until_stalled(
        self, speed: int = None, tolerance: int = 0, then=Stop.COAST
    ):
        if speed is None:
            speed = self.settings()[0]
        self.drive(speed, 0)
        self._wait_until_stalled(speed_tolerance=tolerance, then=then)

    def arc_until_stalled(self, radius: float, tolerance: int = 0, then=Stop.COAST):
        straight_speed = self.settings()[0]
        max_turn_rate = self.settings()[2]
        turn_rate = straight_speed / radius * (180 / 3.14159265)
        if abs(turn_rate) > max_turn_rate:
            straight_speed = abs(max_turn_rate * radius * 3.14159265 / 180)
            turn_rate = straight_speed / radius * (180 / 3.14159265)
        self.drive(straight_speed, turn_rate)
        self._wait_until_stalled(speed_tolerance=tolerance, then=then)

    def turn_until_stalled(
        self, turn_rate: int = None, tolerance: int = 0, then=Stop.COAST
    ):
        if turn_rate is None:
            turn_rate = self.settings()[2]
        self.drive(0, turn_rate)
        wait(200)  
        while True:
            if ld.stalled() or rd.stalled():
                break
            if tolerance > 0:
                state = self.state()
                if abs(state[3]) <= tolerance:
                    break
            wait(10)
        self._stop_with(then)

drive_base = LebobletsBase(ld, rd, wheel_diameter=WHEEL_DIAMETRE, axle_track=AXLE_TRACK)
drive_base.use_gyro(True)

missions = []

def mission(description):
    def decorator(func_ptr):
        missions.append({"func": func_ptr, "desc": description})
        return func_ptr
    return decorator

def reset_robot():
    drive_base.stop()
    lbm.stop()
    rbm.stop()
    wait(50)

    drive_base.reset()
    hub.imu.reset_heading(0)
    lbm.reset_angle(0)
    rbm.reset_angle(0)

storage_offset = 0

def load_saved_mission_index():
    try:
        data = hub.system.storage(storage_offset, read=1)
        if len(data) == 1:
            return data[0] % len(missions)
    except Exception:
        pass
    return 0

def save_mission_index(index):
    try:
        hub.system.storage(storage_offset, write=bytes([index]))
    except Exception:
        pass

def mission_selector():
    mission_index = load_saved_mission_index()

    print(f"\n[Selected Mission {mission_index + 1}]: {missions[mission_index]['desc']}")

    while True:
        hub.display.char(str(mission_index + 1))
        pressed = hub.buttons.pressed()

        if Button.LEFT in pressed:
            mission_index = (mission_index + 1) % len(missions)
            print(f"\n[Selected Mission {mission_index + 1}]: {missions[mission_index]['desc']}")
            while hub.buttons.pressed():
                wait(20)

        elif Button.RIGHT in pressed:
            mission_index = (mission_index - 1) % len(missions)
            print(f"\n[Selected Mission {mission_index + 1}]: {missions[mission_index]['desc']}")
            while hub.buttons.pressed():
                wait(20)

        elif Button.CENTER in pressed:
            while hub.buttons.pressed():
                wait(20)
            next_index = (mission_index + 1) % len(missions)
            save_mission_index(next_index)
            hub.light.on(Color.GREEN)
            timer = StopWatch()
            try:
                timer.resume()
                missions[mission_index]["func"]()
            finally:
                print(f"Time Elapsed: {timer.time()}ms")
                reset_robot()
                hub.light.on(Color.BLUE)
            mission_index = next_index
            print(f"\n[Selected Mission {mission_index + 1}]: {missions[mission_index]['desc']}")

        wait(20)

@mission("Drives forward.")
def mission_1():
    wait(500)
    drive_base.straight(400)

@mission("Drives backward.")
def mission_2():
    wait(500)
    drive_base.straight(-400)

@mission("Turns 90 degrees right.")
def mission_3():
    wait(500)
    drive_base.turn(90)

@mission("Spins left attachment motor a full revolution clockwise.")
def mission_4():
    wait(500)
    lbm.run_angle(300, -120)
    lbm.run_angle(300, 120)

@mission("Spins right attachment motor a fuull revolution anticlockwise.")
def mission_5():
    wait(500)
    rbm.run_angle(300, -360)

@mission("Drives straight until stalled.")
def mission_6():
    wait(500)
    drive_base.straight_until_stalled()

@mission("Test attachment.")
def mission_7():
    print(hub.imu.heading())
    wait(500)
    drive_base.straight(300)
    drive_base.turn(540)
    drive_base.straight(500)
    print(hub.imu.heading()%180)
    
@mission("Square")
def mission_8():
    wait(500)
    print(hub.imu.heading())
    drive_base.straight(300)
    drive_base.turn(90)
    print(hub.imu.heading())
    drive_base.straight(300)
    drive_base.turn(90)
    print(hub.imu.heading())
    drive_base.straight(300)
    drive_base.turn(90)
    print(hub.imu.heading())
    drive_base.straight(300)
    drive_base.turn(90)
    print(hub.imu.heading())

@mission("Tree lift")
def mission_9():
    wait(500)
    drive_base.settings(straight_speed=500)
    drive_base.straight(100)
    drive_base.settings(straight_speed=1000)

def main():
    voltage = hub.battery.voltage()
    percentage = max(0, min(100, (voltage - 6000) * 100 // (8400 - 6000)))
    settings = drive_base.settings()
    print(f"Battery: {str(percentage)}% ({str(voltage)}mV)")
    print(f"Settings: {tuple(settings) if settings else ()}")

    hub.system.set_stop_button(Button.BLUETOOTH)
    hub.display.orientation(Side.BOTTOM)
    hub.light.on(Color.RED if percentage < 75 else Color.BLUE)  
    reset_robot()
    drive_base.settings(
        straight_speed=1000,
        straight_acceleration=1000,
        turn_rate=400,
        turn_acceleration=500
    )
   
    mission_selector()  

main()