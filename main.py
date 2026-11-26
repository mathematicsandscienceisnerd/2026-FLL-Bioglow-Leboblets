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

ld = Motor(Port.F, positive_direction=Direction.COUNTERCLOCKWISE)
rd = Motor(Port.E, positive_direction=Direction.CLOCKWISE)

rbm = Motor(Port.C)
lbm = Motor(Port.D)

AXLE_TRACK = 137
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

    def first_drive_callibrate(self, time):
        time = time
        speed = 400
        calibrate_value = 7
        self.drive(speed,calibrate_value)
        wait(time)
        self._stop_with(then="")

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
    mission_index = 5

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

@mission("Blue side.")
def mission_3():
    global drive_base
    lbm.run_angle(500, 120)
    lbm.run_target(500, 0)
    drive_base.straight(660)
    lbm.run_angle(300, 100)
    drive_base.straight(10)
    lbm.run_angle(300, 80)
    drive_base.straight(15)
    lbm.hold()
    drive_base.straight(30)
    drive_base.turn(45)
    drive_base.straight(90)
    wait(800)

    drive_base.straight(-90)
    drive_base.turn(11)
    drive_base.straight(70)
    drive_base.turn(19)
    drive_base.straight(-200)
    drive_base.turn(42)
    drive_base.straight(-195)
    drive_base.turn(105)
    drive_base.straight(-80)
    drive_base.settings(
        straight_speed=100,
        straight_acceleration=1000,
        turn_rate=400,
        turn_acceleration=500
    )
    drive_base.straight(-200)
    drive_base.settings(
        straight_speed=1000,
        straight_acceleration=1000,
        turn_rate=400,
        turn_acceleration=500
    )
    drive_base.straight(130)
    drive_base.turn(90)
    drive_base.straight(-800)

@mission("Seed+Rock")
def mission_4():
    wait(500)
    drive_base.straight(400)
    drive_base.straight(-400)
    rbm.run_until_stalled(-300)
    drive_base.straight(100)
    drive_base.turn(50)
    drive_base.straight(350)
    rbm.run_until_stalled(300)
    rbm.hold()
    drive_base.straight(-350)
    drive_base.turn(-50)
    drive_base.straight(-100)

@mission("Drone Mission+Animal Drop")
def mission_5():
    wait(500)
    drive_base.straight(285)
    drive_base.settings(
        straight_speed=100,
        straight_acceleration=1000,
        turn_rate=400,
        turn_acceleration=500
    )
    drive_base.straight(30)
    drive_base.settings(
        straight_speed=1000,
        straight_acceleration=1000,
        turn_rate=400,
        turn_acceleration=500
    )
    drive_base.straight(405)
    drive_base.straight(-55)
    drive_base.turn(-48)
    drive_base.turn(48)
    drive_base.straight(-320)
    drive_base.turn(-45)
    drive_base.straight(340)
    drive_base.turn(20)
    drive_base.straight(175)
    drive_base.turn(25)
    drive_base.straight(95)
    lbm.run_angle(250,145)
    lbm.run_angle(400,-145)
    drive_base.straight(325)
    drive_base.turn(25)
    drive_base.straight(600)

@mission("Tree + Vine")
def mission_6():
    wait(500)
    drive_base.straight(90)
    drive_base.turn(20)
    drive_base.straight(475)
    drive_base.turn(-20)
    rbm.run_angle(500,-80)
    drive_base.straight(185)
    
    rbm.run_angle(700,150)
    drive_base.settings(
        straight_speed=100,
        straight_acceleration=1000,
        turn_rate=400,
        turn_acceleration=500
    )
    drive_base.straight(80)
    rbm.run_angle(500,-95)
    drive_base.straight(-20)
    rbm.run_angle(500,-95)
    drive_base.straight(-30)
    rbm.run_angle(500,350)
    drive_base.settings(
        straight_speed=300,
        straight_acceleration=1000,
        turn_rate=400,
        turn_acceleration=500
    )
    drive_base.straight(150)
    drive_base.turn(-20)
    rbm.run_angle(500,-30)



    drive_base.settings(
        straight_speed=1000,
        straight_acceleration=1000,
        turn_rate=400,
        turn_acceleration=500
    )
    drive_base.straight(-200)
    # rbm.run_angle(400,50)



    # rbm.run_angle(300, 40)
    # rbm.run_target(500, 0)
    # drive_base.straight(200)
    # drive_base.turn(45)
    # drive_base.straight(640)
    # drive_base.turn(-45)
    # drive_base.straight(-60)
    # rbm.run_angle(500, 30)
    # drive_base.settings(
    #     straight_speed=200,
    #     straight_acceleration=1000,
    #     turn_rate=400,
    #     turn_acceleration=500
    # )
    # drive_base.straight(100)
    # drive_base.settings(
    #     straight_speed=1000,
    #     straight_acceleration=1000,
    #     turn_rate=400,
    #     turn_acceleration=500
    # )
    # drive_base.straight(100)
    # drive_base.settings(
    #     straight_speed=200,
    #     straight_acceleration=1000,
    #     turn_rate=400,
    #     turn_acceleration=500
    # )
    # drive_base.straight(-200)

@mission("The Tree temporaAry mission,")
def mission_7():
    wait(500)
    drive_base.straight(90)
    drive_base.turn(40)
    drive_base.straight(740)
    drive_base.turn(-40)
    drive_base.settings(
        straight_speed=300,
        straight_acceleration=1000,
        turn_rate=400,
        turn_acceleration=500
    )
    drive_base.straight(150)
    drive_base.straight(-150)


def main():
    lbm.reset_angle(0)
    rbm.reset_angle(0)
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
