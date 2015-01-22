from robot import *


class OmegaController(Robot):

    def __init__(self, framework, robotid, position):

        super(OmegaController, self).__init__(framework, robotid, position)

        # timer is initialized to zero and omega is converted to ticks
        self.timer = 0
        self.omega_ticks = self.framework.settings.omega / self.framework.ticklength

        #  wireless_range is set but not used (it is also converted to sim units)
        self.wireless_range = self.framework.calcSimSize(self.framework.settings.wireless_range)

        #  we set the standard range to half the IR sensor range, full IR sensor range used when illuminated
        self.srange = self.IRSensRange / 2

    def drive(self):

        # increment timer by 1 tick
        self.timer += 1

        if self.state == "forward":

            self.driveForward()

            # if obstacle detected on front sensors, avoid it!
            if self.is_front_obstacle_detected():
                self.state = "avoid"

            #  check to see if we need to cohere due to timer
            elif self.timer > self.omega_ticks:
                self.state = "turning"

                centroid = self.calculate_swarm_centroid()

                self.changeHeading(self.headingToCoordinate(centroid.x, centroid.y))

        # avoid obstacles
        elif self.state == "avoid":

            # Turn left or right, depending on which of the front sensors are activated
            # This introduces a bias to turn left, but shouldn't affect the algorithm overall
            if self.is_sensor_active(0) or self.is_sensor_active(1):
                self.driveForwardLeft()

            elif self.is_sensor_active(6) or self.is_sensor_active(7):
                self.driveForwardRight()

            else:  # Transition back to the forward state when we've finished turning
                self.state = "forward"
                # set timer to zero after avoiding a robot
                self.timer = 0

        elif self.state == "turning":

            # keep turning until the desired heading is achieved
            if not self.headingAngleAchieved:
                self.turnToHeading()
            else:  # Transition back to the forward state when we've finished turning
                self.state = "forward"

                # reset timer to zero as we've finished turning
                self.timer = 0

    #  calculates the centroid of the swarm excluding this robot, this function uses global information and does not
    #  account for any kind of sensor limitations
    def calculate_swarm_centroid(self):

        xpos = 0
        ypos = 0

        #  iterate through all other robots and collect positions
        for another_robot in self.framework.robotlist:
            if self != another_robot:
                xpos += another_robot.body.position.x
                ypos += another_robot.body.position.y

        #  find the number of robots in the main swarm
        num_robots = len(self.framework.robotlist) - 1

        # calculate the mean x position and y position
        xpos /= num_robots
        ypos /= num_robots

        return Box2D.b2Vec2(xpos, ypos)

    #  checks to see if a specific IR sensor is active, this accounts for illumination
    def is_sensor_active(self, sensor_id):
        if self.illuminated:
            return self.IRSensList[sensor_id].contactObs
        else:
            if self.IRSensList[sensor_id].contactObs:
                return self.IRSensList[sensor_id].contactDistance < self.srange
            else:
                return False

    #  checks to see if there is an obstacle in front of the robot using the IR sensors, this accounts for illumination
    def is_front_obstacle_detected(self):
        for sensor_id in [0, 1, 6, 7]:
            if self.is_sensor_active(sensor_id):
                return True

        return False