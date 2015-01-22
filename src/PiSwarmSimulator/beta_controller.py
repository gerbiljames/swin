from robot import *

class BetaController(Robot):

    def __init__(self, framework, robotid, position):
        super(BetaController, self).__init__(framework, robotid, position)

        self.neighbours = []    # Robots within wireless communication range
        self.prevneighbours = []  # Neighbours at the previous time step

        self.beta = self.framework.settings.beta  # Beta threshold
        self.wireless_range = self.framework.calcSimSize(self.framework.settings.wireless_range)  # Wireless communication range

    def drive(self):

        # Calculate which robots are within wireless sensing range
        self.neighbours = []
        for anotherrobot in self.framework.robotlist:
            if self != anotherrobot:
                distance = self.framework.calcDistance(self.body.position, anotherrobot.body.position)
                if distance < self.wireless_range:
                    self.neighbours.append(anotherrobot)

        # Default state - robot moves straight ahead
        if self.state == "forward":

            self.driveForward()

            # If a communication link has been lost, perform coherence
            if len(self.neighbours) < len(self.prevneighbours):

                # Figure out which robot(s) we lost a connection to
                lost = set(self.prevneighbours) - set(self.neighbours)

                # By default, coherence is not required
                coherence_required = False

                # We may have lost multiple connections in a single time step. Iterate over each in turn.
                for robot in lost:

                    # Keep track of how many neighbours can still see the robot we lost a connection to
                    neighbours_still_connected = 0

                    # Iterate through our list of neighbours, and check their lists of neighbours
                    for neighbour in self.neighbours:

                        # Check whether the robot we lost a connection to can still be seen by this neighbour
                        if robot in neighbour.neighbours:
                            # If so, increment the number of neighbours that are still connected to the lost robot
                            neighbours_still_connected = neighbours_still_connected + 1

                    # If fewer than beta neighbours can still see the lost robot(s), then we should perform coherence                
                    if neighbours_still_connected < self.beta:
                        coherence_required = True

                    # If lost robot is illuminated then we should perform coherence
                    if robot.illuminated:
                        coherence_required = True

                # Perform coherence
                if coherence_required:
                    self.state = "turning"
                    self.changeHeading(180)

            # If obstacle detected on front sensors, transition to avoid state 
            elif self.IRSensList[0].contactObs or \
                            self.IRSensList[1].contactObs or \
                            self.IRSensList[6].contactObs or \
                            self.IRSensList[7].contactObs:
                self.state = "avoid"

        # Avoid obstacles         
        elif self.state == "avoid":

            # Turn left or right, depending on which of the front sensors are activated
            if self.IRSensList[0].contactObs or self.IRSensList[1].contactObs:
                self.driveForwardLeft()
            elif self.IRSensList[6].contactObs or self.IRSensList[7].contactObs:
                self.driveForwardRight()
            else: # Transition back to the forward state when we've finished turning
                self.state = "forward"

        elif self.state == "turning":

            # Keep turning until the desired heading is achieved
            if not self.headingAngleAchieved:
                self.turnToHeading()
            else:  # Transition back to the forward state when we've finished turning
                self.state = "forward"

        # Record the neighbours we currently have
        self.prevneighbours = self.neighbours