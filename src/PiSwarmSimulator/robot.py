#    Pi Swarm Simulator is a simple graphical simulation environment for the Pi Swarm robots
#    Copyright (C) 2014 Alan Millard, Becky Naylor, Jon Timmis, University of York

#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.

#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.

#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <http://www.gnu.org/licenses/>.

# Import external libraries
import random, math
from framework import *

# Import simulator classes
from proxSensor import *

class Robot(object):
    
    def __init__(self, framework, robotid, position):
        
        # Reference to the simulation framework - allows access to global information about the world
        self.framework = framework
        
        self.robotid = robotid # Unique ID for each robot
                
        self.diameter = framework.calcSimSize(9.5) # Body diameter - 9.5cm
        self.IRSensRange = framework.calcSimSize(10) # IR sensor range - 10cm
        
        self.state = "forward" # Default state of finite state machine controller
        
        # Variables used when a robot is turning
        self.headingAngle = 0 # Turn to this angle (in radians)
        self.headingAngleAchieved = True # Flag used to tell when to stop turning
        
        self.sensor_fixtures = [] # Used to represent IR sensor cones
        self.illuminated = False # Is the robot illuminated by the IR beacon?
        
        # Choose initial heading at random
        start_heading = math.radians(random.randint(0, 359))
                
        # Construct body in the world
        robotshape = b2CircleShape(radius=(self.diameter / 2))
        self.fixtures = b2FixtureDef(shape=robotshape, density=1, friction=0.3, userData=self) # Pass the robot pointer to both fixture and body for use in collisions
        self.body = framework.world.CreateDynamicBody(position=position, angle=start_heading, fixtures=self.fixtures, linearDamping=5, angularDamping=5, userData=self)
    
        # Set up IR sensors at correct positions round the robot
        IRsensID = 0 # IR sensor ID
        IRposList = [15, 50, 90, 154, 206, 270, 310, 345] # Angles the IR sensors are placed at on the robot's body (in degrees)
        self.IRSensList = []
        for IRpos in IRposList:
            IRsens = ProxSensor(IRsensID, self.diameter / 2, self.IRSensRange, 30, IRpos, self.body.transform)
            self.IRSensList.append(IRsens)
            self.body.CreateFixture(IRsens.sensorfixture)
            IRsensID += 1

    # Robot controller (implemented in sub-classes)
    def drive(self):
        pass
    
    # Returns simulation time - if you want this in 'seconds' return: self.framework.clock * self.framework.ticklength
    def getSimulationTime(self):
        return self.framework.clock
    
    # Specify a new heading in degrees (relative to robot's current heading)
    def changeHeading(self, desiredHeading):
        heading = math.degrees(self.normaliseAngle(self.body.transform.R.angle))
        heading = heading + desiredHeading
        self.changeHeadingRad(math.radians(heading))
    
    # Specify a new heading in radians (absolute - not relative to robot's current heading)
    def changeHeadingRad(self, desiredHeading):
        
        # Get co-ordinates of desired heading
        x = -(math.cos(desiredHeading))
        y = -(math.sin(desiredHeading))
        
        # Set own parameters to the desired heading
        self.headingAngle = self.normaliseAngle(desiredHeading)
        self.headingAngleAchieved = False
    
    def driveForward(self):
        # Equal forward power from both wheels
        self.driveRightWheelForward()
        self.driveLeftWheelForward()
            
    def driveBackward(self):
        # Equal backward power from both wheels
        self.driveRightWheelBackward()
        self.driveLeftWheelBackward()
    
    # Turn while still moving
    def driveForwardLeft(self):
        self.driveRightWheelForward(0)
        self.driveLeftWheelForward(0.5)
        
    def driveForwardRight(self):
        self.driveRightWheelForward(0.5)
        self.driveLeftWheelForward(0)
        
    # Check whether the requested heading is met, if so then set flags to stop turning, if not set flags to keep turning
    def headingAchieved(self):
        
        permittedOffset = 1
        radOffset = math.radians(permittedOffset)
       
        # Get the current robot heading
        heading = self.normaliseAngle(self.body.transform.R.angle)
        
        desiredHeadingAngle = self.headingAngle
        lowestPermissable = self.normaliseAngle(desiredHeadingAngle - radOffset)
        highestPermissable = self.normaliseAngle(desiredHeadingAngle + radOffset)

        # Check heading against desired heading
        # Special case for crossing 0/359 degrees 
        if (lowestPermissable > highestPermissable): 
            if not((heading >= lowestPermissable) or (heading <= highestPermissable)):
                self.headingAngleAchieved = False
            else:
                self.headingAngleAchieved = True
        else:
            # Check if the heading is within the allowable range 
            if not(lowestPermissable <= heading <= highestPermissable):
                self.headingAngleAchieved = False
            else:
                self.headingAngleAchieved = True
        
        if self.headingAngleAchieved == True:
            # If it is achieved then clear the heading values
            self.headingAngle = 0
        
        return self.headingAngleAchieved
    
    # Turn to specified heading 
    def turnToHeading(self):
    
        # Check if we have reached the desired heading - if not continue to turn
        if self.headingAchieved() == False:

            # Set direction to turn the shortest way                  
            # When relative to the world it is the difference between current and desired headings
            heading = self.normaliseAngle(self.body.transform.R.angle)
            
            if (self.normaliseAngle(self.headingAngle - heading) >= math.pi) or (self.normaliseAngle(self.headingAngle - heading) <= 0):    
                self.direction = "right"
                self.turnClockwise()
            else:
                self.direction = "left"
                self.turnAntiClockwise()
                
    def headingToCoordinate(self, x, y):
        # Get the robot's current heading in degrees
        current_heading = math.degrees(self.normaliseAngle(self.body.transform.R.angle))
        
        # Calculate the angle to the coordinate
        heading_to_coordinate = math.degrees(self.angleToCoord((x, y)))
        
        # Return the difference (relative angle the robot must turn by to face coordinate)
        return heading_to_coordinate - current_heading
    
    # Pass in a coordinate to head to, return angle in degrees
    def angleToCoord(self, (x2, y2)):
        # Get current position
        (x1, y1) = self.body.position
        
        # Calculate the angle of the vector between current position and desired position
        angleToCoord = self.normaliseAngle(math.atan2((y1 - y2), (x1 - x2)))
        
        return angleToCoord
    
    def turnClockwise(self):
        self.driveRightWheelForward(0.5)
        self.driveLeftWheelBackward(0.5)
        
    def turnAntiClockwise(self):
        self.driveLeftWheelForward(0.5)
        self.driveRightWheelBackward(0.5)
    
    # Functions to control each wheel independently - allowing movement and turning of the robot
    def driveRightWheelForward(self, speed = 5):
        # Get robot heading vector from the perspective of the right wheel
        # Right wheel position is (0,-1), front of robot is (-1,0)
        f = b2Mul(self.body.transform.R,(-1.0,-1.0))
        
        # Adjust vector to speed 
        f[0] = speed*f[0]
        f[1] = speed*f[1]
        
        # Apply forward vector to wheel
        p = self.body.GetWorldPoint(localPoint=(0.0, -1.0))
        self.body.ApplyForce(f, p, True)
        
    def driveLeftWheelForward(self, speed = 5):
        # Get robot heading vector from the perspective of the left wheel
        # Left wheel position is (0,1), front of robot is (-1,0)
        f = b2Mul(self.body.transform.R,(-1.0,1.0))
        
        # Adjust vector to speed 
        f[0] = speed*f[0]
        f[1] = speed*f[1]
        
        # Apply forward vector to wheel
        p = self.body.GetWorldPoint(localPoint=(0.0, 1.0))
        self.body.ApplyForce(f, p, True)
            
    def driveRightWheelBackward(self, speed = 1):
        # Get robot reverse heading vector from the perspective of the right wheel
        # Right wheel position is (0,-1), front of robot is (-1,0)
        f = b2Mul(self.body.transform.R,(1.0,-1.0))
        
        # Adjust vector to speed 
        f[0] = speed*f[0]
        f[1] = speed*f[1]
        
        # Apply forward vector to wheel
        p = self.body.GetWorldPoint(localPoint=(0.0, -1.0))
        self.body.ApplyForce(f, p, True)
        
    def driveLeftWheelBackward(self, speed = 1):
        # Get robot reverse heading vector from the perspective of the left wheel
        # Left wheel position is (0,1), front of robot is (-1,0)
        f = b2Mul(self.body.transform.R,(1.0,1.0))
        
        # Adjust vector to speed 
        f[0] = speed*f[0]
        f[1] = speed*f[1]
        
        # Apply forward vector to wheel
        p = self.body.GetWorldPoint(localPoint=(0.0, 1.0))
        self.body.ApplyForce(f, p, True)
        
    # Issues where angle is negative or greater than 2pi radians (to decide whether to turn left or right)
    # Therefore normalise to a value between 0 and 2pi
    def normaliseAngle(self, angle): 
        if angle < 0:
            angle = (2*math.pi) + angle 
        elif angle > (2*math.pi):
            angle = angle - 2*math.pi
        return angle