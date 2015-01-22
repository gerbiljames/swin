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
import os, random, math, datetime, re
from framework import *

# Import simulator classes
from arena import *
from robot import *
from proxSensor import *


from beta_controller import *
from omega_controller import *

import time, pygame

# Initialise and run the simulation
class runSim(Framework):
    
    name = "Pi Swarm Simulator"
    
    def __init__(self):
        super(runSim, self).__init__()
        
        if self.settings.taxis_algorithm not in ["beta", "omega"]:
            raise Exception("settings.taxis_algorithm must be either 'beta' or 'omega'")

        if self.settings.seed is None:
            # Seed RNG, use system time converted to int so it can easily be stored and rerun
            self.starttime = datetime.datetime.now()
            timestamp = str(self.starttime)
            seed = re.sub("\D", "", timestamp)
        else:
            seed = self.settings.seed

        random.seed(seed)
            
        if self.settings.headless:
            # Disable rendering
            os.environ["SDL_VIDEODRIVER"] = "dummy"
            self.screen = pygame.display.set_mode((1, 1))
            
        if self.settings.experiment or self.settings.log_advanced:

            experiment_name = ""
            if self.settings.log_advanced:
                experiment_name += "adv_"

            experiment_name += self.settings.taxis_algorithm

            if self.settings.taxis_algorithm == "beta":
                experiment_name += "_" + str(self.settings.beta)
            elif self.settings.taxis_algorithm == "omega":
                experiment_name += "_" + str(self.settings.omega)
            else:
                pass # Add your own code here
                
            # Construct log files and directory
            self.path = 'logs/' 
            if not os.path.exists(self.path):
                os.makedirs(self.path)
            
            self.logfile = open(self.path + experiment_name + '_' + seed + '.log', 'w')
                
        # Define simulation timing
        self.ticklength = 0.25 # Proportion of a second that each timestep is
        self.clock = 0
        
        # Delay between simulation steps (in seconds) to slow down physics for easier visualisation
        self.step_delay = 0.001
        
        # Set up the infrared beacon
        self.rays_visible = True
        self.beacon_position = b2Vec2(0, 1)
        beaconshape = b2CircleShape(radius=0.5)
        beaconfixture = b2FixtureDef(shape=beaconshape, userData=self)
        self.world.CreateStaticBody(position=self.beacon_position, angle=math.radians(270), fixtures=beaconfixture, userData=self)
        
        # Define world parameters
        self.world.gravity = (0.0, 0.0)
        self.unitsize = 10.0 # Number of cm one simulation unit represents
    
        # Define simulation values
        num_robots = self.settings.robots
        arena_x_size = 500 # Size in cm
        arena_y_size = 500
        
        # Set up the arena
        self.thearena = Arena(self.world, self.calcSimSize(arena_x_size), self.calcSimSize(arena_y_size))
        
        # Generate a swarm of robots
        self.robotlist = []
        
        for x in range(num_robots):
            
            # Calculate random initial position for each robot
            arenax = self.calcSimSize(arena_x_size)
            arenay = self.calcSimSize(arena_y_size)

            xmin = -(float(arenax)/2) / 6
            xmax = (float(arenax)/2) / 6
            ymin = 0
            ymax = arenay / 6
        
            (xpos, ypos) = (random.uniform(xmin, xmax), random.uniform(ymin, ymax) + ymax * 4.5)
            
            if self.settings.taxis_algorithm == "beta":
                currentRobot = BetaController(self, x, b2Vec2(xpos, ypos))
            else: # self.settings.taxis_algorithm == "omega":
                currentRobot = OmegaController(self, x, b2Vec2(xpos, ypos))
            
            self.robotlist.append(currentRobot)

    # Carry out these actions at each timestep
    def Step(self, settings):
        super(runSim, self).Step(settings)
        
        for therobot in self.robotlist:
            
            # Cast ray from the beacon to each robot, to check for line-of-sight 
            callback = RayCastClosestCallback()            
            self.world.RayCast(callback, self.beacon_position, therobot.body.position)
            
            # Update illumination status based on raycast result
            if callback.hit:
                if callback.fixture.body.position == therobot.body.position:
                    therobot.illuminated = True
                else:
                    therobot.illuminated = False
            
            # Drive robots
            therobot.drive()
        
        # Output to log file
        if self.settings.experiment or self.settings.log_advanced:

            if not self.settings.log_advanced:
                # Output the simulation time and distance of swarm centroid from beacon
                centroid = self.calculate_swarm_centroid()
                distance = self.calccmSize(self.calcDistance(centroid, self.beacon_position))
                outputlist = [str(self.clock * self.ticklength), str(distance)]

            else:
                # output simulation time, distance of swarm centroid from beacon and ...
                centroid = self.calculate_swarm_centroid()
                beacon_distance = self.calccmSize(self.calcDistance(centroid, self.beacon_position))
                avg_distance_from_centroid = self.calccmSize(self.calculate_mean_distance_from_swarm_centroid())
                lost_robots = self.num_lost_robots()
                outputlist = [str(self.clock * self.ticklength), str(beacon_distance), str(avg_distance_from_centroid), str(lost_robots)]

            outputstring = ",".join([str(x) for x in outputlist])
            self.logfile.write(outputstring + "\n")
            # End the simulation after a fixed number of iterations
            if self.clock > 50000:
                self.QuitPygame()
                 
        # Increment simulation clock
        self.clock += 1
        
        if not self.settings.headless:
            time.sleep(self.step_delay) # Slow simulation down for easier visualisation

    def Draw(self):
        super(runSim, self).Draw()
         
        for robot in self.robotlist:
            
            # Draw beacon rays to illuminated robots
            if self.rays_visible and robot.illuminated:
                self.renderer.DrawSegment(self.renderer.to_screen(self.beacon_position), self.renderer.to_screen(robot.body.position), b2Color(1, 1, 0))
            
            # Colour robot sensors            
            for sensor in robot.IRSensList:                                 
                shape = sensor.sensorshape
                if sensor.contactObs == True:
                    self.colourPolygon(robot.body.transform, shape, b2Color(1.2,0,0))
                else:
                    self.colourPolygon(robot.body.transform, shape, b2Color(1.0,0.5,1.2)) 
        
        # Colour the robots blue       
        self.colourShapes([x for x in self.robotlist if x.illuminated == False], b2Color(0.0,1.5,1.5))
        self.colourShapes([x for x in self.robotlist if x.illuminated == True], b2Color(1.5,1.0,0.0))
        
        # Print robot IDs
        for robot in self.robotlist:
            # Get the position of the robots
            (xpos, ypos) = robot.body.position
            
            # Convert the position to screen co-ords
            (xpos, ypos) = self.renderer.to_screen((xpos, ypos))

            self.DrawStringAt(xpos-1, ypos-1, str(robot.robotid), color=(0.0,0.0,0.0))
                        
        # Redraw the arena
        centre = self.thearena.centrePoint
        arena_vertices = [self.renderer.to_screen((centre[0]+x[0], centre[1]+x[1])) for x in self.thearena.corners]
        self.renderer.DrawPolygon(arena_vertices,b2Color(1.0,1.0,1.0))
        
        # Print simulation time elapsed in the corner of the screen    
        self.Print("Time: %f s" % (self.ticklength * self.clock), (255,255,255))
        self.Print("Step delay: %f s" % (self.step_delay), (255,255,255))  

    # Add colours to simulation objects
    def colourShapes(self, object_array, colour):
        
        # If a single object is provided then make it a list
        if not(isinstance(object_array, list)):
            object_array = [object_array]    
        
        for shapeToColour in (object_array):
        
            # Get the shape position from body
            shape = shapeToColour.fixtures.shape
        
            # If the shape is a polygon use polygon functions
            if isinstance(shape, b2PolygonShape):
                self.colourPolygon(shapeToColour.body.transform, shape.vertices, colour) 
        
            # If the shape is a circle use circle functions
            if isinstance(shape, b2CircleShape):
                
                # Offset by xpos and ypos
                position_vect = shapeToColour.body.position
                
                transform = shapeToColour.body.transform
                
                # Calculate the axis line on the robot (this is the method used in b2world.cpp)
                axis = b2Mul(transform.R, (1.0,0.0))
                
                # Convert the position to screen co-ords
                position_vect = self.renderer.to_screen(position_vect)
            
                # Draw circle
                self.renderer.DrawSolidCircle(position_vect, shape.radius, axis, colour)
    
    # Draw solid polygon of given shape and colour at (x,y) shape pos            
    def colourPolygon(self, shapepos, vertices, colour):
        # Offset by xpos and ypos and angle
        position_vect = [(shapepos*v) for v in vertices]
            
        # Convert the position to screen co-ords
        position_vect = map(self.renderer.to_screen, position_vect)
            
        # Draw polygon
        self.renderer.DrawSolidPolygon(position_vect, colour)

    # Check for contact between fixtures
    def BeginContact(self, contact):
        super(runSim, self).BeginContact(contact)
        
        # Get the two objects that have collided
        fixtureA = contact.fixtureA.userData
        fixtureB = contact.fixtureB.userData
        bodyA = contact.fixtureA.body.userData 
        bodyB = contact.fixtureB.body.userData
        
        # Update IR sensor flag to indicate that an obstacle has been detected
        if isinstance(fixtureA, ProxSensor) and not isinstance(fixtureB, ProxSensor):
            fixtureA.contactObs = True
            if isinstance(fixtureB, Robot):
                fixtureA.contactDistance = self.calcDistance(fixtureA.robottransform.position, fixtureB.body.position)
                fixtureA.contactDistance = fixtureA.contactDistance - fixtureB.diameter # Subtract diameter of a robot (same as radius of both the robots combined) 
        elif isinstance(fixtureB, ProxSensor) and not isinstance(fixtureB, ProxSensor):
            fixtureB.contactObs = True
            if isinstance(fixtureA, Robot):
                fixtureB.contactDistance = self.calcDistance(fixtureB.robottransform.position, fixtureA.body.position)
                fixtureB.contactDistance = fixtureB.contactDistance - fixtureA.diameter # Subtract diameter of a robot (same as radius of both the robots combined)
        
    def EndContact(self, contact):
        super(runSim, self).EndContact(contact)
        
        # Get the two objects that have stopped colliding
        fixtureA = contact.fixtureA.userData
        fixtureB = contact.fixtureB.userData
        bodyA = contact.fixtureA.body.userData 
        bodyB = contact.fixtureB.body.userData

        # Update IR sensor flag to indicate that an obstacle is no longer detected                
        if isinstance(fixtureA, ProxSensor):
            fixtureA.contactObs = False                
        elif isinstance(fixtureB, ProxSensor):
            fixtureB.contactObs = False
    
    # Check for keyboard input
    def Keyboard(self, key):
        if key == Keys.K_p:
           self.step_delay *= 0.1
        elif key == Keys.K_o:
           self.step_delay /= 0.1
        elif key == Keys.K_b:
           self.rays_visible = not self.rays_visible
    
    # From the real world size in cm calculate the size in simulation units
    def calcSimSize(self, sizeInCm):        
        sizeInUnits = (1 / float(self.unitsize)) * sizeInCm
        return sizeInUnits
 
    # From the size in simulation units calculate real world size in cm
    def calccmSize(self, sizeinUnits):
        sizeIncm = sizeinUnits * float(self.unitsize)
        return sizeIncm
    
    # Calculate Euclidean distance between two points
    def calcDistance(self, point_a, point_b):
        return math.sqrt(math.pow(point_a[0] - point_b[0], 2) + math.pow(point_a[1] - point_b[1], 2))

    def calculate_swarm_centroid(self):

        xpos = 0
        ypos = 0

        for another_robot in self.robotlist:
            xpos += another_robot.body.position.x
            ypos += another_robot.body.position.y

        num_robots = len(self.robotlist)

        xpos /= num_robots
        ypos /= num_robots

        return b2Vec2(xpos, ypos)

    def calculate_mean_distance_from_swarm_centroid(self):

        centroid = self.calculate_swarm_centroid()

        total_distance = 0

        for robot in self.robotlist:
            robot_position = b2Vec2(robot.body.position.x, robot.body.position.y)

            total_distance += self.calcDistance(robot_position, centroid)

        return total_distance / len(self.robotlist)

    def num_lost_robots(self):

        lost_robots = 0

        for robot in self.robotlist:

            neighbours = 0

            for another_robot in self.robotlist:

                if robot != another_robot:

                    neighbours += 1

            if neighbours == 0:

                lost_robots += 1

        return lost_robots


# Raycast class modified from pybox2D raycasting example code 
class RayCastClosestCallback(b2RayCastCallback):
    """This callback finds the closest hit"""
    def __repr__(self): return 'Closest hit'
    def __init__(self, **kwargs):
        b2RayCastCallback.__init__(self, **kwargs)
        self.fixture=None
        self.hit=False

    # Called for each fixture found in the query. You control how the ray proceeds
    # by returning a float that indicates the fractional length of the ray. By returning
    # 0, you set the ray length to zero. By returning the current fraction, you proceed
    # to find the closest point. By returning 1, you continue with the original ray
    # clipping. By returning -1, you will filter out the current fixture (the ray
    # will not hit it).
    def ReportFixture(self, fixture, point, normal, fraction):
        self.hit=True
        self.fixture=fixture
        self.point=b2Vec2(point)
        self.normal=b2Vec2(normal)
        # You will get this error: "TypeError: Swig director type mismatch in output value of type 'float32'"
        # without returning a value
        if fixture.sensor == True:
            self.hit = False
            return -1 # Ignore sensor fixtures
        else:
            return fraction


# Run simulation
if __name__ == '__main__': main(runSim)