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
import math
from framework import *

class ProxSensor:
        # Each sensor on a robot has a unique ID, a field of view calculated using the radius and sensor apeture and a position in degrees around the edge of the robot
        # For the distance sensing we also need the robot position and rotation (robottransform)
        def __init__(self, sensid, robotRadius, sensorRange, sensApeture, position, robottransform):
            
            # Sensor properties
            self.proxid = sensid 
            self.contactObs = False     # If an obstacle is in this sensor's range then true
            self.contactDistance = 0
            
            # Each sensor on a robot has a unique ID, a field of view calculated using the radius and sensor apeture and a position in degrees around the edge of the robot
            self.robotRadius = robotRadius
            self.radpos = math.radians(position)
            self.robottransform = robottransform
            
            # Calculate extent of the sensor
            self.radius = robotRadius + sensorRange
            
            # Use 10 points around the edge of the cone, start in the centre of the robot
            # TODO: Move IR sensors start coordinate to edge of robot, not the centre
            num_vertices = 10
            vertices = [(0,0)] 
            
            # Calculate cone of perception - has length set by radius and arc by sensApeture
            for i in range(0,num_vertices+1):
                # i/num_vert calculates current vertex position, the -0.5 centres the cone about the position
                # position rotates the cone to the correct angle
                angle = (((float(i)/num_vertices)-0.5)*(math.radians(sensApeture)) - math.radians(position)) 
                # Get the angles of the two edges of the sensor
                if i == 0:
                    self.leftangle = angle
                if i == num_vertices:
                    self.rightangle = angle
                
                vertices.append((-(self.radius * math.cos(angle)), -(self.radius * math.sin(angle))))

            self.leftcoord = vertices[1]
            self.rightcoord = vertices[num_vertices+1]
            
            # Return fixture with correct polygon shape
            self.sensorshape=b2PolygonShape(vertices=vertices)
            self.sensorfixture = b2FixtureDef(shape=self.sensorshape, isSensor=True, userData=self)