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

# Arena perimeter polygon, currently there should be just one of these
class Arena():
    def __init__(self, world, xsize, ysize):
        
        self.xsize = xsize
        self.ysize = ysize

        # Centre the arena in the screen
        self.centrePoint = (0, ysize / 2)
        self.walls = world.CreateBody(position=self.centrePoint, userData=self)
        
        # List of corner positions to create edges
        self.corners = [ (-xsize/2, -ysize/2),
                                (-xsize/2 , ysize/2),
                                (xsize/2, ysize/2),
                                (xsize/2, -ysize/2),
                                (-xsize/2, -ysize/2) ]
        
        # Make vertices
        self.walls.CreateEdgeChain(self.corners)