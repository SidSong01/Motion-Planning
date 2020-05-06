# obstacles.py

#

# simple class for AABB obstacles
class BoxObstacle(object):

    def __init__(self, points):
        self.points = points # the 4 vertices of the box
        
        xs = []
        ys = []
        for p in self.points:
            xs.append(p[0])
            ys.append(p[1])
        
        #AABB representation
        self.x_min = min(xs)
        self.x_max = max(xs)
        self.y_min = min(ys) 
        self.y_max = max(ys)

        self.width = self.x_max - self.x_min
        self.height = self.y_max - self.y_min
