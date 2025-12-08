import matplotlib
import shapely

class Inside:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    
    def check(self, polygon):
        '''
        path=matplotlib.path.Path(polygon)
        return path.contains_point([self.x,self.y]) or [self.x, self.y] in polygon
        '''
        poly=shapely.Polygon(polygon)
        poly=poly.buffer(2.0)
        pt=shapely.Point(self.x,self.y)
        return poly.contains(pt)