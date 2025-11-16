import turtle as t

class Space:

    def __init__(self, grid):
        self.grid = grid
        self.horizontal = len(grid[0])
        self.vertical = len(grid)
        self.mvsz = 20
        self.offset = [-self.horizontal//2*self.mvsz, self.vertical//2*self.mvsz]
        self.win = t.Screen()
        self.win.tracer(0)
        t.speed(10)
        t.pu()

    def draw_grid(self, start, end):
        for y in range(self.vertical):
            for x in range(self.horizontal):
                clr = "black" if self.grid[y][x] == 1 else 'red'
                t.goto(self.offset[0] + self.mvsz*x, self.offset[1] -self.mvsz*y)
                t.dot(10, clr)
        t.goto(self.offset[0] + self.mvsz*start[1], self.offset[1] -self.mvsz*start[0])
        t.dot(8, "green")
        t.goto(self.offset[0] + self.mvsz*end[1], self.offset[1] -self.mvsz*end[0])
        t.dot(8, "blue")
        self.win.update()
        self.win.tracer(1)


    def trace(self, start, moveset):
        t.pencolor("black")
        t.width(1)
        t.goto(self.offset[0]+start[1]*self.mvsz, self.offset[1]-start[0]*self.mvsz)
        t.pd()
        
        for move in range(len(moveset)):
            t.goto(self.offset[0]+moveset[move][1]*self.mvsz, self.offset[1]-moveset[move][0]*self.mvsz)
    
    def poly(self,poly):
        t.pencolor("red")
        t.width(4)
        t.goto(self.offset[0] + self.mvsz*poly[0][0], self.offset[1] -self.mvsz*poly[0][1])
        t.pd()
        for point in range(1,len(poly)):
            t.goto(self.offset[0] + self.mvsz*poly[point][0], self.offset[1] - self.mvsz*poly[point][1])
        t.goto(self.offset[0] + self.mvsz*poly[0][0], self.offset[1] -self.mvsz*poly[0][1])
        t.pu()
    
    def complete(self):
        t.done()
