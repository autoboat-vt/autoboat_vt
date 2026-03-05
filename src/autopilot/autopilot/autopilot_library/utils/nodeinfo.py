class NodeInfo:
    def __init__(self):
        # coordinates
        self.oldi = 0 
        self.oldj = 0
        self.f = float('inf') # total cost, default infinity
        self.g = float('inf') # heuristic cost, default infinity
        self.h = 0 # cumulative movement cost