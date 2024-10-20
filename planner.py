# Type of planner
POINT_PLANNER=0; TRAJECTORY_PLANNER=1



class planner:
    def __init__(self, type_):

        self.type=type_

    
    def plan(self, goalPoint=[-1.0, -1.0]):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(goalPoint)
        
        elif self.type==TRAJECTORY_PLANNER:
            return self.trajectory_planner()


    def point_planner(self, goalPoint):
        x = goalPoint[0]
        y = goalPoint[1]
        return x, y

    # TODO Part 6: Implement the trajectories here
    def trajectory_planner(self):
        # return[[0.0, 0.0], [0.5, 1.0], [1.0,0.0]]
        return [[0.0, 0.0], [0.5, 0.25], [1.0, 1.0], [1.5, 2.25]]
        # return [[0,0], [0.5, 0.4621], [1,0.7616], [1.5, 0.9051], [2,0.9640], [2.5, 0.9866]]
        # the return should be a list of trajectory points: [ [x1,y1], ..., [xn,yn]]
        # return 

