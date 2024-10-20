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
        return [[0.0,0.000],[0.1,0.0997],[0.2,0.1974],[0.3,0.2913],[0.4,0.3799],[0.5,0.4621],[0.6,0.5370],[0.7,0.6044],[0.8,0.6640],[0.9,0.7163],[1.0,0.7616],[1.1,0.8005],[1.2,0.8337],[1.3,0.8620],[1.4,0.8860],[1.5,0.9051],[1.6,0.9202],[1.7,0.9324],[1.8,0.9420],[1.9,0.9495],[2.0,0.9640],[2.1,0.9698],[2.2,0.9746],[2.3,0.9786],[2.4,0.9821],[2.5,0.9866]]
        # return [[0.0,0.00],[0.1,0.01],[0.2,0.04],[0.3,0.09],[0.4,0.16],[0.5,0.25],[0.6,0.36],[0.7,0.49],[0.8,0.64],[0.9,0.81],[1.0,1.00],[1.1,1.21],[1.2,1.44],[1.3,1.69],[1.4,1.96],[1.5,2.25]]


