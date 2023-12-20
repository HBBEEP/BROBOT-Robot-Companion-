
class BrobotPos():
    def __init__(self):
        self.jointName = ['joint_1','joint_2','joint_3','joint_4']
        self.start_positions = [0.0,0.0,0.0,0.0]
        self.goal_positions = [0.0,0.0,0.0,0.0]
        self.setpoint_position = self.start_positions
        self.jointVelocity = [0.1,0.1,0.1,0.1]
        self.is_change_point = True
        self.index_step_pos = 0
        self.status = True
        self.pose = False
        self.countStatus = 0

        self.Pos=[[[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0]],
        [[0.0,0.0,-3.0,3.0],[0.0,0.0,3.0,-3.0],[0.1,0.1,0.5,0.5]],
        [[0.0,0.0,-3.0,-3.0],[0.0,0.0,3.0,3.0],[0.1,0.1,0.5,0.5]],
        [[0.0,0.0,0.0,0.0],[0.0,0.5,0.0,-3.0],[0.1,0.1,0.5,0.5]],
        [[0.0,-0.5,-3.0,-3.0],[0.0,0.5,3.0,3.0],[0.1,0.1,0.5,0.5]],
        [[2.0,0.0,-3.0,3.0],[-2.0,0.0,3.0,-3.0],[0.1,0.1,0.5,0.5]],
        [[2.0,0.0,-3.0,0.0],[-2.0,0.0,-3.0,0.0],[0.1,0.1,0.5,0.5]],
        [[-2.0,0.5,-0.5,-0.5],[2.0,0.5,-0.5,-0.5],[0.1,0.1,0.5,0.5]],
    ]
    def BasicPos(self):
        self.index_step_pos = self.index_step_pos + 1
        if self.index_step_pos >= 7:
            self.countStatus +=1
            self.index_step_pos = 0
            if self.is_change_point == False:
                self.is_change_point = True
                self.setpoint_position = self.start_positions
            else:
                self.is_change_point = False
                self.setpoint_position = self.goal_positions
        

    def managePos(self,action): 
        if self.countStatus>7:
            self.countStatus=0
            self.status = True
            self.pose = True

        self.start_positions = self.Pos[action][0]
        self.goal_positions = self.Pos[action][1]
        self.jointVelocity = self.Pos[action][2]
        self.BasicPos()
