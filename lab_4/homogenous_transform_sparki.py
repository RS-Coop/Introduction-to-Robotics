import numpy as np


class homogenous_transform:
    def __init__(self, robotFrame2world, ultrasonicFrame2Robot, theta1 , theta2 , ultrasonicPoint):
        # self.world_frame = np.array(world_frame) # [0,0,0]
        self.robotFrame2World = np.array(robotFrame2World) # [x,y,z]'
        self.ultrasonicFrame2Robot = np.array(ultrasonicFrame2Robot) # [x,y,z]'
        self.theta1 = theta1
        self.theta2 = theta2
        self.ultrasonicPoint = np.array(ultrasonicPoint) # remember numpy array [x,y, z]'

        self.R_Robot2World = self.get_rotationMatrix(theta1) # in radians
        self.R_Ultrasonic2Robot = self.get_rotationMatrix(theta2) # in radians

    def get_rotationMatrix(self,yaw):
        rotation = np.array([
                            [np.cos(yaw), -np.sin(yaw),    0   ],
                            [np.sin(yaw), np.cos(yaw),     0   ],
                            [      0    ,     0      ,     1   ]
        ])
        return rotation


    def get_ultrasonic2World(self, ultrasonic_p = None):
        '''
        Q_world = Rot_from_robot2World * Rot_sensor2Robot * sensor_p + rot_robot2world * frame_senor_in_robot + frame_robot_in_world

        '''
        if ultrasonic_p is None:
            ultrasonic_p = self.ultrasonicPoint

        result_rotation = np.dot(self.R_Robot2World, self.R_Ultrasonic2Robot)

        ultrasonic_p2world = np.dot(result_rotation, ultrasonic_p)

        result_frame_ultrasonic2World = np.dot(self.R_Robot2World, self.ultrasonicFrame2Robot)

        total_result = np.add(ultrasonic_p2world, result_frame_ultrasonic2World)

        total_result = np.add(total_result, self.robotFrame2World)

        return total_result

if __name__  == '__main__':
    robotFrame2World = np.array([[0],[0], [0]])
    ultrasonicFrame2Robot = np.array([[1],[0],[0]])
    theta1 = 0
    theta2 = 0
    ultrasonic_p = np.array([[0.1],[0],[0]]) # 10 cm in x coordinate

    obj = homogenous_transform(robotFrame2World,ultrasonicFrame2Robot, theta1, theta2, ultrasonic_p)

    print(obj.R_Robot2World)
    print(obj.R_Ultrasonic2Robot)

    print(obj.get_ultrasonic2World())
