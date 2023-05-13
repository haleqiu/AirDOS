import numpy as np

class Pose:
    def __init__(self, time, SE3):
        self.time = time
        self.SE3  = SE3
    
    def L_transform(self, SE3, deltaTime=0) -> "Pose":
        return Pose(self.time + deltaTime, SE3 @ self.SE3)
    
    def R_transform(self, SE3, deltaTime=0) -> "Pose":
        return Pose(self.time + deltaTime, self.SE3 @ SE3)
    
    def __sub__(self, pose2: "Pose") -> "Pose":
        # Calculate self - pose2, return relative 
        # SE3 from self to pose2
        # time = self time - pose2 time
        return Pose(self.time - pose2.time, np.linalg.inv(pose2.SE3) @ self.SE3)

    def __lt__(self, pose2: "Pose") -> bool:
        return self.time < pose2.time

    def __gt__(self, pose2: "Pose") -> bool:
        return self.time > pose2.time

    def __str__(self) -> str:
        return f"Time:{str(self.time)}\nSE3:\n{str(self.SE3)}\n"
    
    def __repr__(self) -> str:
        return str(self)

