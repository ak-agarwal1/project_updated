
import rclpy
import numpy as np

from math import pi, sin, cos, acos, atan2, sqrt, fmod, exp

# Grab the utilities
from project_updated.GeneratorNode      import *
from project_updated.TransformHelpers   import *
from project_updated.TrajectoryUtils    import *

# Grab the general fkin from HW5 P5.
from project_updated.KinematicChain     import KinematicChain
#from std_msgs.msg import Float64


#
#   Trajectory Class
#
class Trajectory():
    # Initialization.
    def __init__(self, node):
        
        joint_names = self.jointnames()

        self.pelvis_leftfoot_chain = KinematicChain(node,'pelvis','l_foot',joint_names[0:6])
        self.pelvis_rightfoot_chain = KinematicChain(node,'pelvis','r_foot',joint_names[6:12])
        self.pelvis_uppertorso_chain = KinematicChain(node,'pelvis','utorso',joint_names[12:15])
        self.uppertorso_head = KinematicChain(node,'utorso','head',joint_names[15:16])
        self.uppertorso_lefthand = KinematicChain(node,'utorso','l_hand',joint_names[16:23])
        self.uppertorso_righthand = KinematicChain(node,'utorso','r_hand',joint_names[23:30])
        
        self.q0 = np.radians(pi*np.zeros((30,1)).reshape((-1,1)))
    

    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        joint_names = [
              'l_leg_hpz', 'l_leg_hpx', 'l_leg_hpy',
              'l_leg_kny',
              'l_leg_aky', 'l_leg_akx',

              'r_leg_hpz', 'r_leg_hpx', 'r_leg_hpy',
              'r_leg_kny',
              'r_leg_aky', 'r_leg_akx',

              'back_bkz', 'back_bky', 'back_bkx',

              'neck_ry',

              'l_arm_shz', 'l_arm_shx',
              'l_arm_ely', 'l_arm_elx',
              'l_arm_wry', 'l_arm_wrx', 'l_arm_wry2',

              'r_arm_shz', 'r_arm_shx',
              'r_arm_ely', 'r_arm_elx',
              'r_arm_wry', 'r_arm_wrx', 'r_arm_wry2']
        return joint_names


    # Evaluate at the given time.  This was last called (dt) ago.
    def evaluate(self, t, dt):
        if(t<pi):
            q = (2*t*np.ones((30,1)).reshape((-1,1)))
        else:
            q = self.q0
        qdot = 2*np.radians(np.ones((30,1)).reshape((-1,1)))
        
        

        return (q.flatten().tolist(), qdot.flatten().tolist())


#
#  Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Initialize the generator node for 100Hz udpates, using the above
    # Trajectory class.
    generator = GeneratorNode('generator', 100, Trajectory)

    # Spin, meaning keep running (taking care of the timer callbacks
    # and message passing), until interrupted or the trajectory ends.
    generator.spin()

    # Shutdown the node and ROS.
    generator.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

