
import rclpy
import numpy as np

from math import pi, sin, cos, acos, atan2, sqrt, fmod, exp

# Grab the utilities
from project_updated.GeneratorNode      import *
#from TransformHelpers   import *
#from TrajectoryUtils    import *

# Grab the general fkin from HW5 P5.
#from KinematicChain     import KinematicChain
#from std_msgs.msg import Float64


#
#   Trajectory Class
#
class Trajectory():
    # Initialization.
    def __init__(self, node):
        
        self.q = np.radians(pi*np.zeros((30,1)).reshape((-1,1)))
        #Set up the kinematic chain object.
        #self.chain = KinematicChain(node, 'world', 'tip', self.jointnames())

        # Initialize the current/starting joint position.
        #self.q  = self.q0
        #(self.pd, self.Rd, self.Jv,self.Jw ) = self.chain.fkin(self.q0)


    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        joint_names = ['back_bkx','back_bky','back_bkz', 'l_arm_elx','l_arm_ely','l_arm_shx','l_arm_shz','l_arm_wrx','l_arm_wry','l_arm_wry2','l_leg_akx','l_leg_aky','l_leg_hpx','l_leg_hpy','l_leg_hpz','l_leg_kny','neck_ry', 'r_arm_elx','r_arm_ely','r_arm_shx','r_arm_shz','r_arm_wrx','r_arm_wry','r_arm_wry2','r_leg_akx','r_leg_aky','r_leg_hpx','r_leg_hpy','r_leg_hpz','r_leg_kny']
        return joint_names

    # Evaluate at the given time.  This was last called (dt) ago.
    def evaluate(self, t, dt):
        if(t<pi):
            q = (t*np.ones((30,1)).reshape((-1,1)))
            print(q)
        else:
            q = self.q
        qdot = np.radians(np.ones((30,1)).reshape((-1,1)))
        
        

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

