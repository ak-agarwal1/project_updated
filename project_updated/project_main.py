
import rclpy
import numpy as np

from math import pi, sin, cos, acos, atan2, sqrt, fmod, exp

# Grab the utilities
from project_updated.GeneratorNode      import *
from project_updated.TransformHelpers   import *
from project_updated.TrajectoryUtils    import *
from project_updated.CustomUtils        import *

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
        self.uppertorso_head_chain = KinematicChain(node,'utorso','head',joint_names[15:16])
        self.uppertorso_lefthand_chain = KinematicChain(node,'utorso','l_hand',joint_names[16:23])
        self.uppertorso_righthand_chain = KinematicChain(node,'utorso','r_hand',joint_names[23:30])
        
        #Initial Condition for all joints
        self.q0 = np.radians(np.zeros((30,1)).reshape((-1,1)))

        #Update from each eval, start from q0
        self.q  = self.q0

        #evaluate inital condition for left hand chain
        (self.pd, self.Rd, self.Jv,self.Jw ) = self.uppertorso_lefthand_chain.fkin(self.q[16:23].reshape((-1,1)))
        self.p_in = self.pd
        self.R_in = self.Rd

        self.pfinal = self.p_in + np.array([0.4,-0.4,0.2]).reshape((-1,1))


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
        
        if(t<2):
            (s0, s0dot) = goto(t, 2.0, 0.0, 1.0)

            pd = self.p_in + (self.pfinal - self.p_in)*s0
            vd = self.pfinal * s0dot

            Rd = Rotz(-pi/2 * s0)
            wd = ez() * (-pi/2 * s0dot)
        else:
            return(None)




        qlast = self.q
        (q_pelvis_leftfoot,q_pelvis_rightfoot,q_pelvis_uppertorso,q_uppertorso_head,q_uppertorso_lefthand,q_uppertorso_righthand) = get_indv_chain_q_from_full_q(qlast)

        
        q_lh,qdot_lh = get_qdot_from_qlast(q_uppertorso_lefthand, self.uppertorso_lefthand_chain,self.pd,self.Rd, vd,wd,dt)

        q = np.append(np.zeros(16),q_lh)
        q = np.append(q,np.zeros(7))

        qdot = np.append(np.zeros(16),qdot_lh)
        qdot = np.append(q,np.zeros(7))

        self.q = q
        self.pd = pd
        self.Rd = Rd




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

