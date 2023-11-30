
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


#from project_updated.pirouette          import *

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


        ################### Initial Positions and Rotations for tips of all 6 chains ################

        #evaluate inital condition for left foot chain
        (self.pd_leftfoot, self.Rd_leftfoot, _,_) = self.pelvis_leftfoot_chain.fkin(self.q[0:6].reshape((-1,1)))
        self.p_initial_leftfoot = self.pd_leftfoot
        self.R_initial_leftfoot = self.Rd_leftfoot

        #evaluate inital condition for right foot chain
        (self.pd_rightfoot, self.Rd_rightfoot, _,_) = self.pelvis_rightfoot_chain.fkin(self.q[6:12].reshape((-1,1)))
        self.p_initial_rightfoot = self.pd_rightfoot
        self.R_initial_rightfoot = self.Rd_rightfoot

        #evaluate inital condition for upper torso chain
        (self.pd_uppertorso, self.Rd_uppertorso, _,_) = self.pelvis_uppertorso_chain.fkin(self.q[12:15].reshape((-1,1)))
        self.p_initial_uppertorso = self.pd_uppertorso
        self.R_initial_uppertorso = self.Rd_uppertorso

        #evaluate inital condition for head chain
        (self.pd_head, self.Rd_head, _,_) = self.uppertorso_head_chain.fkin(self.q[15:16].reshape((-1,1)))
        self.p_initial_head = self.pd_head
        self.R_initial_head = self.Rd_head

        #evaluate inital condition for left hand chain
        (self.pd_lefthand, self.Rd_lefthand, _,_) = self.uppertorso_lefthand_chain.fkin(self.q[16:23].reshape((-1,1)))
        self.p_initial_lefthand = self.pd_lefthand
        self.R_initial_lefthand = self.Rd_lefthand

        #evaluate inital condition for right hand chain
        (self.pd_righthand, self.Rd_righthand, _,_) = self.uppertorso_righthand_chain.fkin(self.q[23:30].reshape((-1,1)))
        self.p_initial_righthand = self.pd_righthand
        self.R_initial_righthand = self.Rd_righthand

        ##############################################################################################
        


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
        

        if(t<3):
            (s0, s0dot) = goto(t, 3.0, 0.0, 1.0)

            movementlh = np.array([0.4,0,0.1]).reshape(-1,1)
            movementrh = np.array([0.4,0,0.1]).reshape(-1,1)

            pd_rightfoot = self.p_initial_rightfoot + (movementrh)*s0
            vd_rightfoot = movementrh * s0dot

            Rd_rightfoot = Roty(-pi/2 * s0)
            wd_rightfoot = ey() * (-pi/2 * s0dot)

            pd_leftfoot = self.p_initial_leftfoot + (movementlh)*s0
            vd_leftfoot = movementlh * s0dot

            Rd_leftfoot = Roty(-pi/2 * s0)
            wd_leftfoot = ey() * (-pi/2 * s0dot)
        else:
            return(None)


        qlast = self.q
        (q_pelvis_leftfoot,q_pelvis_rightfoot,q_pelvis_uppertorso,q_uppertorso_head,q_uppertorso_lefthand,q_uppertorso_righthand) = decompose_into_indv_chains(qlast)

        
        q_rf,qdot_rf = get_qdot_and_q_from_qlast(q_pelvis_rightfoot, self.pelvis_rightfoot_chain,self.pd_rightfoot,self.Rd_rightfoot, vd_rightfoot,wd_rightfoot,dt)
        q_lf,qdot_lf = get_qdot_and_q_from_qlast(q_pelvis_leftfoot, self.pelvis_leftfoot_chain,self.pd_leftfoot,self.Rd_leftfoot, vd_leftfoot,wd_leftfoot,dt)


        q = combine_indv_chain_to_q(q_lf,q_rf,q_pelvis_uppertorso,q_uppertorso_head,q_uppertorso_lefthand,q_uppertorso_righthand)
        qdot = combine_indv_chain_to_q(qdot_lf,qdot_rf,q_pelvis_uppertorso,q_uppertorso_head,q_uppertorso_lefthand,q_uppertorso_righthand)

        self.q = q
        self.pd_rightfoot = pd_rightfoot
        self.Rd_rightfoot = Rd_rightfoot
        self.pd_leftfoot = pd_leftfoot
        self.Rd_leftfoot = Rd_leftfoot


        return (q.flatten().tolist(), qdot.flatten().tolist())
    


    
    def pelvis_movement(self, t, dt):

        # Compute position/orientation of the pelvis (w.r.t. world).
        ppelvis = pxyz(0.0, 0.5, 1.5 + 0.5 * np.sin(t/2))
        Rpelvis = Rotz(np.sin(t))
        Tpelvis = T_from_Rp(Rpelvis, ppelvis)

        return Tpelvis


#
#  Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Initialize the generator node for 100Hz udpates, using the above
    # Trajectory class.
    generator = GeneratorNode('generator', 100, Trajectory)
    #node = DemoNode('pirouette',100)

    # Spin, meaning keep running (taking care of the timer callbacks
    # and message passing), until interrupted or the trajectory ends.
    #rclpy.spin(node)
    generator.spin()
    

    # Shutdown the node and ROS.
    generator.shutdown()

if __name__ == "__main__":
    main()

