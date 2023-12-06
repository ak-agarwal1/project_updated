
import rclpy
import numpy as np

from math import pi, sin, cos, acos, atan2, sqrt, fmod, exp

# Grab the utilities
from project_updated.GeneratorNode          import *
from project_updated.TransformHelpers       import *
from project_updated.TrajectoryUtils        import *
from project_updated.CustomUtils            import *
from project_updated.GeneratedTrajectories  import *

# Grab the general fkin from HW5 P5.
from project_updated.KinematicChain         import KinematicChain


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

        self.pelvis_pxyz_wrt_world = pxyz(0,0,1)
        


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
        
        print(t)
        if(t<=3):
            (pd_lefthand,vd_lefthand,Rd_lefthand,wd_lefthand,
             pd_righthand,vd_righthand,Rd_righthand,wd_righthand) = get_hand_to_initial_pos(t,3,self.p_initial_righthand,self.p_initial_lefthand,self.R_initial_righthand,self.R_initial_lefthand)
            
            (pd_leftfoot,vd_leftfoot,Rd_leftfoot,wd_leftfoot) = (self.p_initial_leftfoot,np.zeros((3,1)),self.R_initial_leftfoot,np.zeros((3,1)))
            (pd_rightfoot,vd_rightfoot,Rd_rightfoot,wd_rightfoot) = (self.p_initial_rightfoot,np.zeros((3,1)),self.R_initial_rightfoot,np.zeros((3,1)))
            (pd_uppertorso,vd_uppertorso,Rd_uppertorso,wd_uppertorso) = (self.p_initial_uppertorso,np.zeros((3,1)),self.R_initial_uppertorso,np.zeros((3,1)))
            (pd_head,vd_head,Rd_head,wd_head) = (self.p_initial_head,np.zeros((3,1)),self.R_initial_head,np.zeros((3,1)))

            if(t>=2.99):
                (self.p_initial_lefthand,self.R_initial_lefthand) = (pd_lefthand,Rd_lefthand)
                (self.p_initial_righthand,self.R_initial_righthand) = (pd_righthand,Rd_righthand)

        elif(t>3 and t<=4):
            
            (pd_lefthand,vd_lefthand,Rd_lefthand,wd_lefthand) = hand_trajectory(t-3,1,self.p_initial_lefthand,Rotx(-pi/2),pxyz(0,0,0),exyz(0,0,1),-pi/6)
            (pd_righthand,vd_righthand,Rd_righthand,wd_righthand) = hand_trajectory(t-3,1,self.p_initial_righthand,Rotx(pi/2),pxyz(0,0,0),exyz(0,0,1),pi/6)

            (pd_uppertorso,vd_uppertorso,Rd_uppertorso,wd_uppertorso) = rotate_back(t-3,1,self.p_initial_uppertorso,self.R_initial_uppertorso,pxyz(0.1,0,0),pi/16)

            (pd_leftfoot,vd_leftfoot,Rd_leftfoot,wd_leftfoot) = (self.p_initial_leftfoot,np.zeros((3,1)),self.R_initial_leftfoot,np.zeros((3,1)))
            (pd_rightfoot,vd_rightfoot,Rd_rightfoot,wd_rightfoot) = (self.p_initial_rightfoot,np.zeros((3,1)),self.R_initial_rightfoot,np.zeros((3,1)))
            (pd_head,vd_head,Rd_head,wd_head) = (self.p_initial_head,np.zeros((3,1)),self.R_initial_head,np.zeros((3,1)))
            
            if(t>=3.99):
                (self.p_initial_lefthand,self.R_initial_lefthand) = (pd_lefthand,Rd_lefthand)
                (self.p_initial_righthand,self.R_initial_righthand) = (pd_righthand,Rd_righthand)
                (self.p_initial_uppertorso,self.R_initial_uppertorso) = (pd_uppertorso,Rd_uppertorso)

        elif(t>4 and t<=6):

            (pd_rightfoot,vd_rightfoot,Rd_rightfoot,wd_rightfoot) = injured_right_leg_move(t-4,2,self.p_initial_rightfoot,self.R_initial_rightfoot)
            
            (pd_lefthand,vd_lefthand,Rd_lefthand,wd_lefthand) = (self.pd_lefthand,np.zeros((3,1)),self.Rd_lefthand,np.zeros((3,1)))
            (pd_righthand,vd_righthand,Rd_righthand,wd_righthand) = (self.pd_righthand,np.zeros((3,1)),self.Rd_righthand,np.zeros((3,1)))
            (pd_uppertorso,vd_uppertorso,Rd_uppertorso,wd_uppertorso) = (self.pd_uppertorso,np.zeros((3,1)),self.Rd_uppertorso,np.zeros((3,1)))
            (pd_leftfoot,vd_leftfoot,Rd_leftfoot,wd_leftfoot) = (self.p_initial_leftfoot,np.zeros((3,1)),self.R_initial_leftfoot,np.zeros((3,1)))
            (pd_head,vd_head,Rd_head,wd_head) = (self.p_initial_head,np.zeros((3,1)),self.R_initial_head,np.zeros((3,1)))

            if(t>=5.99):
                (self.p_initial_rightfoot,self.R_initial_rightfoot) = (pd_rightfoot,Rd_rightfoot)
        
        elif(t>6 and t<=8):

            (pd_leftfoot,vd_leftfoot,Rd_leftfoot,wd_leftfoot,
             pd_rightfoot,vd_rightfoot,Rd_rightfoot,wd_rightfoot) = walk(t-6,2,self.p_initial_rightfoot,self.p_initial_leftfoot,self.Rd_rightfoot,self.R_initial_leftfoot)
              
            (pd_lefthand,vd_lefthand,Rd_lefthand,wd_lefthand) = hand_trajectory(t-6,2,self.p_initial_lefthand,Rotx(-pi/2)@ Rotz(-pi/6),pxyz(-0.4,0,0),exyz(0,0,1),pi/6)
            (pd_righthand,vd_righthand,Rd_righthand,wd_righthand) = hand_trajectory(t-6,2,self.p_initial_righthand,Rotx(pi/2)@ Rotz(pi/6),pxyz(-0.4,0,0),exyz(0,0,1),-pi/6)
            
            (pd_uppertorso,vd_uppertorso,Rd_uppertorso,wd_uppertorso) = rotate_back(t-6,2,self.p_initial_uppertorso,self.R_initial_uppertorso,pxyz(-0.1,0,0),-pi/16)
            
            (pd_head,vd_head,Rd_head,wd_head) = (self.p_initial_head,np.zeros((3,1)),self.R_initial_head,np.zeros((3,1)))

            if(t>7.99):
                (self.p_initial_rightfoot,self.R_initial_rightfoot) = (pd_rightfoot,Rd_rightfoot)
                (self.p_initial_lefttfoot,self.R_initial_leftfoot) = (pd_leftfoot,Rd_leftfoot)
                (self.p_initial_lefthand,self.R_initial_lefthand) = (pd_lefthand,Rd_lefthand)
                (self.p_initial_righthand,self.R_initial_righthand) = (pd_righthand,Rd_righthand)
                (self.p_initial_uppertorso,self.R_initial_uppertorso) = (pd_uppertorso,Rd_uppertorso)

        else:
            t_prime = (t-8)%4

            if(t_prime<=2):

                # Does not work right now, will probably have to get hands in position first, then add injured leg movement after
                (pd_rightfoot,vd_rightfoot,Rd_rightfoot,wd_rightfoot) = right_leg_only_move(t_prime,2,self.p_initial_rightfoot,self.R_initial_rightfoot)
            
                #(pd_lefthand,vd_lefthand,Rd_lefthand,wd_lefthand) = hand_trajectory(t-6,2,self.p_initial_lefthand,Rotx(pi/2)@ Rotz(pi/6),pxyz(0.1,0,0),exyz(0,0,1),pi/6)
                #(pd_righthand,vd_righthand,Rd_righthand,wd_righthand) = hand_trajectory(t-6,2,self.p_initial_righthand,Rotx(-pi/2)@ Rotz(-pi/6),pxyz(0.1,0,0),exyz(0,0,1),-pi/6)
                (pd_lefthand,vd_lefthand,Rd_lefthand,wd_lefthand) = hand_trajectory(t_prime,2,self.p_initial_lefthand,Rotx(-pi/2),pxyz(0.4,0,0),exyz(0,0,1),-pi/6)
                (pd_righthand,vd_righthand,Rd_righthand,wd_righthand) = hand_trajectory(t_prime,2,self.p_initial_righthand,Rotx(pi/2),pxyz(0.4,0,0),exyz(0,0,1),pi/6)

                (pd_uppertorso,vd_uppertorso,Rd_uppertorso,wd_uppertorso) = rotate_back(t_prime,2,self.p_initial_uppertorso,self.R_initial_uppertorso,pxyz(0.1,0,0),pi/16)

                (pd_leftfoot,vd_leftfoot,Rd_leftfoot,wd_leftfoot) = (self.p_initial_leftfoot,np.zeros((3,1)),self.R_initial_leftfoot,np.zeros((3,1)))
                (pd_head,vd_head,Rd_head,wd_head) = (self.p_initial_head,np.zeros((3,1)),self.R_initial_head,np.zeros((3,1)))

                if(t_prime>1.99):
                    (self.p_initial_rightfoot,self.R_initial_rightfoot) = (pd_rightfoot,Rd_rightfoot)
                    (self.p_initial_lefthand,self.R_initial_lefthand) = (pd_lefthand,Rd_lefthand)
                    (self.p_initial_righthand,self.R_initial_righthand) = (pd_righthand,Rd_righthand)
                    (self.p_initial_uppertorso,self.R_initial_uppertorso) = (pd_uppertorso,Rd_uppertorso)
        
            else:

                (pd_leftfoot,vd_leftfoot,Rd_leftfoot,wd_leftfoot,
                 pd_rightfoot,vd_rightfoot,Rd_rightfoot,wd_rightfoot) = walk(t_prime-2,2,self.p_initial_rightfoot,self.p_initial_leftfoot,self.Rd_rightfoot,self.R_initial_leftfoot)
              
                (pd_lefthand,vd_lefthand,Rd_lefthand,wd_lefthand) = hand_trajectory(t_prime-2,2,self.p_initial_lefthand,Rotx(-pi/2)@ Rotz(-pi/6),pxyz(-0.4,0,0),exyz(0,0,1),pi/6)
                (pd_righthand,vd_righthand,Rd_righthand,wd_righthand) = hand_trajectory(t_prime-2,2,self.p_initial_righthand,Rotx(pi/2)@ Rotz(pi/6),pxyz(-0.4,0,0),exyz(0,0,1),-pi/6)
            
                (pd_uppertorso,vd_uppertorso,Rd_uppertorso,wd_uppertorso) = rotate_back(t_prime-2,2,self.p_initial_uppertorso,self.R_initial_uppertorso,pxyz(-0.1,0,0),-pi/16)
            
                (pd_head,vd_head,Rd_head,wd_head) = (self.p_initial_head,np.zeros((3,1)),self.R_initial_head,np.zeros((3,1)))

                if(t_prime>3.99):
                    (self.p_initial_rightfoot,self.R_initial_rightfoot) = (pd_rightfoot,Rd_rightfoot)
                    (self.p_initial_lefttfoot,self.R_initial_leftfoot) = (pd_leftfoot,Rd_leftfoot)
                    (self.p_initial_lefthand,self.R_initial_lefthand) = (pd_lefthand,Rd_lefthand)
                    (self.p_initial_righthand,self.R_initial_righthand) = (pd_righthand,Rd_righthand)
                    (self.p_initial_uppertorso,self.R_initial_uppertorso) = (pd_uppertorso,Rd_uppertorso)
        


        ##############################################################################################
        #### NO NEED TO EDIT BEYOND THIS FOR evaluate() IF pd,Rd,vd,wd IS DEFINED FOR ALL 6 CHAINS ###
        #### SET UNUSED CHAIN pd AND Rd TO LAST VALUES STORED AND vd AND wd TO ZEROS               ###
        ##############################################################################################

        qlast = self.q
        (q_pelvis_leftfoot,q_pelvis_rightfoot,q_pelvis_uppertorso,q_uppertorso_head,q_uppertorso_lefthand,q_uppertorso_righthand) = decompose_into_indv_chains(qlast)

        #q_sec_right_kny = float(10*((pi)-q_pelvis_rightfoot[3]))
        #q_sec_right_foot = np.array([0,0,0,q_sec_right_kny,0,0]).reshape(-1,1)

        q_leftfoot,qdot_leftfoot = get_qdot_and_q_from_qlast(q_pelvis_leftfoot, self.pelvis_leftfoot_chain,self.pd_leftfoot,self.Rd_leftfoot, vd_leftfoot,wd_leftfoot,dt)
        q_rightfoot,qdot_rightfoot = get_qdot_and_q_from_qlast(q_pelvis_rightfoot, self.pelvis_rightfoot_chain,self.pd_rightfoot,self.Rd_rightfoot, vd_rightfoot,wd_rightfoot,dt)
        q_uppertorso,qdot_uppertorso = get_qdot_and_q_from_qlast(q_pelvis_uppertorso, self.pelvis_uppertorso_chain,self.pd_uppertorso,self.Rd_uppertorso, vd_uppertorso,wd_uppertorso,dt)
        q_head,qdot_head = get_qdot_and_q_from_qlast(q_uppertorso_head, self.uppertorso_head_chain,self.pd_head,self.Rd_head, vd_head,wd_head,dt)
        q_lefthand,qdot_lefthand = get_qdot_and_q_from_qlast(q_uppertorso_lefthand, self.uppertorso_lefthand_chain,self.pd_lefthand,self.Rd_lefthand,vd_lefthand,wd_lefthand,dt)
        q_righthand,qdot_righthand = get_qdot_and_q_from_qlast(q_uppertorso_righthand, self.uppertorso_righthand_chain,self.pd_righthand,self.Rd_righthand, vd_righthand,wd_righthand,dt)
        

        q = combine_indv_chain_to_q(q_leftfoot,q_rightfoot,q_uppertorso,q_head,q_lefthand,q_righthand)
        qdot = combine_indv_chain_to_q(qdot_leftfoot,qdot_rightfoot,qdot_uppertorso,qdot_head,qdot_lefthand,qdot_righthand)

        self.q = q

        (self.pd_leftfoot,self.Rd_leftfoot) = (pd_leftfoot,Rd_leftfoot)
        (self.pd_rightfoot,self.Rd_rightfoot) = (pd_rightfoot,Rd_rightfoot)
        (self.pd_uppertorso,self.Rd_uppertorso) = (pd_uppertorso,Rd_uppertorso)
        (self.pd_head,self.Rd_head) = (pd_head,Rd_head)
        (self.pd_lefthand,self.Rd_lefthand) = (pd_lefthand,Rd_lefthand)
        (self.pd_righthand,self.Rd_righthand) = (pd_righthand,Rd_righthand)



        return (q.flatten().tolist(), qdot.flatten().tolist())
    


    
    def pelvis_movement(self, t, dt):

        # Compute position/orientation of the pelvis (w.r.t. world).
        if(t<6):
            ppelvis = self.pelvis_pxyz_wrt_world
            Rpelvis = Reye()
            Tpelvis = T_from_Rp(Rpelvis, ppelvis)
        else:
            t_prime = (t-6)%4

            if(t_prime <= 2):
                self.pelvis_pxyz_wrt_world = self.pelvis_pxyz_wrt_world + pxyz(0.01*0.5,0,0)
            
            ppelvis = self.pelvis_pxyz_wrt_world
            Rpelvis = Reye()
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

