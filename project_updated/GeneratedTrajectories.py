import rclpy
import numpy as np

from math import pi, sin, cos, acos, atan2, sqrt, fmod, exp

# Grab the utilities
from project_updated.TransformHelpers   import *
from project_updated.TrajectoryUtils    import *
from project_updated.CustomUtils        import *


def get_hand_to_initial_pos(t,T,p_initial_righthand,p_initial_lefthand,R_initial_righthand,R_initial_lefthand):

    (s0, s0dot) = goto(t, T, 0.0, 1.0)

    movementlh = np.array([0.2,-0.55,-0.55]).reshape(-1,1)
    movementrh = np.array([0.2,0.55,-0.55]).reshape(-1,1)

    rotation = pi/2

    pd_righthand = p_initial_righthand + (movementrh)*s0
    vd_righthand = movementrh * s0dot

    Rd_righthand = R_initial_righthand @ Rotx(rotation * s0)  
    wd_righthand = R_initial_righthand @ (ex() * (rotation * s0dot))

    pd_lefthand = p_initial_lefthand + (movementlh)*s0
    vd_lefthand = movementlh * s0dot

    Rd_lefthand = R_initial_lefthand @ Rotx(-rotation * s0)
    wd_lefthand = R_initial_lefthand @ (ex() * (-rotation * s0dot))

    return (pd_lefthand,vd_lefthand,Rd_lefthand,wd_lefthand,pd_righthand,vd_righthand,Rd_righthand,wd_righthand)



def hand_trajectory(t,T,p_initial,R_initital,movement,e,alpha):

    (s0, s0dot) = goto(t, T, 0.0, 1.0)
    #(s0,s0dot) = (abs(sin(pi*(t/2))),abs(cos(pi*(t/2))))

    pd = p_initial + (movement)*s0
    vd = (movement)*s0dot

    Rd= R_initital @ Rote(e,alpha*s0)
    wd = R_initital @ (e * (alpha * s0dot)) 

    return (pd,vd,Rd,wd)



def rotate_back(t,T,p_initial_uppertorso,R_initial_uppertorso,movement,y_alpha):

    (s0, s0dot) = goto(t, T, 0.0, 1.0)

    pd_uppertorso = p_initial_uppertorso +movement*s0
    vd_uppertorso = movement*s0dot

    Rd_uppertorso= R_initial_uppertorso @ Roty(y_alpha*s0)
    wd_uppertorso= R_initial_uppertorso @ (ey()*(y_alpha * s0dot))

    return (pd_uppertorso,vd_uppertorso,Rd_uppertorso,wd_uppertorso)




def injured_right_leg_move(t,T,p_initiaL_rightfoot,R_initial_rightfoot):

    (s0, s0dot) = goto(t, T, 0.0, 1.0)
    e = exyz(0,1,0)

    rotation = pi/4
    movement = np.array([0.38,0,0.12]).reshape(-1,1)

    pd_rightfoot = p_initiaL_rightfoot +movement*s0
    vd_rightfoot = movement*s0dot

    Rd_rightfoot = R_initial_rightfoot @ Rote(e,rotation*s0) 
    wd_rightfoot = R_initial_rightfoot @ (e * (rotation * s0dot))

    return (pd_rightfoot,vd_rightfoot,Rd_rightfoot,wd_rightfoot)



def right_leg_only_move(t,T,p_initiaL_rightfoot,R_initial_rightfoot):

    (s0, s0dot) = goto(t, T, 0.0, 1.0)
    e = exyz(0,1,0)

    rotation = 0
    movement = np.array([0.38,0,0.0]).reshape(-1,1)

    pd_rightfoot = p_initiaL_rightfoot +movement*s0
    vd_rightfoot = movement*s0dot

    Rd_rightfoot = R_initial_rightfoot @ Rote(e,rotation*s0) 
    wd_rightfoot = R_initial_rightfoot @ (e * (rotation * s0dot))

    return (pd_rightfoot,vd_rightfoot,Rd_rightfoot,wd_rightfoot)





def walk(t,T,p_initial_rightfoot,p_initial_leftfoot,R_initial_rightfoot,R_initial_leftfoot):

    (s0, s0dot) = goto(t, T, 0.0, 1.0)

    (s0_first,s0dot_first) = (sin(pi*(t/2)),cos(pi*(t/2)))

    movementlf = np.array([0,0,0]).reshape(-1,1)
    movementrf = np.array([-0.38,0,0]).reshape(-1,1)
    leftfoot_height = np.array([0,0,0.1]).reshape(-1,1)

    
    pd_rightfoot = p_initial_rightfoot + (movementrf)*s0
    vd_rightfoot = movementrf * s0dot

    Rd_rightfoot = R_initial_rightfoot
    wd_rightfoot = np.zeros((3,1))

    pd_leftfoot = p_initial_leftfoot + ((movementlf)/2 + leftfoot_height)*s0_first
    vd_leftfoot = ((movementlf)/2 + leftfoot_height)* s0dot

    Rd_leftfoot = R_initial_leftfoot
    wd_leftfoot = np.zeros((3,1))

    return (pd_leftfoot,vd_leftfoot,Rd_leftfoot,wd_leftfoot,pd_rightfoot,vd_rightfoot,Rd_rightfoot,wd_rightfoot)




