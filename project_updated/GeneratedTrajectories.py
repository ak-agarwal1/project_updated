import rclpy
import numpy as np

from math import pi, sin, cos, acos, atan2, sqrt, fmod, exp

# Grab the utilities
from project_updated.TransformHelpers   import *
from project_updated.TrajectoryUtils    import *
from project_updated.CustomUtils        import *


def get_hand_to_initial_pos(t,T,p_initial_righthand,p_initial_lefthand):

    (s0, s0dot) = goto(t, T, 0.0, 1.0)

    movementlh = np.array([0.2,-0.55,-0.55]).reshape(-1,1)
    movementrh = np.array([0.2,0.55,-0.55]).reshape(-1,1)

    pd_righthand = p_initial_righthand + (movementrh)*s0
    vd_righthand = movementrh * s0dot

    Rd_righthand = Rotx(pi/2 * s0)
    wd_righthand = ex() * (pi/2 * s0dot)

    pd_lefthand = p_initial_lefthand + (movementlh)*s0
    vd_lefthand = movementlh * s0dot

    Rd_lefthand = Rotx(-pi/2 * s0)
    wd_lefthand = ex() * (-pi/2 * s0dot)

    return (pd_lefthand,vd_lefthand,Rd_lefthand,wd_lefthand,pd_righthand,vd_righthand,Rd_righthand,wd_righthand)