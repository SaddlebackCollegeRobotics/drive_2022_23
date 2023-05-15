"""
Differential Drive Robot Kinematics
"""

WHEEL_SEPARATION = 0.38735 * 2
WHEEL_RADIUS = 0.194

separation: float = WHEEL_SEPARATION
radius: float = WHEEL_RADIUS


def angular_to_linear(angular_val: float) -> float:
    """convert angle, angular velocity, or angular acceleration to linear
    using neutonian mechanics"""
    return angular_val * WHEEL_RADIUS


def linear_to_angular(linear_val: float) -> float:
    """convert distance, velocity, or acceleration to angular
    using neutonian mechanics"""
    return linear_val / WHEEL_RADIUS


# ==== Differential Drive Robot Kinematics ====================================
# Title: Motion Model for the Differential Drive Robot
# Authors: Frank Dellaert, Seth Hutchinson
# Date: 2021
# Availability: https://www.roboticsbook.org/S52_diffdrive_actions.html
# =============================================================================
def i_kinematics(v_x: float, omega: float, L=separation, r=radius) -> tuple:
    """inverse kinematics: calculate angular wheels speeds from velocity.
    returns: (left wheel angular velocity, right wheel angular velocity)"""
    return (v_x - (L/2)*omega)/r, (v_x + (L/2)*omega)/r


def f_kinematics(phidot_l: float,
                 phidot_r: float,
                 L=separation,
                 r=radius) -> tuple:
    """forward kinematics: calculate velocity from angular wheels speeds.
    returns: (linear velocity, angular velocity)"""
    return (phidot_r+phidot_l)*r/2, (phidot_r-phidot_l)*r/L
