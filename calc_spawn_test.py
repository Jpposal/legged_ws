
import pinocchio as pin
import numpy as np
import sys
import os

# Define simplified link lengths for IIWA 14
# This is an approximation based on URDF values
# Joint Z locations:
# J1: 0
# J2: 0.36
# J3: 0.36 + 0.0 (Length of link 2 is 0?) No, L2 connects J2 to J3.
# Let's verify lengths from URDF logic:
# J1 origin: 0. 
# J2 origin: 0,0,0.36 (relative to L1).
# J3 origin: 0,0,0.0 (relative to L2). 
# J4 origin: 0.42 (relative to L3).
# J5 origin: 0 (relative to L4).
# J6 origin: 0.4 (relative to L5).
# J7 origin: 0 (relative to L6).
# Flange: 0.
# Total Vertical: 0.36 + 0.42 + 0.4 = 1.18m.

def get_ik(target_pos, target_rot, base_pos, base_rot_z=0):
    # Create a simple geometric model or Use Pinocchio if installed
    # Constructing a model manually is tedious.
    # Let's use a heuristic.
    
    # We want the end effector at `target_pos`.
    # Base is at `base_pos`.
    # Relative target:
    rel_pos = target_pos - base_pos
    # Rotate based on base rotation (simplified, assuming base rotation is just Z)
    # If base is rotated by PI around Z (Arm 1), we need to untransform.
    
    # Arm 1 Base (1.0, 0, 0) rotated PI.
    # Target (0.1, 0, 1.0).
    # Vector: (-0.9, 0, 1.0).
    # In Base Frame (Rotated PI): (0.9, 0, 1.0).
    
    # Arm 2 Base (0, 1.0, 0) rotated -PI/2.
    # Target (0, 0.1, 1.0).
    # Vector (0, -0.9, 1.0).
    # In Base Frame (Rotated -PI/2): (0.9, 0, 1.0) approx.
    # (Checking: X_base points -Y_world. Y_base points +X_world.
    #  Vector (0, -0.9, 1). X_base component is +0.9 (since -Y world is +X base). 
    #  Wait. Base frame of Arm 2:
    #  Origin (0,1,0). Rot -90 deg Z.
    #  X axis -> (0, -1, 0).
    #  Y axis -> (1, 0, 0).
    #  Target (0, 0.1, 1) relative to (0,1,0) is (0, -0.9, 1).
    #  Dot (0,-0.9,0) with X(0,-1,0) = 0.9.
    #  Dot (0,-0.9,0) with Y(1,0,0) = 0.
    #  So Local Target is (0.9, 0, 1.0).
    
    # So BOTH arms need to reach (0.9, 0, 1.0) in their local frame.
    
    # Kinematics for (x,y,z) = (0.9, 0, 1.0).
    # Since Y is 0, we can keep J1=0 (or 180? depends on config).
    # Staying in the plane.
    # We have links L (0.36 up), then a chain of (0.42, 0.4).
    # J2 is at height 0.36.
    # Target height 1.0. Delta Z = 0.64.
    # Target Radius = 0.9.
    
    # We have a 2-link planar arm (effectively) in the vertical plane defined by J2, J4, J6.
    # Link A = 0.42 (J2 to J4?? No, J2 rotates Y. J3 rotates Z. J4 rotates -Y).
    # So Links correspond to [0.42, 0.40].
    # Distance needed: sqrt(0.9^2 + 0.64^2) = sqrt(0.81 + 0.4096) = sqrt(1.2196) = 1.10.
    # Max Reach = 0.42 + 0.40 = 0.82.
    # 0.82 < 1.10.
    # IMPOSSIBLE.
    
    # My dimensions or positions are wrong.
    # IIWA 14 reach is 820mm (from name r820)?
    # Ah, the name says r820. Radius 820mm?
    # If so, placing base at 1.0m away for a grasp is too far.
    
    # If payload is at (0,0,1).
    # Grasp is at 0.1.
    # Base is at 1.0. Distance = 0.9.
    # Too far.
    
    print(f"Distance 0.9 is > Reach 0.82. Need to move base closer.")
    
get_ik(None, None, None)
