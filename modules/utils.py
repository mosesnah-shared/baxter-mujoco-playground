import re
import sys
import math
import numpy as np

from   modules.constants import Constants as C

# Define functions to be imported when used "import *"
__all__  = [ "pose_right2left", "dict2arr", "arr2dict", "str2float", "quaternion2euler", "print_vars", "min_jerk_traj" ] 

def pose_right2left( pose: dict ):
    """
        Changing the dictionary key's prefix name from "right_" to "left_", and flipping the sign too
        The reason why we don't have left2right is because we will follow the "right-hand" convention,
        meaning, all the constants in "my_constants.py" are saved as "right" hand informatino
        Arguments:
            [1] pose ( dict ): 7-element dictionary with keys "right_" + s0, s1, e0, e1, w0, w1, w2
    """
    
    # Check whether the given dictionary has all the "right_" + s0, s1, e0, e1, w0, w1 and w2 on the keys.
    assert all( [ c in pose.keys( ) for c in C.JOINT_NAMES[ "right" ] ] )   

    new_pose = dict( )
    
    # We can write this with a single line, but just for code readability, using the for loop 
    for right_name in C.JOINT_NAMES[ "right" ]:
        left_name = C.RIGHT2LEFT[ right_name ]
        new_pose[ left_name ] = C.LEFT_JOINT_SIGN[ left_name ] * pose[ right_name ]

    return new_pose

def dict2arr( which_arm: str, my_dict: dict  ):
    """
        Change the dictionary to array value     
        
        Args: 
        
            [2] my_dict - should contain joints s0, s1, e0, e1, w0, w1, w2 
        
    """

    assert which_arm in [ "right", "left" ]

    # Check whether s0, s1, e0, e1, w0, w1, w2, which is the list of C.BASIC_JOINT_NAMES are in the my_dict keys    
    assert all( [ c in my_dict.keys( ) for c in C.JOINT_NAMES[ which_arm ] ] ) 
    
    # The length of the dictionary must be 7
    assert len( my_dict ) == 7
    
    # Generate a 1x7 array with numbers ordered as s0, s1, e0, e1, w0, w1, w2 
    # Again, for readability, we will set this using a for loop
    return np.array( [ my_dict[ key ] for key in C.JOINT_NAMES[ which_arm ] ] )

def arr2dict( which_arm: str, arr: np.ndarray ):

    assert which_arm in [ "right", "left" ]
    assert len( arr ) == 7

    return { joint_name : arr[ i ] for i, joint_name in enumerate( C.JOINT_NAMES[ which_arm ] ) }


def min_jerk_traj( t: float, ti: float, tf: float, pi: float, pf: float, D: float ):
    """
        Returning the 1D position and velocity data at time t of the minimum-jerk-trajectory ( current time )
        Time should start at t = 0
        Note that the minimum-jerk-trajectory remains at the initial (respectively, final) posture before (after) the movement.
        Arguments
        ---------
            [1] t : current time
            [2] ti: start of the movement
            [3] tf: end   of the movement
            [4] pi: initial ( reference ) posture
            [5] pf: final   ( reference ) posture
            [6]  D: duration
    """

    assert  t >=  0 and ti >= 0 and tf >= 0 and D >= 0
    assert tf >= ti

    if   t <= ti:
        pos = pi
        vel = 0

    elif ti < t <= tf:
        tau = ( t - ti ) / D                                                # Normalized time
        pos =    pi + ( pf - pi ) * ( 10 * tau ** 3 - 15 * tau ** 4 +  6 * tau ** 5 )
        vel = 1 / D * ( pf - pi ) * ( 30 * tau ** 2 - 60 * tau ** 3 + 30 * tau ** 4 )

    else:
        pos = pf
        vel = 0

    return pos, vel

def quaternion2euler( quat: np.ndarray ):                                         
    """
        Description
        -----------
        Converting a R4 quaternion vector (w, x, y, z) to Euler Angle (Roll, Pitch, Yaw)
        This code is directly from the following reference
        [REF] https://computergraphics.stackexchange.com/questions/8195/how-to-convert-euler-angles-to-quaternions-and-get-the-same-euler-angles-back-fr

        Arguments
        ---------
            [NAME]          [TYPE]        [DESCRIPTION]
            (1) quatVec     List          The quaternion vector, ordered in w, x, y and z

        Outputs
        --------
            [NAME]                   [TYPE]        [DESCRIPTION]
            (1) yaw, pitch, roll                   The euler angles of the given quaternion vector.
    """

    assert len( quat ) == 4

    w, x, y ,z  = quat[:]

    t0     =       + 2.0 * ( w * x + y * z )
    t1     = + 1.0 - 2.0 * ( x * x + y * y )
    roll   = math.atan2( t0, t1 )

    t2     = + 2.0 * ( w * y - z * x )
    t2     = + 1.0 if t2 > +1.0 else t2
    t2     = - 1.0 if t2 < -1.0 else t2
    pitch  = math.asin( t2 )

    t3     =       + 2.0 * ( w * z + x * y )
    t4     = + 1.0 - 2.0 * ( y * y + z * z )
    yaw    = math.atan2( t3, t4 )

    return yaw, pitch, roll

def str2float( string2parse : str ):
    """
        Return A list of float that is parsed from given string s
    """

    return [ float( i ) for i in re.findall( r"[-+]?\d*\.\d+|[-+]?\d+", string2parse ) ]


def print_vars( vars2print: dict , save_dir = sys.stdout ):
    """
        Print out all the details of the variables to the standard output + file to save. 
    """

    # Iterate Through the dictionary for printing out the values. 
    for var_name, var_vals in vars2print.items( ):

        # Check if var_vals is a list or numpy's ndarray else just change it as string 
        if   isinstance( var_vals, ( list, np.ndarray ) ):
            
            # First, change the list to numpy array to make the problem easier 
            var_vals = np.array( var_vals ) if isinstance( var_vals, list ) else var_vals

            # If the numpy array is
            var_vals = np.array2string( var_vals.flatten( ), separator =', ', floatmode = 'fixed' )

        else:
            var_vals = str( var_vals )

        print( f'[{var_name}]: {var_vals}', file = save_dir )
