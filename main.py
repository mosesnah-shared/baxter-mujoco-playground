"""

# ============================================================================= #
| Project:        '[M3X] Baxter'
| Title:          mujoco-py scripts for running a Baxter simulation
| Author:         Moses C. Nah
| Email:          [Moses] mosesnah@mit.edu
| Creation Date:  Aug 18th, 2022
| Final Update:   Aug 18th, 2022
# ============================================================================= #

"""


import os
import sys
import argparse

import numpy             as np
import matplotlib.pyplot as plt

# To Add Local Files, adding the directory via sys module
# __file__ saves the current directory of this file. 
sys.path.append( os.path.join( os.path.dirname(__file__), "modules" ) )

from simulation   import Simulation
from controllers  import JointImpedanceController, CartesianPositionImpedanceController
from constants    import Constants  as C
from utils        import *

# Setting the numpy print options, useful for printing out data with consistent pattern.
np.set_printoptions( linewidth = np.nan, suppress = True, precision = 4 )       
                                                                                
# Argument Parsers
parser = argparse.ArgumentParser( description = 'Parsing the arguments for running the simulation' )
parser.add_argument( '--version'     , action = 'version'     , version = C.VERSION )

parser.add_argument( '--start_time'  , action = 'store'       , type = float ,  default = 0.0,                   help = 'Start time of the controller'                                                      )
parser.add_argument( '--run_time'    , action = 'store'       , type = float ,  default = 4.0,                   help = 'Total run time of the simulation'                                                  )
parser.add_argument( '--model_name'  , action = 'store'       , type = str   ,  default = 'baxter_torque' ,      help = 'Model name for the simulation'                                                     )
parser.add_argument( '--ctrl_name'   , action = 'store'       , type = str   ,  default = 'joint_imp_ctrl',      help = 'Model name for the simulation'                                                     )
parser.add_argument( '--cam_pos'     , action = 'store'       , type = str   ,                                   help = 'Get the whole list of the camera position'                                         )
parser.add_argument( '--mov_pars'    , action = 'store'       , type = str   ,                                   help = 'Get the whole list of the movement parameters'                                     )
parser.add_argument( '--print_mode'  , action = 'store'       , type = str   ,  default = 'normal',              help = 'Print mode, choose between [short] [normal] [verbose]'                             )
parser.add_argument( '--print_freq'  , action = 'store'       , type = int   ,  default = 60      ,              help = 'Specifying the frequency of printing the date.'                                    )
parser.add_argument( '--vid_speed'   , action = 'store'       , type = float ,  default = 1.      ,              help = 'The speed of the video. It is the gain of the original speed of the video '        )

parser.add_argument( '--record_vid'  , action = 'store_true'  ,                                                  help = 'Record video of the simulation,  with the specified speed'     )
parser.add_argument( '--save_data'   , action = 'store_true'  ,                                                  help = 'Save the details of the simulation'                            )
parser.add_argument( '--vid_off'     , action = 'store_true'  ,                                                  help = 'Turn off the video'                                            )

# For jupyter compatibility.
# [REF] https://stackoverflow.com/questions/48796169/how-to-fix-ipykernel-launcher-py-error-unrecognized-arguments-in-jupyter
args, unknown = parser.parse_known_args( )


if __name__ == "__main__":

    # Generate an instance of our Simulation
    my_sim = Simulation( args )
    

    # Instance for the controller of the simulation
    if   args.ctrl_name == "joint_imp_ctrl": 
        
        # The RIGHT LIMB Impedances
        imp1_R = JointImpedanceController( my_sim, args, which_arm = "right" )
        
        Kq_mat = 0 * np.eye( 7 )
        Bq_mat = 0.6 * np.eye( 7 )
        
        imp1_R.set_impedance( Kq = Kq_mat, Bq = Bq_mat )
        imp1_R.add_movement( q0i = dict2arr( "right", C.GRASP_POSE_UP ), q0f = dict2arr( "right", C.GRASP_POSE_UP ), D = 3, ti = 1.0 )        

        my_sim.add_ctrl( imp1_R )

        my_sim.initialize( which_arm = "right", qpos = dict2arr( "right", C.GRASP_POSE_UP ), qvel = np.zeros( 7 ) )

        # The position impedance controller
        imp2_R = CartesianPositionImpedanceController( my_sim, args, which_arm = "right" )

        Kx_mat = 10.0 * np.eye( 3 )
        Bx_mat = 0.4 * Kx_mat
        imp2_R.set_impedance( Kx = Kx_mat, Bx = Bx_mat )

        imp2_R.add_movement( x0i = imp2_R.get_end_effector_pos( ), x0f = imp2_R.get_end_effector_pos( ) + np.array( [ 0, -0.4, 0. ] ), D = 4, ti = 1.0 )        

        my_sim.add_ctrl( imp2_R )

    elif args.ctrl_name == "task_imp_ctrl":
        pass

    else:
        raise ValueError( f"[ERROR] Wrong controller name" )

    my_sim.run( )
    
    my_sim.close( )

