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
from controllers  import JointImpedanceController
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

def run_single_trial( mj_sim, mov_pars: dict, init_cond: dict ):
    """
        A function for running a single trial of simulation and return an array of the objective value of the simulation. 
    """

    mj_sim.ctrl.set_traj( mov_pars = mov_pars )    
    mj_sim.initialize( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )

    # Run the simulation
    mj_sim.run( )

    # Return the value of the simulation's objective value 
    return mj_sim.obj_arr 



if __name__ == "__main__":

    # Generate an instance of our Simulation
    my_sim = Simulation( args )

    # Instance for the controller of the simulation
    if   args.ctrl_name == "joint_imp_ctrl": 
        ctrl = JointImpedanceController( my_sim.mj_model, my_sim.mj_data, args, t_start = args.start_time )

    elif args.ctrl_name == "task_imp_ctrl":
        pass

    else:
        raise ValueError( f"[ERROR] Wrong controller name" )


    # Setup the controller and objective of the simulation
    my_sim.set_ctrl( ctrl )


    n = my_sim.ctrl.n_act   

    mov_pars  = {  "q0i": mov_arrs[ :n ] ,   "q0f": mov_arrs[ n: 2*n ] ,  "D": mov_arrs[ -1 ]  } 
    init_cond = { "qpos": mov_arrs[ :n ] ,  "qvel": np.zeros( my_sim.ctrl.n_act ) }


    obj_arr = run_single_trial( my_sim,  mov_pars = mov_pars, init_cond = init_cond )
    print( f"The minimum value of this trial is { min(obj_arr):.5f}" )

    my_sim.close( )

