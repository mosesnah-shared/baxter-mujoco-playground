"""

# ============================================================================= #
| Project:        [M3X Whip Project]
| Title:          mujoco-py scripts for running BAXTER model
| Author:         Moses C. Nah
| Email:          [Moses] mosesnah@mit.edu
| Creation Date:  Tuesday, June 29th, 2021
# ============================================================================= #

# ============================================================================= #
| (0A) [DESCRIPTION]
|
|  - Python Script for running mujoco-py.
|    The corresponding xml model file is under "models" directory.
|
# ============================================================================= #

# ============================================================================= #
| (0B) [KEYWORDS DEFINITION]
|       : type the following "keywords" for cases as...
|         - [BACKUP] [NAME]: Back-up code in case it's needed for the near future
|         - [TODO]: The task which should be done as soon as possible
|         - [TIP]: The reason why the following code was written.
|         - [SBA]: Acronym for "Should Be Added"  in the future. This means that the mentioned functionality should be added soon.
|         - [CBC]: Acronym for "Could Be Changed" in the future. This means that the following code could be changed (or deprecated) soon.
# ============================================================================= #

# ============================================================================= #
| (0C) [PYTHON NAMING CONVENTION]
|       Our project will follow the python naming convention, [REF]: https://stackoverflow.com/a/8423697/13437196
|       ---------------------------------------------------------------------------------------------------------
|       module_name, package_name, ClassName, method_name, ExceptionName, function_name,
|       GLOBAL_CONSTANT_NAME, global_var_name, instance_var_name, function_parameter_name, local_var_name.
# ============================================================================= #

# ============================================================================= #
| (0D) [DOCOPT PARSE]
|      From now on, the written comments are specifically for "docopt" function.
|      [REF] http://docopt.org/
# ============================================================================= #

Usage:
    run.py [options]
    run.py -h | --help
    run.py -d | --debugMode

Arguments:

Options:
    -h --help                  Showing the usage and options
    --version                  Show version
    -s --saveData              Saving the neccessary data from MuJoCo simulation as a txt file in the current directory
                               [default: False]
    -r --recordVideo           Record simulation video as a .mp4 file in the current directory
                               [default: False]
    --vidRate=RATE             The rate of how fast the video runs. If 1.0, then normal speed, if 0.5, then 2 times slower.
                               [default: 1.0]
    --runTime=TIME             The total time of the simulation
                               [default: 5.0]
    --startTime=TIME           The start time of the movement, or controller
                               [default: 0.0]
    --runOptimization          Run the optimization of the simulation
                               [default: False]
    --modelName=NAME           Setting the xml model file name which will be used for the simulation.
                               The starting number of the xml model file indicates the type of simulation, hence the --modelName
                               already contains the simulation typep information.
                               List of models.
                                 - baxter.xml
                               [default: baxter.xml]
    --videoOFF                 Turning off the video
                               This is useful for cases when you want to make the computation of the simulation faster
                               [default: False]
    --camPos=STRING            Setting the Camera Position of the simulation.
                               default is None
    --verbose                  Print more text
                               [default: False]
    --attachWhipModel=STRING   Auto generating the xml model file used for the simulation.
                               The input string must contain N, M, L, k and b value.
                               For more detailed information, please see myMethods.py module's generateWhipModel function.
                               default is None

Examples, try:
      python3 run.py --help
      python3 run.py --version
      python3 run.py --modelName="baxter.xml" --runTime=6
      python3 run.py --modelName="baxter.xml" --startTime=1   --runTime=6


"""



# ============================================================================= #
# (0A) [IMPORT MODULES]
# Importing necessary modules + declaring basic configurations for running the whole mujoco simulator.

# [Built-in modules]
import sys
import os
import re
import argparse
import datetime
import shutil
import pickle

# [3rd party modules]
import cv2
import numpy as np

try:
    import mujoco_py as mjPy
except ImportError as e:
    raise error.DependencyNotInstalled( "{}. (HINT: you need to install mujoco_py, \
                                             and also perform the setup instructions here: \
                                             https://github.com/openai/mujoco-py/.)".format( e ) )

from docopt  import docopt

# [3rd party modules] - For Optimization
import matplotlib.pyplot as plt
# import nevergrad   as ng  # [BACKUP] Needed for Optimization

import sympy as sp
from sympy.utilities.lambdify import lambdify, implemented_function

# [Local modules]
from modules.simulation   import Simulation
from modules.controllers  import NullController
from modules.utils        import ( my_print, my_mkdir, args_cleanup,
                                   my_rmdir, str2float, camel2snake, snake2camel )
from modules.constants    import Constants

# ============================================================================= #

# ============================================================================= #
# (0B) [SYSTEM SETTINGS]

if sys.version_info[ : 3 ] < ( 3, 0, 0 ):                                       # Simple version check of the python version. python3+ is recommended for this file.
    my_print( NOTIFICATION = " PYTHON3+ is recommended for this script " )


                                                                                # [Printing Format]
prec = 4                                                                        # Defining the float precision for print/number comparison.
np.set_printoptions( linewidth = 8000, suppress = True, precision = prec )      # Setting the numpy print options, useful for printing out data with consistent pattern.

args = docopt( __doc__, version = Constants.VERSION )                           # Parsing the Argument
args = args_cleanup( args, '--' )                                               # Cleaning up the dictionary, discard prefix string '--' for the variables

# [TODO] [Moses]
# It might be beneficial, if we have some sort of "parser function", which gets the input args, and save it as the corresponding specific type.
# If video needs to be recorded or data should be saved, then append 'saveDir' element to args dictionary
args[ 'saveDir' ] = my_mkdir( ) if args[ 'recordVideo' ] or args[ 'saveData' ] or args[ 'runOptimization' ] else None

my_print( saveDir = args[ 'saveDir' ] )

# ============================================================================= #


# ============================================================================= #

def main( ):
    # ============================================================================= #
    # (1A) [GENERATE MODEL]

    model_name = "baxter.xml"
    my_print( modelName = model_name )

    # ============================================================================= #

    # ============================================================================= #
    # (1C) [RUN SIMULATION]

    VISUALIZE = False if args[ 'videoOFF' ] else True                           # Turn-off visualization if runOptimization


    mySim = Simulation(   model_name = model_name,
                        is_visualize = VISUALIZE,
                           arg_parse = args )

    ctrl = NullController( mySim.mjModel, mySim.mjData, args )
    mySim.attach_controller( ctrl )


    val = mySim.run( )                # Getting the minimum distance between tip and target




    if args[ 'saveDir' ] is not None:
        mySim.save_simulation_data( args[ 'saveDir' ]  )
        shutil.copyfile( Constants.MODEL_DIR + model_name,
                         args[ 'saveDir' ] + model_name )


    # ============================================================================= #

if __name__ == "__main__":

    try:
        main( )

    except KeyboardInterrupt:
        print( "Ctrl-C was inputted. Halting the program. ", end = ' ' )

        if args[ 'saveDir' ] is not None:
            my_rmdir( args[ 'saveDir' ] )
