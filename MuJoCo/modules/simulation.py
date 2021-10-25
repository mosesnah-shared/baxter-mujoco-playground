"""

[PYTHON NAMING CONVENTION]
    module_name, package_name, ClassName, method_name, ExceptionName, function_name,
    GLOBAL_CONSTANT_NAME, global_var_name, instance_var_name, function_parameter_name,
    local_var_name.


"""

import sys, os
import cv2
import re
import pprint
import numpy as np
import time, datetime


from modules.utils        import ( my_print, quaternion2euler, camel2snake, snake2camel,
                                   MyVideo, str2float)
from modules.constants    import Constants

try:
    import mujoco_py as mjPy


except ImportError as e:
    raise error.DependencyNotInstalled( "{}. (HINT: you need to install mujoco_py, \
                                             and also perform the setup instructions here: \
                                             https://github.com/openai/mujoco-py/.)".format( e ) )

# from mujoco_py import

class Simulation( ):
    """
        Running a single Whip Simulation

        [INHERITANCE]

        [DESCRIPTION]


        [NOTE]
            All of the model files are saved in "models" directory, and we are using "relative directory"
            to generate and find the .xml model file. Hence do not change of "model directory" variable within this

    """

    MODEL_DIR     = Constants.MODEL_DIR
    SAVE_DIR      = Constants.SAVE_DIR
    VISUALIZE     = True

    current_time  = 0
    controller    = None                                                        # Control input function

    min_val       = np.inf                                                      # Initializing the min_val of the optimization to np.inf

    def __init__( self, model_name = None, is_visualize = True, arg_parse = None ):
        """
            Default constructor of THIS class

            [ARGUMENTS]
                [NAME]             [TYPE]        [DESCRIPTION]
                (1) model_name     string        The xml model file name for running the MuJoCo simulation.
                (2) is_visualized  boolean       Turn ON/OFF the mjViewer (visualizer) of the simulation. This flag is useful when optimizing a simulation.
                (3) arg_parse      dictionary    Dictionary which contains all the arguments given to the main `run.py` script.
        """

        if model_name is None:
            self.mjModel  = None
            self.mjSim    = None
            self.mjData   = None
            self.mjViewer = None
            self.args     = arg_parse

            my_print( WARNING = "MODEL FILE NOT GIVEN, PLEASE INPUT XML MODEL FILE WITH `attach_model` MEMBER FUNCTION" )

        else:
            # If model_name is given, then check if there exist ".xml" at the end, if not, append
            model_name = model_name + ".xml" if model_name[ -4: ] != ".xml" else model_name
            self.model_name = model_name

            # Based on the model_name, construct the simulation.
            self.mjModel  = mjPy.load_model_from_path( self.MODEL_DIR + model_name )      #  Loading xml model as and save it as "model"
            self.mjSim    = mjPy.MjSim( self.mjModel )                                    # Construct the simulation environment and save it as "sim"
            self.mjData   = self.mjSim.data                                               # Construct the basic MuJoCo data and save it as "mjData"
            self.mjViewer = mjPy.MjViewerBasic( self.mjSim ) if is_visualize else None    # Construct the basic MuJoCo viewer and save it as "myViewer"
            self.args     = arg_parse

            # Saving the default simulation variables
            self.fps         = 60                                               # Frames per second for the mujoco render
            self.dt          = self.mjModel.opt.timestep                        # Time step of the simulation [sec]
            self.sim_step    = 0                                                # Number of steps of the simulation, in integer [-]
            self.update_rate = round( 1 / self.dt / self.fps / 5 )              # 1/dt = number of steps N for 1 second simulaiton, dividing this with frames-per-second (fps) gives us the frame step to be updated.
            self.g           = self.mjModel.opt.gravity                         # Calling the gravity vector of the simulation environment

            # Saving additional model parameters for multiple purposes
            self.act_names      = self.mjModel.actuator_names
            self.geom_names     = self.mjModel.geom_names
            self.idx_geom_names = [ self.mjModel._geom_name2id[ name ] for name in self.geom_names  ]

            self.n_acts      = len( self.mjModel.actuator_names )
            self.n_limbs     = '-'.join( self.mjModel.body_names ).lower().count( 'arm' )

            self.run_time   = float( self.args[ 'runTime'   ] )                 # Run time of the total simulation
            self.start_time = float( self.args[ 'startTime' ] )                 # Start time of the movements


        self.VISUALIZE = is_visualize                                           # saving the VISUALIZE Flag

    def read_data( self ):
        data_txt = "data/speed_joint_origin.txt"

        f = open( data_txt, "r" )
        self.data = str2float( f.readline( ) )

        while True:
            tmp = str2float( f.readline( ) )
            if not tmp:
                break
            else:
                self.data = np.vstack( ( self.data, tmp ) )


    def attach_model( self, model_name ):

        if self.mjModel is not None:
            my_print( WARNING = "MODEL FILE EXIST! OVERWRITTING THE WHOLE MUJOCO FILE" )

        self.__init__( model_name )

    def attach_controller( self, controller_name ):
        """
            Attaching the controller object for running the simulation.

            For detailed controller description, please check "controllers.py"

        """

        self.controller = controller_name

    def attach_obj_function( self, obj_func, weights = 1):
        """
            Attaching the objective function to be optimization

            The "obj_func" must have a form of following

            [SYNTAX]
                scalar_output = obj_func( mjModel, mjData )
        """

        self.obj_func = obj_func

    def run( self ):
        """
            Description
            -----------
            Running a single simulation.

            Arguments
            ---------
                [VAR NAME]             [TYPE]     [DESCRIPTION]
                (1) run_time           float      The whole run time of the simulation.
                (2) ctrl_start_time    float
        """

        # Check if mjModel or mjSim is empty and raise error
        if self.mjModel is None or self.mjSim is None:
            raise ValueError( "mjModel and mjSim is Empty! Add it before running simulation"  )


        if self.args[ 'recordVideo' ]:
            vid = MyVideo( fps = self.fps * float( self.args[ 'vidRate' ] ),
                       vid_dir = self.args[ 'saveDir' ] )                       # If args doesn't have saveDir attribute, save vid_dir as None

        if self.args[ 'saveData' ] or self.args[ 'runOptimization' ]:
            file = open( self.args[ 'saveDir' ] + "data_log.txt", "a+" )


        if self.args[ 'camPos' ] is not None:

            tmp = str2float( self.args[ 'camPos' ] )

            self.mjViewer.cam.lookat[ 0:3 ] = tmp[ 0 : 3 ]
            self.mjViewer.cam.distance      = tmp[ 3 ]
            self.mjViewer.cam.elevation     = tmp[ 4 ]
            self.mjViewer.cam.azimuth       = tmp[ 5 ]

        # self.read_data()

        # N = self.data.shape[ 0 ]

        while self.current_time <= self.run_time:

            if self.sim_step % self.update_rate == 0:

                if self.mjViewer is not None:
                    self.mjViewer.render( )                                     # Render the simulation


                if self.args[ 'verbose' ]:
                    my_print( camParameters = [ self.mjViewer.cam.lookat[ 0 ], self.mjViewer.cam.lookat[ 1 ], self.mjViewer.cam.lookat[ 2 ],
                                                self.mjViewer.cam.distance,    self.mjViewer.cam.elevation,   self.mjViewer.cam.azimuth ] )

                if self.args[ 'recordVideo' ]:
                    vid.write( self.mjViewer )

                if self.args[ 'saveData' ]:
                    my_print(  currentTime       = self.current_time, file              = file   )

            # [input controller]
            # input_ref: The data array that are aimed to be inputted (e.g., qpos, qvel, qctrl etc.)
            # input_idx: The specific index of input_ref data array that should be inputted
            # input:     The actual input value which is inputted to input_ref
            # input_ref, input_idx, input = self.controller.input_calc( self.start_time, self.current_time )
            #
            # if input_ref is not None:
            #     input_ref[ input_idx ] = input


            # self.mjSim.step( )                                                  # Single step update

            if( self.is_sim_unstable() ):                                       # Check if simulation is stable

                # If not optimization, and result unstable, then save the detailed data
                print( "[WARNING] UNSTABLE SIMULATION, HALTED AT {0:f} for at {1:f}".format( self.current_time, self.run_time )  )
                if self.args[ 'saveData' ] or self.args[ 'runOptimization' ]:
                    file.close( )

                break

            for j_names in Constants.LEFT_JOINT_NAME:
                idx = self.mjModel.joint_name2id( j_names )
                self.mjData.qpos[ idx ] = Constants.GRASP_POSTURE_LEFT[ j_names ]

            for j_names in Constants.RIGHT_JOINT_NAME:
                idx = self.mjModel.joint_name2id( j_names )
                self.mjData.qpos[ idx ] = Constants.GRASP_POSTURE_RIGHT[ j_names ]

            self.mjSim.forward( )    

            self.current_time = self.mjData.time                                # Update the current_time variable of the simulation



            # Calculate objective value based on mujoco Data
            # if self.obj_func is not None:
            #     self.min_val = min( self.min_val, self.obj_func( self.mjModel, self.mjData )  )

            # if self.sim_step % self.update_rate == 0:

                # if self.args[ 'saveData' ]:
                #     # Saving all the necessary datas for the simulation
                #     # my_print(  inputVal = input,
                #     #             minVal  = self.min_val,
                #     #                ZFT  = self.controller.phi,
                #     #                dZFT = self.controller.dphi,
                #     #                file = file )
                #
                # # elif self.args[ 'runOptimization' ]:
                #
                #     my_print(  geomXYZPositions  = self.mjData.geom_xpos[ self.idx_geom_names ],
                #                geomXYZVelocities = self.mjData.geom_xvelp[ self.idx_geom_names ],
                #                       outputVal  = self.obj_func( self.mjModel, self.mjData ),
                #                        inputVal  = input,file = file  )



                    # outputVal  = self.obj_func( self.mjModel, self.mjData )
                    #

                    # my_print(    minVal = self.min_val )

                # else:
                    # my_print(    minVal = self.min_val )

            self.sim_step += 1

        if self.args[ 'recordVideo' ]:
            vid.release( )                                                      # If simulation is finished, wrap-up the video file.

        if self.args[ 'saveData' ]:
            file.close()

        return self.min_val                                                     # Returning the minimum value achieved with the defined objective function

    def save_simulation_data( self, dir ):
        """
            Save all the details of the controller parameters, inputs and output of the simulation
        """

        if dir is not None and dir[ -1 ] != "/":                                # Quick Check of whether result_dir has backslash "/" at the end
            dir += "/"                                                          # Append the backslash

        # [TIP] [MOSES]
        # By using the "with" function you don't need to call f.close( ), the file will automatically close the opened file.
        # [REF] https://lerner.co.il/2015/01/18/dont-use-python-close-files-answer-depends/

        with open( dir + "simulation_details.txt", "w+" ) as f:
            pprint.pprint( self.controller.__dict__, f )                        # Using pretty-print (pprint) to flush out the data in a much readable format
            print( self.args                , file = f )                        # Flushing out all the arguments detail.


    def is_sim_unstable( self ):
        """
            Description
            -----------
            Check whether the simulation is stable.
            If the simulation exceeds some threshold value (10^6) for this case, then halting the simulation

        """
        thres = 1 * 10 ** 6

        if ( max( np.absolute( self.mjData.qpos ) ) > thres ) or \
           ( max( np.absolute( self.mjData.qvel ) ) > thres ) or \
           ( max( np.absolute( self.mjData.qacc ) ) > thres ):
           return True

        else:
           return False


    def reset( self ):
        """
            Reseting the mujoco simulation
        """
        self.current_time = 0
        self.sim_step     = 0
        self.min_val      = np.inf
        self.mjSim.reset( )
