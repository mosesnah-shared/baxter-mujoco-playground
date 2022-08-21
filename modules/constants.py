class Constants:
    PROJECT_NAME            = '[M3X] Baxter'
    VERSION                 = '2.0.0'
    UPDATE_DATE             = '2022.08.18'
    AUTHOR_GITHUB           = 'mosesnah-shared'
    AUTHOR_FULL_NAME        = 'Moses C. Nah'
    DESCRIPTION             = "mujoco-py scripts for running a Baxter simulation"
    URL                     = 'https://github.com/mosesnah-shared/baxter-mujoco-playground',
    AUTHOR_EMAIL            = 'mosesnah@mit.edu', 'mosesnah@naver.com'

    # =============================================================== #
    # Constant variables for running the simulation
    # =============================================================== #
    
    # The module directory which contains all python modules.
    MODULE_DIR     = "./modules/"

    # The model directory which contains all the xml model files.
    MODEL_DIR     = "./models/"

    # The directory which saves all the simulation results
    SAVE_DIR      = "./results/"

    # The directory which saves all the temporary data
    TMP_DIR       = "./tmp/"

    BASIC_JOINT_NAMES = [ "s0", "s1", "e0", "e1", "w0", "w1", "w2" ]
    LEFT_JOINT_NAMES  = [  "left_" + joint_name for joint_name in BASIC_JOINT_NAMES ]
    RIGHT_JOINT_NAMES = [ "right_" + joint_name for joint_name in BASIC_JOINT_NAMES ]
    
    JOINT_NAMES = { "left": LEFT_JOINT_NAMES, "right": RIGHT_JOINT_NAMES }
    
    LEFT_ACT_NAMES  = [  "motor_" + joint_name for joint_name in LEFT_JOINT_NAMES ]
    RIGHT_ACT_NAMES = [  "motor_" + joint_name for joint_name in RIGHT_JOINT_NAMES ]    

    ACT_NAMES   = { "left": LEFT_ACT_NAMES, "right": RIGHT_ACT_NAMES }

    RIGHT2LEFT = { right_name:  left_name for right_name,  left_name in zip( RIGHT_JOINT_NAMES,  LEFT_JOINT_NAMES ) }
    LEFT2RIGHT = {  left_name: right_name for  left_name, right_name in zip(  LEFT_JOINT_NAMES, RIGHT_JOINT_NAMES ) }

    # Whether it is plus or minus of the value of the joint
    RIGHT_JOINT_SIGN = { "right_s0": +1, "right_s1": +1, "right_e0": +1, "right_e1": +1, "right_w0": +1, "right_w1": +1, "right_w2": +1 }       
    LEFT_JOINT_SIGN  = {  "left_s0": -1,  "left_s1": +1,  "left_e0": -1,  "left_e1": +1,  "left_w0": -1,  "left_w1": +1,  "left_w2": -1 }
    JOINT_SIGNS      = {  "right": RIGHT_JOINT_SIGN , "left": LEFT_JOINT_SIGN   }

    GRASP_POSE = {  'right_s0' : 0.7869321442,
                    'right_s1' : 0.4045874328,
                    'right_e0' : -0.0149563127,
                    'right_e1' : 1.4116458201,
                    'right_w0' : -0.0464029188,
                    'right_w1' :  0.3879126465,
                    'right_w2' : -1.5823011827 }

    MID_POSE = {    'right_s0' : 0.7869321442,
                    'right_s1' : -0.5419854693,
                    'right_e0' : -0.0149563127,
                    'right_e1' : 0.3097428536,
                    'right_w0' : -0.0464029188,
                    'right_w1' : -0.759660457, 
                    'right_w2' : -1.5823011827     }

    # Posture that is higher than expected
    GRASP_POSE_UP = {   'right_s0' : 0.6051554208,
                        'right_s1' : -0.1487961364,
                        'right_e0' : 0.1376747757,
                        'right_e1' : 1.3767477571,
                        'right_w0' : -0.0441019477,
                        'right_w1' : -1.3341797903,
                        'right_w2' : -1.7096215881 }