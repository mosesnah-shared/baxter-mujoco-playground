class Constants:
    PROJECT_NAME            = '[M3X] Whip Project'
    VERSION                 = '1.0.0'
    AUTHOR_GITHUB           = 'mosesnah-shared'
    AUTHOR_FULL_NAME        = 'Moses C. Nah'
    DESCRIPTION             = "mujoco-py scripts for running a whip-targeting simuation"
    URL                     = 'https://github.com/mosesnah-shared/whip-project-targeting',
    AUTHOR_EMAIL            = 'mosesnah@mit.edu', 'mosesnah@naver.com'

    # =============================================================== #
    # Constant variables for running the simulation

    # The model directory which contains all the xml model files.
    MODEL_DIR     = "models/"

    # The directory which saves all the simulation results
    SAVE_DIR      = "results/"

    LEFT_JOINT_NAME  = [  "left_s0",  "left_s1",  "left_e0",  "left_e1",  "left_w0",  "left_w1",  "left_w2" ]
    RIGHT_JOINT_NAME = [ "right_s0", "right_s1", "right_e0", "right_e1", "right_w0", "right_w1", "right_w2" ]

    REST_POSTURE_RIGHT =  { "right_s0":  0.6711165946998685,
                            "right_s1": -0.2834029505618302,
                            "right_e0":  0.1441941940612289,
                            "right_e1":  1.3997574689454400,
                            "right_w0": -3.0480198255283173,
                            "right_w1": -0.4329660773806580,
                            "right_w2": -1.8047283969471892  }

    REST_POSTURE_LEFT  =  {  "left_s0": -0.6711165946998685,
                             "left_s1": -0.2834029505618302,
                             "left_e0": -0.1441941940612289,
                             "left_e1":  1.3997574689454400,
                             "left_w0": -3.0480198255283173,
                             "left_w1": -0.4329660773806580,
                             "left_w2":  1.8047283969471892  }

    GRASP_POSTURE_RIGHT =  { "right_s0":  0.63621853177547540,
                             "right_s1": -0.30219421521342650,
                             "right_e0":  0.45099035163831164,
                             "right_e1":  1.78095169473496530,
                             "right_w0": -0.03604854851530722,
                             "right_w1": -1.44922834935474460,
                             "right_w2": -2.00567988016017830  }

    GRASP_POSTURE_LEFT =   {  "left_s0": -0.63621853177547540,
                              "left_s1": -0.30219421521342650,
                              "left_e0": -0.45099035163831164,
                              "left_e1":  1.78095169473496530,
                              "left_w0": -0.03604854851530722,
                              "left_w1": -1.44922834935474460,
                              "left_w2":  2.00567988016017830  }
