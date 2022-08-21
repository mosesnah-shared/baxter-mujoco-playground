import numpy  as np
from   modules.utils     import *
from   modules.constants import Constants as C

class Controller:
    """
        Description:
        -----------
            Parent class for the controllers

    """

    def __init__( self, mj_sim, args, which_arm : str ): 

        # Saving the reference of mujoco model and data for 
        self.mj_sim   = mj_sim
        self.mj_model = mj_sim.mj_model
        self.mj_data  = mj_sim.mj_data

        # Saving the arguments passed via ArgumentParsers
        self.mj_args  = args

        # Saving whether the limb is right or left
        assert which_arm in [ "right", "left" ]
        self.which_arm = which_arm 

        # Check whether we will save the data
        self.is_save_data = args.save_data

        # There are crucial parameters which can be calculated from the given model. 
        # Hence, "parsing" the xml model file

        # ====================================================== #


    def input_calc( self, t ):
        """
            Calculating the torque input for the given time 
        """
        raise NotImplementedError

    def get_joint_pos( self ):
        """
            Return a (1 x 7) array with the RIGHT/LEFT LIMB's joint posture
        """
        
        return np.array( [ self.mj_data.get_joint_qpos( joint ) for joint in C.JOINT_NAMES[ self.which_arm ]  ] )

    def get_joint_vel( self ):
        """
            Return a (1 x 7) array with the RIGHT/LEFT LIMB's joint velocity
        """
        
        return np.array( [ self.mj_data.get_joint_qvel( joint ) for joint in C.JOINT_NAMES[ self.which_arm ]  ] )

class JointImpedanceController( Controller ):

    """
        Description:
        ----------
            Class for a Joint (i.e., Generalized coordinate) Impedance Controller
            First order impedance controller with gravity compenation

    """

    def __init__( self, mj_sim, mj_args, which_arm:str ):

        # Choose which arm we will use for Baxter
        assert which_arm in [ "right", "left" ]

        super( ).__init__( mj_sim, mj_args, which_arm )

        # The number of actuators for each arm is 7
        self.n_act = 7

        # Define the controller parameters that we can change via "set_ctr_par" method
        self.ctrl_par_names = [ "Kq", "Bq", "q0i", "q0f", "D", "ti" ]       

        # Define the parameters that we will use for printing. This will be useful for "printing out the variables' detail" 
        self.print_par_naems = [ "K", "B", "q0" ]

        # The movement parameters that we need to save
        self.Kq, self.Bq = [], []
        self.q0i, self.q0f, self.D, self.ti = [], [], [], []

        # Saving the joint index for the controllers
        self.idx_ctrl = np.array( [ self.mj_model.actuator_name2id( j ) for j in C.ACT_NAMES[ "right" ] ] )

        # The number of movements 
        self.n_mov = 0 

        # If save data, then 
        if self.is_save_data:
            Ns = int( mj_args.run_time * mj_args.print_freq ) + 1

            # The time array 
            self.t_arr = np.zeros( Ns )

            # The Torque input array
            self.tau_arr = np.zeros( ( 7, Ns ) )            
    
            # The current q (and q0) which is the joint (zero-torque) posture
            self.q_arr  = np.zeros( ( 7, Ns ) )            
            self.q0_arr = np.zeros( ( 7, Ns ) )                        

            # The current qdot (and q0dot) which is the joint (zero-torque) velocity 
            self.dq_arr  = np.zeros( ( 7, Ns ) )                        
            self.dq0_arr = np.zeros( ( 7, Ns ) )                        

            # The Jacobian matrix of the end-effector
            self.Jp_arr = np.zeros( ( 3, 7, Ns ) )
            self.Jr_arr = np.zeros( ( 3, 7, Ns ) )

            # The index for saving the data
            self.idx_data = 0                     


    def set_impedance( self, Kq: np.ndarray, Bq: np.ndarray ):

        # Check whether it is a 7-by-7 array
        assert len( Kq      ) == 7 and len( Bq      ) == 7
        assert len( Kq[ 0 ] ) == 7 and len( Bq[ 0 ] ) == 7
        
        # Setting up the Kq, Bq matrices
        self.Kq = Kq
        self.Bq = Bq

    def add_movement( self, q0i: np.ndarray, q0f:np.ndarray, D:float, ti:float ):

        assert len( q0i ) == 7
        assert len( q0f ) == 7

        assert D > 0 and ti >= 0

        self.q0i.append( q0i )
        self.q0f.append( q0f )
        self.D.append(     D )
        self.ti.append(   ti )     

        self.n_mov += 1   


    def save_data( self, dir_name ):
        """
            Saving the data as a .mat file
        """        
        NotImplementedError( )

    def input_calc( self, t ):
        """
            Descriptions
            ------------
                We implement the controller. 
                The controller generates torque with the following equation 

                tau = K( q0 - q ) + B( dq0 - dq ) 

                (d)q0: The zero-torque trajectory, which follows a minimum-jerk profile. 
                 (d)q: current angular position (velocity) of the robot

            Arguments
            ---------
                t: The current time of the simulation. 
        """

        # Iterating through the movement 
        self.q  = self.get_joint_pos( )
        self.dq = self.get_joint_vel( ) 

        # The Zero torque trajectory 
        self.q0  = np.zeros( 7 )
        self.dq0 = np.zeros( 7 )

        # Setting up the zero-torque trajectory
        for i in range( self.n_mov ):
            for j in range( 7 ):
                tmp_q0, tmp_dq0 = min_jerk_traj( t, self.ti[ i ], self.ti[ i ] + self.D[ i ], self.q0i[ i ][ j ], self.q0f[ i ][ j ], self.D[ i ] )

                self.q0[ j ]  += tmp_q0 
                self.dq0[ j ] += tmp_dq0

        self.tau = self.Kq @ ( self.q0 - self.q ) + self.Bq @ ( self.dq0 - self.dq )
 
        # The  (1) object         (2) index array       (3) It's value. 
        return self.mj_data.ctrl, self.idx_ctrl,        self.tau

    def save_data( self ):
        NotImplementedError( )

    def reset( self ):
        """
            Initialize all variables  
        """

        self.Kq, self.Bq = [], []
        self.q0i, self.q0f, self.D, self.ti = [], [], [], [] 

        self.n_act = 0 

class CartesianImpedanceController( Controller ):

    def __init__( self, mj_model, mj_data, mj_args, t_start: float = 0 ):

        super( ).__init__( mj_model, mj_data, mj_args, t_start )

        # Getting the number of actuators 
        self.n_act = len( self.mj_model.actuator_names )

        # Define the controller parameters that we can change via "set_ctr_par" method
        self.ctrl_par_names = [ "Kx", "Bx", "x0i", "x0f", "D" ]       

        # Define the parameters that we will use for printing. This will be useful for "printing out the variables' detail" 
        self.print_par_naems = [ "K", "B", "q0" ]

    def set_traj( self, mov_pars: dict, basis_func: str = "min_jerk_traj" ):
        self.x0i = mov_pars[ "q0i" ]
        self.x0f = mov_pars[ "q0f" ]
        self.D   = mov_pars[  "D"  ]


        if   basis_func == "min_jerk_traj":
            self.traj_pos = lambda t :      self.x0i + ( self.x0f - self.x0i ) * (  10 * ( t / self.D ) ** 3 - 15 * ( t / self.D ) ** 4 +  6 * ( t / self.D ) ** 5 )
            self.traj_vel = lambda t :  1.0 / self.D * ( self.x0i - self.x0i ) * (  30 * ( t / self.D ) ** 2 - 60 * ( t / self.D ) ** 3 + 30 * ( t / self.D ) ** 4 )

        elif basis_func == "b_spline":
            pass

    def input_calc( self, t:float ):
        # The following two trajectories  should not be "None"
        assert self.traj_pos and self.traj_vel

        # Get the current angular position and velocity of the robot 
        q  = self.mj_data.qpos[ 0 : self.n_act ]       
        dq = self.mj_data.qvel[ 0 : self.n_act ]
 
        if    t < self.t_start:
            self.q0  = self.q0i 
            self.dq0 = np.zeros( self.n_act )

        elif  self.t_start < t <= self.t_start + self.D:
            self.q0  = self.traj_pos( t - self.t_start )
            self.dq0 = self.traj_vel( t - self.t_start )
        else:
            self.q0  = self.q0f
            self.dq0 = np.zeros( self.n_act )

        tau_imp = np.dot( self.K, self.q0 - q ) + np.dot( self.B, self.dq0 - dq )

        self.tau  = tau_imp 

        # The  (1) object         (2) index array          (3) It's value. 
        return self.mj_data.ctrl, np.arange( self.n_act ), self.tau

    def reset( self ):
        NotImplementedError( )
