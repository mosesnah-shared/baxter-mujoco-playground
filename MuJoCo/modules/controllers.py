# [Built-in modules]

# [3rd party modules]
import numpy as np
import sys
import time
import pickle

from   modules.utils        import my_print
import matplotlib.pyplot as plt

try:
    import mujoco_py as mjPy

except ImportError as e:
    raise error.DependencyNotInstalled( "{}. (HINT: you need to install mujoco_py, \
                                             and also perform the setup instructions here: \
                                             https://github.com/openai/mujoco-py/.)".format( e ) )

# Added
try:
    import sympy as sp
    from sympy.utilities.lambdify import lambdify, implemented_function

except ImportError as e:
    raise error.DependencyNotInstalled( "{}. (HINT: you need to install sympy, \
                                             Simply type pip3 install sympy. \
                                             Sympy is necessary for building ZFT Calculation)".format( e ) )

# [Local modules]


class Controller( ):
    """
        Description:
        -----------
            Parent class for the controllers
    """


    def __init__( self, mjModel, mjData, mjArgs ):
        """

        """
        self.mjModel        = mjModel
        self.mjData         = mjData
        self.mjArgs         = mjArgs
        self.ctrl_par_names = None


    def set_ctrl_par( self, **kwargs ):
        """
            Setting the control parameters

            Each controllers have their own controller parameters names (self.ctrl_par_names),

            This method function will become handy when we want to modify, or set the control parameters.

        """
        if kwargs is not None:
            for args in kwargs:
                if args in self.ctrl_par_names:
                    setattr( self, args, kwargs[ args ] )
                else:
                    pass

    def input_calc( self, start_time, current_time ):
        """
            Calculating the torque input
        """
        raise NotImplementedError                                               # Adding this NotImplementedError will force the child class to override parent's methods.


class NullController( Controller ):
    """
        Description:
        ----------
            Controller which is simply empty, useful when practicing/debugging with MuJoCo

    """
    def __init__( self, mjModel, mjData, mjArgs ):
        super().__init__( mjModel, mjData, mjArgs )
        self.n_act = 0

    def set_ZFT( self ):
        return 0

    def input_calc( self, start_time, current_time ):
        return None, None, 0


# [TODO] [Moses C. Nah] [04.20.2021]
# We can simply use inheritance for the impedance controller

class ImpedanceController( Controller ):
    """
        Description:
        ----------
            Class for an Impedance Controller
            Inheritance of parent class "Contronller"

    """
    def __init__( self, mjModel, mjData, mjArgs ):

        super().__init__( mjModel, mjData, mjArgs )

        self.act_names      = mjModel.actuator_names                            # The names of the actuators, all the names end with "TorqueMotor" (Refer to xml model files)
        self.n_act          = len( mjModel.actuator_names )                     # The number of actuators, 2 for 2D model and 4 for 3D model
        self.idx_act        = np.arange( 0, self.n_act )                        # The idx array of the actuators, this is useful for self.input_calc method
        self.n_limbs        = '-'.join( mjModel.body_names ).lower().count( 'arm' ) # The number of limbs of the controller. Checking bodies which contain "arm" as the name (Refer to xml model files)
        self.type           = None
        self.g              = mjModel.opt.gravity                               # The gravity vector of the simulation

        # Impedance Controller uses pos, vel and acc of the ZFT (Zero-force trajectory), ZTT (Zero-torque trajectory)
        self.ZFT_func_pos   = None
        self.ZFT_func_vel   = None
        self.ZFT_func_acc   = None

        # The impedance parameter of the controller
        self.Kmat = None
        self.Bmat = None
        self.Mmat = None

        self.n_mov_pars     = None                                              # The number of parameters of the movement
        self.mov_parameters = None                                              # The actual values of the movement parameters, initializing it with random values
        self.n_ctrl_pars    = None                                              # The number of ctrl parameters. This definition would be useful for the optimization process.
        self.ctrl_par_names = None                                              # Useful for self.set_ctrl_par method
        self.t_sym = sp.symbols( 't' )                                          # time symbol of the equation


    def input_calc( self, start_time, current_time ):
        """
            Calculating the torque input
        """
        raise NotImplementedError                                               # Adding this NotImplementedError will force the child class to override parent's methods.

    def set_ZFT( self ):
        """
            Calculating the torque input
        """
        raise NotImplementedError                                               # Adding this NotImplementedError will force the child class to override parent's methods.

    def get_ZFT( self):
        """
            Calculating the torque input
        """
        raise NotImplementedError                                               # Adding this NotImplementedError will force the child class to override parent's methods.


if __name__ == "__main__":
    pass
