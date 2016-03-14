% function MexJSBSim( string_directive, string_acname )
% DESCRIPTION:
% A mex-function that instantiates an JSBSim::FGFDMExec implements a number of 
% function usage:
% result = MexJSBSim( string_directive [, string, value] )
% 
% Examples:
%     res = MexJSBSim('help');
%        Prints this help text. returns 1 (always)
% 
%     res = MexJSBSim('open','c172r');
%        returns 1 if success, 0 otherwise
% 
%     res = MexJSBSim('get','fcs/elevator-cmd-norm')
%        returns the value of the property,
%        or -9999 if property not found
% 
%     res = MexJSBSim('set','fcs/elevator-cmd-norm',-0.5)
%        returns 1 if success, 0 otherwise
% 
%     res = MexJSBSim('set','elevator-cmd-norm',-0.5)
%     res = MexJSBSim('set','u-fps',80.5)
%        same as above, but it scans a set of predefined,
%        and commonly used variable names:
%             'u-fps', 'v-fps', 'w-fps'
%             'p-rad_sec', 'q-rad_sec', 'r-rad_sec'
%             'h-sl-ft'
%             'long-gc-deg'
%             'lat-gc-deg'
%             'phi-rad', 'theta-rad', 'psi-rad'
%             'elevator-cmd-norm'
%             'aileron-cmd-norm'
%             'rudder-cmd-norm'
% 
%     res = MexJSBSim('init',ic)
%        initializes the JSBSim::FGFDMExec by means of the
%        matlab structure ic.
%        The structure ic must have the two fields: name and value.
%        The Matlab user is forced to build such a structure first,
%        then to pass it to the mex-file. Example:
%        >> ic(1).name = 'u-fps'; ic(1).value = 80;
%        >> ic(2).name = 'v-fps'; ic(2).value =  0;
%        >> ic(3).name = 'w-fps'; ic(3).value =  0;
%        >> ic(4).name = 'p-rad_sec'; ic(4).value =  0;
%        >> MexJSBSim('init', ic);
% 
%     res = MexJSBSim('dot',ic)
%        same as above.
%        returns a res vector containing the following time rates.
%        ___ body-axis velocity component rates
%        res( 1) : u_dot (ft/s/s)
%        res( 2) : v_dot (ft/s/s)
%        res( 3) : w_dot (ft/s/s)
%        ___ body-axis angular velocity component rates
%        res( 4) : p_dot (rad/s/s)
%        res( 5) : q_dot (rad/s/s)
%        res( 6) : r_dot (rad/s/s)
%        ___ orientation quaternion component rates
%        res( 7) : q1_dot 
%        res( 8) : q2_dot 
%        res( 9) : q3_dot 
%        res(10) : q4_dot 
%        ___ ECEF position vector component rates
%        res(11) : x_dot (ft/s)
%        res(12) : y_dot (ft/s)
%        res(13) : z_dot (ft/s)
%        ___ Euler angle rates
%        res(14) : phi_dot   (rad/s)
%        res(15) : theta_dot (rad/s)
%        res(16) : psi_dot   (rad/s)
%        ___ Altitude rate
%        res(17) : h_dot     (ft/s)
%        ___ Aerodynamic angle rates
%        res(18) : alpha_dot (rad/s)
%        res(19) : beta_dot  (rad/s)
