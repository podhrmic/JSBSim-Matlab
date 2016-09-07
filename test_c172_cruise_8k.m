clear all; close all
clc

MexJSBSim('SetVerbosity',1)
MexJSBSim('open','c172x')

%% initial conditions and inputs
ic( 1).name  = 'u-fps';
ic( 1).value = 0;
ic( 2).name  = 'v-fps';
ic( 2).value = 0;
ic( 3).name  = 'w-fps';
ic( 3).value = 0;

ic( 4).name  = 'p-rad_sec';
ic( 4).value = 0.0;
ic( 5).name  = 'q-rad_sec';
ic( 5).value = 0.0;
ic( 6).name  = 'r-rad_sec';
ic( 6).value = 0.0;

ic( 7).name  ='h-sl-ft';
ic( 7).value = 4000;
ic( 8).name  ='long-gc-deg';
ic( 8).value = 49;
ic( 9).name  = 'lat-gc-deg';
ic( 9).value = 28;

ic(10).name  = 'phi-rad';
ic(10).value = 0;
ic(11).name  = 'theta-rad';
ic(11).value = 0;
ic(12).name  = 'psi-rad';
ic(12).value = 200;

ic(13).name  = 'aileron-cmd-norm';
ic(13).value = 0;
ic(14).name  = 'elevator-cmd-norm';
ic(14).value = 0;
ic(15).name  = 'rudder-cmd-norm';
ic(15).value = 0;

ic(16).name  = 'fcs/throttle-cmd-norm';
ic(16).value = 0;
ic(17).name  = 'fcs/mixture-cmd-norm';
ic(17).value = 0;
ic(18).name  = 'set-running';
ic(18).value = 0;
    
%% Init twice (so we zero out the inputs)
ret = MexJSBSim('init',ic)
ret = MexJSBSim('init',ic)

%% Set vt in knots
MexJSBSim('set','ic/vt-kts',100)

%%
MexJSBSim('run')
MexJSBSim('get','velocities/v-east-fps')
MexJSBSim('get','simulation/sim-time-sec')