clear all; close all
clc

global ic;

MexJSBSim('SetVerbosity','silent')
MexJSBSim('open','c172r')

% MexJSBSim('SetVerbosity',1)

% initial conditions and inputs
ic( 1).name  = 'u-fps';
ic( 1).value = convvel(90,'km/h','ft/s');
ic( 2).name  = 'v-fps';
ic( 2).value = 0;
ic( 3).name  = 'w-fps';
ic( 3).value = ic( 1).value * tan(4*pi/180);

ic( 4).name  = 'p-rad_sec';
ic( 4).value = 0.0;
ic( 5).name  = 'q-rad_sec';
ic( 5).value = 0.0;
ic( 6).name  = 'r-rad_sec';
ic( 6).value = 0.0;

ic( 7).name  ='h-sl-ft';
ic( 7).value = 3000;
ic( 8).name  ='long-gc-deg';
ic( 8).value = 122;
ic( 9).name  = 'lat-gc-deg';
ic( 9).value = 47;

ic(10).name  = 'phi-rad';
ic(10).value = 0;
ic(11).name  = 'theta-rad';
ic(11).value = 2*pi/180.;
ic(12).name  = 'psi-rad';
ic(12).value = 0;

ic(13).name  = 'aileron-cmd-norm';
ic(13).value = 1;
ic(14).name  = 'elevator-cmd-norm';
ic(14).value = -0.1;
ic(15).name  = 'rudder-cmd-norm';
ic(15).value = 1;

ic(16).name  = 'fcs/throttle-cmd-norm';
ic(16).value = 1;
ic(17).name  = 'fcs/mixture-cmd-norm';
ic(17).value = 0.7;
ic(18).name  = 'set-running';
ic(18).value = 1;

MexJSBSim('init',ic);

%% -------------------------------------------------
u = ic( 1).value;
v = ic( 2).value;
w = ic( 3).value;
theta = ic(11).value;

Vt = sqrt(u.^2+v.^2+w.^2);
[alpha beta] = alphabeta([u v w]);

alpha*180/pi

na = 10;
ne = 15;
valpha = linspace(-12,12,na)*pi/180;
vde    = linspace(-1,1,ne);
[X,Y] = meshgrid(valpha,vde);
X = X*180/pi;


for i=1:ne
    for j=1:na
        Z(i,j) = cost_fdm(Vt,valpha(j),beta,5*pi/180,0,vde(i),0,0.8);
    end
end

surf(X,Y,Z)
