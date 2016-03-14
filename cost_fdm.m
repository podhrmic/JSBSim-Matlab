function r = cost_fdm(V,alpha,beta,theta,da,de,dr,dth)

global ic;

ic(11).value = theta;

w1  = V.*sin(beta);
Vxz = V.*cos(beta);
u1  = Vxz.*cos(alpha);
v1  = Vxz.*sin(alpha);

ic( 1).value = u1;
ic( 2).value = v1;
ic( 3).value = w1;

statedot = MexJSBSim('dot',ic);
% Note: returns a column vector

r = sqrt(statedot'*statedot);