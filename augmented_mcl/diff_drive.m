function X_new = diff_drive(X,U,dt)
%DIFF_DRIVE state transition model for differential drive robots
% U=[vl vr] control input - Velocities of the left wheel and right wheel,respectively
% X=[x y phi] state vector - [x,y] position and heading (phi) of the robot
% Ts sampling time 

%Parameters
R=1; %m
L=2; %m

% States
x=X(1);
y=X(2);
phi=X(3);

%Control Inputs
vl=U(1);
vr=U(2);

% State Transition Model
x_dot   = R/2*(vr+vl)*cos(phi);
y_dot   = R/2*(vr+vl)*sin(phi);
phi_dot = R/L*(vr-vl);

%State Integration
x_new   = x   + dt*x_dot;
y_new   = y   + dt*y_dot;
phi_new = phi + dt*phi_dot;

X_new=[x_new y_new phi_new]';
end

