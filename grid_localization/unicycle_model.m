function X_new = unicycle_model(X,U,dt)

%Parameters
R=1; %m
L=2; %m

% States
x=X(1);
y=X(2);
phi=X(3);


%Control Inputs
v=U(1);
w=U(2);

%Actual controls
vr=(2*v+w*L)/(2*R);
vl=(2*v-w*L)/(2*R);

% State Transition Model
phi_new = phi + dt*w;

x_dot   = R/2*(vr+vl)*cos(phi_new);
y_dot   = R/2*(vr+vl)*sin(phi_new);


%State Integration
x_new   = x   + dt*x_dot;
y_new   = y   + dt*y_dot;
phi_new=wrapToPi(phi_new);
X_new=[x_new y_new phi_new]';
end

