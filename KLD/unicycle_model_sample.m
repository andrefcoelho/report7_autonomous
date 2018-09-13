function X_new = unicycle_model_sample(X,U,dt)
%Variances
alpha1=0.1*2;
alpha2=0.1*2;
alpha3=0.1*2;
alpha4=0.1*2;
alpha5=0.1*2;
alpha6=0.1*2;

v=U(1);
w=U(2);
sigma1=alpha1*abs(v)+alpha2*abs(w);
sigma2=alpha3*abs(v)+alpha4*abs(w);
sigma1=max(sigma1,1e-10);
sigma2=max(sigma2,1e-10);

v=v+sample_norm(sigma1);
w=w+sample_norm(sigma2);
%Parameters
R=1; %m
L=2; %m

% States
x=X(1);
y=X(2);
phi=X(3);


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

