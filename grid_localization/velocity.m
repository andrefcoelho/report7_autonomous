function p = velocity(X,X_old,v,w,dt)

%Variances
alpha1=0.1*100;
alpha2=0.1*100;
alpha3=0.1*100;
alpha4=0.1*100;
alpha5=0.1*100;
alpha6=0.1*100;
%Variances
% alpha1=0.005*500;
% alpha2=0.005*500;
% alpha3=1*100;
% alpha4=1*100;
% alpha5=1e-1*10;
% alpha6=1e-1*10;


%Parameters
R=1; %m
L=2; %m

%states
x_old=X_old(1);
y_old=X_old(2);
try
    phi_old=X_old(3);
catch
    phi_old=atan2(x_old,y_old);
end
x=X(1);
y=X(2);
% phi=X(3);
phi=atan2(y,x);

%derivatives
x_dot=(x-x_old)/dt;
y_dot=(y-y_old)/dt;
phi_dot=(phi-phi_old)/dt;


if (phi~=0)
    %wheel velocities
    vl=y_dot/(R*sin(phi))-L/(2*R)*phi_dot;  %%phi_old?
    vr=L/R*phi_dot+vl;
    %linear and angular velocities
    v_hat=R/2*(vr+vl);
    w_hat=R/L*(vr-vl);
else
    v_hat=x_dot;
    w_hat=0;
end



%correction term
gamma_hat=phi_dot-w_hat;



sigma1=alpha1*abs(v)+alpha2*abs(w);
sigma2=alpha3*abs(v)+alpha4*abs(w);
sigma3=alpha5*abs(v)+alpha6*abs(w);

sigma1=max(sigma1,1e-10);
sigma2=max(sigma2,1e-10);
sigma3=max(sigma3,1e-10);
%probability
p1= gauss(v,v_hat,sigma1);
p2= gauss(w,w_hat,sigma2);
p3= gauss(gamma_hat,0,sigma3);
% p1= triang(v,v_hat,sigma1);
% p2= triang(w,w_hat,sigma2);
% p3= triang(gamma_hat,0,sigma3);

%probability of being at a state given velocity measurements
p=p1*p2*p3;



