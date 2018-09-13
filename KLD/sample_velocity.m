function X = sample_velocity(X_old,v,w,dt)
alpha1=0.005*100;
alpha2=0.005*100;
alpha3=1*2;
alpha4=1*2;
alpha5=1e-1*10;
alpha6=1e-1*10;

%states
x_old=X_old(1);
y_old=X_old(2);
phi_old=X_old(3);

v_hat=v+sample_norm(alpha1*abs(v)+alpha2*abs(w));
w_hat=w+sample_norm(alpha3*abs(v)+alpha4*abs(w));
gamma_hat=sample_norm(alpha5*abs(v)+alpha6*abs(w));

x=x_old-v_hat/w_hat*sin(phi_old)+v_hat/w_hat*sin(phi_old+w_hat*dt);
y=y_old-v_hat/w_hat*cos(phi_old)+v_hat/w_hat*cos(phi_old+w_hat*dt);
phi=phi_old+w_hat*dt+gamma_hat*dt;
X=[x;y;phi];
