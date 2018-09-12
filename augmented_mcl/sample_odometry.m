function X = sample_odometry(X_old,Xbar,Xbar_old)
alpha1=0.01*10;
alpha2=0.01*10;
alpha3=1e-6*50;
alpha4=1e-6*50;


%odometry measurements
xbar_old=Xbar_old(1);
xbar=Xbar(1);
ybar_old=Xbar_old(2);
ybar=Xbar(2);
phibar_old=Xbar_old(3);
phibar=Xbar(3);

%real states
x_old=X_old(1);
y_old=X_old(2);
phi_old=X_old(3);


%odometry deltas
Drot1=atan2(ybar-ybar_old,xbar-xbar_old)-phibar;
Dtrans=sqrt((xbar_old-xbar)^2+(ybar_old-ybar)^2);
Drot2=phibar-phibar_old-Drot1;

if(Drot1<0)
    Drot1=Drot1+2*pi;
end
if(Drot2<0)
    Drot2=Drot2+2*pi;
end

%state deltas
Drot1_hat=Drot1-sample_norm(alpha1*Drot1+alpha2*Dtrans);
Dtrans_hat=Dtrans-sample_norm(alpha3*Dtrans+alpha4*(Drot1+Drot2));
Drot2_hat=Drot2-sample_norm(alpha1*Drot2+alpha2*Dtrans);

if(Drot1_hat<0)
    Drot1_hat=Drot1_hat+2*pi;
end
if(Drot2_hat<0)
    Drot2_hat=Drot2_hat+2*pi;
end

x=x_old+Dtrans_hat*cos(phi_old+Drot1_hat);
y=y_old+Dtrans_hat*sin(phi_old+Drot1_hat);
phi=phi_old+Drot1_hat+Drot2_hat;
X=[x;y;phi];