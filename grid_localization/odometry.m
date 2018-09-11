function p = odometry(X,X_old,Xbar,Xbar_old)
% alpha1=0.01;
% alpha2=0.01;
% alpha3=1e-4;
% alpha4=1e-4;
alpha1=0.01*100;
alpha2=0.01*100;
alpha3=1e-3*500;
alpha4=1e-3*50;

%odometry measurements
xbar_old=Xbar_old(1);
xbar=Xbar(1);
ybar_old=Xbar_old(2);
ybar=Xbar(2);
phibar_old=Xbar_old(3);
phibar=Xbar(3);

%real states
x_old=X_old(1);
x=X(1);
y_old=X_old(2);
y=X(2);
phi_old=X_old(3);
phi=X(3);

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
Drot1_hat=atan2(y-y_old,x-x_old)-phi;
Dtrans_hat=sqrt((x_old-x)^2+(y_old-y)^2);
Drot2_hat=phi-phi_old-Drot1_hat;


if(Drot1_hat<0)
    Drot1_hat=Drot1_hat+2*pi;
end
if(Drot2_hat<0)
    Drot2_hat=Drot2_hat+2*pi;
end


sigma1=alpha1*Drot1_hat+alpha2*Dtrans_hat;
sigma2=alpha3*Dtrans_hat+alpha4*(Drot1_hat+Drot2_hat);
sigma3=alpha1*Drot2_hat+alpha2*Dtrans_hat;

%probability
p1= gauss(Drot1,Drot1_hat,sigma1);
p2= gauss(Dtrans,Dtrans_hat,sigma2);
p3= gauss(Drot2,Drot2_hat,sigma3);
% p1= triang(Drot1,Drot1_hat,sigma1);
% p2= triang(Dtrans,Dtrans_hat,sigma2);
% p3= triang(Drot2,Drot2_hat,sigma3);

%probability of being at a state given odometry measurements
p=p1*p2*p3;