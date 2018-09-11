function [mu,sigma,p]=ekf_loc_unknown(mu1,sigma1,v,w,z,L,dt)
% variances
a1=50*dt*0.001;
a2=50*dt;
a3=50*dt;
a4=50*dt;
var_r=10;
var_phi=10;

sigma1=sigma1(1:2,1:2);

th=mu1(3);
G=[ 1 0; ... v/w*(-cos(th)+cos(th+w*dt)); ...
    0 1];% v/w*(-sin(th)+sin(th+w*dt)); ...
%     0 0 1];
V=[-1/w*(sin(th)+sin(th+w*dt)) v/w^2*(sin(th)-sin(th+w*dt))+v/w*(cos(th+w*dt)*dt); ...
    1/w*(cos(th)-cos(th+w*dt)) -v/w^2*(cos(th)-cos(th+w*dt))+v/w*(sin(th+w*dt)*dt)];%; ...
%     0 dt];
M=[a1*v^2+a2*w^2 0;0 a3*v^2+a4*w^2]*100;

mu_bar= unicycle_model(mu1,[v,w],dt);
mu_bar=mu_bar(1:2);
% mu1+[v/w*(-sin(th)+sin(th+w*dt)); ...
%         v/w*(cos(th)-cos(th+w*dt)); ...
%         w*dt];
sigma_bar=G*sigma1*G'+V*M*V';
Q=[var_r 0;0 var_phi];
S=zeros(2,length(z)*2);
H=zeros(2,length(z)*2);
z_hat = sense_unknown_landmark_clean(L,mu_bar);
for i=1:size(z,1)   %for all observed features
    for k=1:size(z_hat,1) %for all landamarks on the map
        H1=[-(L(1,k)-mu_bar(1))/(z_hat(k,1))  -(L(2,k)-mu_bar(2))/(z_hat(k,1)); ...    0; ...
            (L(2,k)-mu_bar(2))/(z_hat(k,1)^2) -(L(1,k)-mu_bar(1))/(z_hat(k,1)^2)];% -1];% ...
        %0 0 0];
        H(:,1+2*(k-1):2+2*(k-1))=H1;
        S1=H1*sigma_bar*H1'+Q;
        S(:,1+2*(k-1):2+2*(k-1))=S1;
        j1(k)= ((det(2*pi*S1))^-0.5)*exp((-0.5*(z(i,1:2)-z_hat(k,:))*inv(S1)*(z(i,1:2)-z_hat(k,:))'));
        j1(isnan(j1))=0;
    end
    [~,imax]=max(j1);
    j=imax;
    if sum(j1)==4
        j=i
    end
        
    K=sigma_bar*H(:,1+2*(j-1):2+2*(j-1))'*inv(S(:,1+2*(j-1):2+2*(j-1)));
    mu_bar=mu_bar+K*(z(i,1:2)-z_hat(j,1:2))';
    sigma_bar=(eye(2)-K*H(:,1+2*(j-1):2+2*(j-1)))*sigma_bar;
end

mu=mu_bar;
sigma=sigma_bar;
p=1;