function z = sense_unknown_landmark(L,pos) %L,c,pos
sigma_r=0.1;
sigma_phi=0.1;
r=zeros(size(L,2),1);
phi=zeros(size(L,2),1);
z=zeros(size(L,2),2);
for i=1:size(L,2)
    r(i)=sqrt((L(1,i)-pos(1))^2+(L(2,i)-pos(2))^2)+sample_norm(sigma_r);
    phi(i)=-atan2(L(2,i)-pos(2),L(1,i)-pos(1))+sample_norm(sigma_phi);
    phi(i)=wrapToPi(phi(i));
    z(i,:)=[r(i) phi(i)];
end


