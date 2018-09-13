function z = sense_unknown_landmark_clean(L,pos) %L,c,pos
r=zeros(length(L),1);
phi=zeros(length(L),1);
z=zeros(length(L),2);
for i=1:size(L,2)
    r(i)=sqrt((L(1,i)-pos(1))^2+(L(2,i)-pos(2))^2);
    phi(i)=-atan2(L(2,i)-pos(2),L(1,i)-pos(1));
    phi(i)=wrapToPi(phi(i));
    z(i,:)=[r(i) phi(i)];
end


