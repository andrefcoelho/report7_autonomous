% Unicycle Localization
clear
close all

%Map and landmarks


M1=imread('map3.bmp');
M=sum(M1,3);
M=(M==765);

L(:,1)=[10 10];
L(:,2)=[10 90];
L(:,3)=[90 90];
L(:,4)=[90 10];
c=[0 0 1;1 0 1;1 1 0;0 1 1];

figure(1)
image(M1)
set(gca,'YDir','normal')
hold on
scatter(L(1,:),L(2,:),100,c,'filled')

N=100;

%simulation
dt=0.1;
n=50/dt;
n=round(n);
v=30*ones(1,n);
w=-1*ones(1,n);
p_all=zeros(n,3);
p=[50 80 0];
% mu=p';
mu=[50 30 0.5]';
sigma=1e5*ones(3,3);

tit=strcat('\Deltat=',num2str(dt));
title(tit)
for i=1:n-1
    p_all(i,:)=p;
    p=unicycle(v(i),w(i),p,dt);
    p=p(end,:);
    z = sense_known_landmark(L,p);
    [mu,sigma,prob]=ekf_loc_known(mu,sigma,v(i),w(i),z,L,dt);
    for j=1:N
        xp(j)=mu(1)+sample_norm(sigma(1,1));
        yp(j)=mu(2)+sample_norm(sigma(1,1));
    end
    plot(xp,yp,'.','color',[1 0.5 0.3])
    scatter(p(1),p(2),300,'g','filled')
    scatter(mu(1),mu(2),300,[1 0.25 0],'filled')
    hold off
    pause(dt/3)
    image(M1)
    title(tit)
    set(gca,'YDir','normal')
    hold on
    scatter(L(1,:),L(2,:),100,c,'filled')
    
end
p_all(i+1,:)=p;
plot(xp,yp,'.','color',[1 0.5 0.3])
scatter(p(1),p(2),300,'g','filled')
scatter(mu(1),mu(2),300,[1 0.25 0],'filled')

% % plot(p_all(:,1),p_all(:,2),'g')
% %
% %
% 
% 
% 
% 
% pos=[160 50];
% %
% % for l=1:5
% %
% %     grid on
% %
% %     x=linspace(1,300,500);
% %     y=linspace(1,100,500);
% %     p=zeros(length(y)*length(x),1);
% %     aux=0;
% %     for i=1:length(x)
% %         for j=1:length(y)
% %             aux=aux+1;
% %             X1=[x(i) y(j)];
% %             p(aux)= prob_landmark(L,c,pos,X1);
% %         end
% %     end
% %     aux=0;
% %     p=p/max(p);
% %     for i=1:length(x)
% %         for j=1:length(y)
% %             aux=aux+1;
% %             if p(aux)>0.1
% %                 plot(x(i),y(j),'.','Color',[1 1 1]*p(aux))
% %             end
% %         end
% %     end
% %     scatter(pos(1),pos(2),600,'s','c','filled')
%     hold off
%     %     pause(0.1)
%     if (create_mov == true)
%         for i=1:fps
%             F = getframe(gcf);
%             writeVideo(mov,F);
%         end
%     end
%     pos(1)=pos(1)-35;
% end
%
% if (create_mov == true)
%     close(mov);
%     %     close all;
% end
