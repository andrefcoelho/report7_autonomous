clear
close all

%Options
opt=2;    %measurement models: 1-feature based known correspondences 2-unknown correspondences


%% Create Movie
% create a movie from plot
create_mov  = false;
if opt==1
    mov_name    = 'grid_loc_known.avi';
else
    mov_name    = 'grid_loc_known.avi';
end
fps         = 2;

if (create_mov == true)
    mov             = VideoWriter(mov_name);
    mov.FrameRate   = fps;
    mov.Quality     = 100;
    open(mov);
end

%% Initial Parameters
dt=0.1;
Zi=[0 0 0]';        %[x y phi];
Ui=[0 0];
L(:,1)=[-10 10];
L(:,2)=[-10 -10];
L(:,3)=[10 -10];
L(:,4)=[10 10];
col=[0 0 1;1 0 1;1 1 0;1 0 0];


Sigma=[10 0.1;0.1 10]*0.1;  %model uncertainty
R=[1.5 0.001 0.01;0.001 1.5 0.001;0.01 0.001 1.5]*0.5;  %measurement uncertainty
Z=Zi;
U=[3 0.5;3 0.5;3 0.5;3 0.5;3 0.5;3 0.5; 3 0.5];  %[v w]
U=[U; U]*2;
%% Generate Initial Grid
M=21;
x=linspace(-10,10,M);
y=linspace(-10,10,M);
phi=linspace(-pi,pi,M);
phi=Zi(3);
index=0;
for k=1:1%M
    for j=1:M
        for i=1:M
            index=index+1;
            X(:,index)=[x(i);y(j);phi(k)];
        end
    end
end
% X=[x;y;phi];
P=ones(size(X,2),1);
P=P/sum(sum(P));
Xxy=X(1:2,1:M^2);
Pxy=ones(size(Xxy,2),1);
Pxy=Pxy/sum(sum(Pxy));
%% Plot First Step

figure(1)
xlim([-12 12])
ylim([-12 12])

for j=1:M
    for i=1:M
        c=(1-Pxy(M*(j-1)+i)/max(Pxy));
        scatter(x(i),y(j),50,c*[1 1 1],'filled')
        grid on
        hold on
    end
end
scatter(L(1,:),L(2,:),100,col,'filled')
scatter(Z(1,1),Z(2,1),200,'filled','g')
hold off
pause(0.2)

if (create_mov == true)
    for i=1:fps
        F = getframe(gcf);
        writeVideo(mov,F);
    end
end
%% Iterate
for n=1:size(U,1)
    
    Pxy_prev=Pxy;
    Pxy_new=zeros(size(Pxy));
    
    
    %%%%%%%%%%%%%%%%%%%%%%    measure   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    m = sense_unknown_landmark(L,Z(1:2));
    for j=1:M
        for i=1:M
            index=(j-1)*M+i;
            m_hat = sense_unknown_landmark_clean(L,Xxy(:,index));
            %         Xxy(:,index)=[x(i);y(j);phi(k)];
            if opt==1
                for k=1:size(L,2)
                    Pxy_new(index) = Pxy_new(index) + gaussian_multi(m_hat(k,:)',m(k,:)',R(1:2,1:2));
                end
            else
                prob_land=zeros(size(L,2),1);
                for k=1:size(L,2)
                    for l=1:size(L,2)
                        prob_land(l) = prob_land(l)+gaussian_multi(m_hat(l,:)',m(k,:)',R(1:2,1:2));
                    end
                    Pxy_new(index) =Pxy_new(index) + max(prob_land);
                end
            end
        end
    end
    Pxy_new=Pxy_new/sum(sum(Pxy_new));
    Pxy=Pxy_new.*Pxy_prev;
    Pxy=Pxy/sum(sum(Pxy));
    
    
    for j=1:M
        for i=1:M
            c=0.9*(1-Pxy(M*(j-1)+i)/max(Pxy));
            scatter(x(i),y(j),50,c*[1 1 1],'filled')
            grid on
            hold on
        end
    end
    scatter(L(1,:),L(2,:),100,col,'filled')
    scatter(Z(1,1),Z(2,1),200,'filled','g')
    hold off
    pause(0.2)
    if (create_mov == true)
        for i=1:fps
            F = getframe(gcf);
            writeVideo(mov,F);
        end
    end
    
    
    
    %%%%%%%%%%%%%%%%%%%%%%%   drive    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %     Z = diff_drive(Z,U(n,:),dt);
    phi_old=Z(3);
    Z=unicycle(U(n,:),Z',dt);
    %     for j=1:M
    %         for i=1:M
    %             index=(j-1)*M+i;
    %             %         Xxy(:,index)=[x(i);y(j);phi(k)];
    %             Pxy_prev(index) = gaussian_multi(Xxy(:,index),Z(1:2),R(1:2,1:2));
    %         end
    %     end
    
    Pxy_new=zeros(size(Pxy));
    
    for i=1:length(Pxy)
        mu1 = unicycle_model([X(1:2,i);phi_old],U(n,:),dt);
        for j=1:length(Pxy)
            Pxy_new(j) = Pxy_new(j)+Pxy(i)*gaussian_multi(Xxy(:,j),mu1(1:2),Sigma(1:2,1:2));
        end
    end
    
    Pxy_new=Pxy_new/sum(sum(Pxy_new));
    Pxy=Pxy_new;%.*Pxy_prev; %%%%%%%%%%%%%
    Pxy=Pxy/sum(sum(Pxy));
    
    for j=1:M
        for i=1:M
            c=0.9*(1-Pxy(M*(j-1)+i)/max(Pxy));
            scatter(x(i),y(j),50,c*[1 1 1],'filled')
            grid on
            hold on
        end
    end
    scatter(L(1,:),L(2,:),100,col,'filled')
    scatter(Z(1,1),Z(2,1),200,'filled','g')
    hold off
    pause(0.2)
    if (create_mov == true)
        for i=1:fps
            F = getframe(gcf);
            writeVideo(mov,F);
        end
    end
end

if (create_mov == true)
    close(mov);
    %     close all;
end


