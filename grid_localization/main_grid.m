clear
close all

%% Create Movie
% create a movie from plot
create_mov  = false;
mov_name    = 'gid_loc.avi';
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
Sigma=[1000 10 10;10 1000 10;10 10 1000];  %model uncertatinty
R=[1.5 0.001 0.01;0.001 1.5 0.001;0.01 0.001 1.5];  %measurement uncertainty
Z=Zi;
U=[10 10;0 20;10 10;20 0;20 0;20 0; 10 10];  %[v_left v_right] %rps
U=[U; U];
%% Generate Initial Grid
M=21;
x=linspace(-10,10,M);
y=linspace(-10,10,M);
phi=linspace(-pi,pi,M);
index=0;
for k=1:M
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
scatter(Z(1,1),Z(2,1),100,'filled','g')
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
    
    %%%%%%%%%%%%%%%%%%%%%%%measure%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for j=1:M
        for i=1:M
            index=(j-1)*M+i;
            %         Xxy(:,index)=[x(i);y(j);phi(k)];
            Pxy_new(index) = gaussian_multi(Xxy(:,index),Z(1:2),R(1:2,1:2));
        end
    end
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
    scatter(Z(1,1),Z(2,1),100,'filled','g')
    hold off
    pause(0.2)
    if (create_mov == true)
        for i=1:fps
            F = getframe(gcf);
            writeVideo(mov,F);
        end
    end
    
    
    
    %%%%%%%%%%%%%%%%%%%%%%% drive %%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    Z = diff_drive(Z,U(n,:),dt);
    for j=1:M
        for i=1:M
            index=(j-1)*M+i;
            %         Xxy(:,index)=[x(i);y(j);phi(k)];
            Pxy_prev(index) = gaussian_multi(Xxy(:,index),Z(1:2),R(1:2,1:2));
        end
    end
    Pxy_new=zeros(size(Pxy));
    
    for i=1:length(Pxy)
        mu1 = diff_drive(X(:,i),U(n,:),dt);
        for j=1:length(Pxy)
            Pxy_new(j) = Pxy_new(j)+gaussian_multi(Xxy(:,j),mu1(1:2),Sigma(1:2,1:2));
        end
    end
    
    
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
    scatter(Z(1,1),Z(2,1),100,'filled','g')
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


