clear
close all

opt=1;

%% Create Movie
% create a movie from plot
create_mov  = false;
if opt==1
    mov_name    = 'mcl_known.avi';
else
    mov_name    = 'mcl_unknown.avi';
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
Zi=[0 -8 0]';        %[x y phi];
Ui=[0 0];
L(:,1)=[-10 10];
L(:,2)=[-10 -10];
L(:,3)=[10 -10];
L(:,4)=[10 10];
col=[0 0 1;1 0 1;1 1 0;1 0 0];


Sigma=[10 0.1;0.1 10]*0.1;  %model uncertainty
R=[0.5 0 0;0 0.1 0;0 0 0.3];  %measurement uncertainty
Z=Zi;
U=[3 0.5;3 0.5;3 0.5;3 0.5;3 0.5];  %[v w]
U=[U;U;U]*4;%% Generate Initial Particles
M=1000;
X=zeros(length(Z),M);
P=zeros(M,1);
for i=1:M
    X(:,i)=[rand(1)*20-10 rand(1)*20-10 rand(1)*2*pi-pi]';
end
%% Plot First Step
figure(1)
plot(X(1,:),X(2,:),'.k')
xlim([-10 10])
ylim([-10 10])
hold on
grid on
scatter(L(1,:),L(2,:),100,col,'filled')
scatter(Z(1,1),Z(2,1),200,'filled','g')
hold off
pause()

if (create_mov == true)
    for i=1:fps
        F = getframe(gcf);
        writeVideo(mov,F);
    end
end

%% Iterate

for n=1:size(U,1)
    %%%%%%%%%%%%%%%%%%%%%% drive %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    for j=1:M
        X(:,j) = unicycle_model_sample(X(:,j),U(n,:),dt);
        %         X(:,j) = sample_odometry(X_old,Xbar,Xbar_old)
    end
    Z=unicycle(U(n,:),Z',dt);
    
    
    %plot
    plot(X(1,:),X(2,:),'.k')
    xlim([-10 10])
    ylim([-10 10])
    hold on
    grid on
    scatter(L(1,:),L(2,:),100,col,'filled')
    scatter(Z(1,1),Z(2,1),200,'filled','g')
    hold off
    pause(0.1)
    
    if (create_mov == true)
        for i=1:fps
            F = getframe(gcf);
            writeVideo(mov,F);
        end
    end
    
    %%%%%%%%%%%%%%%%%%%% measure %%%%%%%%%%%%%%%%%
    
    m = sense_unknown_landmark_clean(L,Z);
    %     for j=1:M
    P=zeros(M,1);
    for i=1:M
        %             index=(j-1)*M+i;
        m_hat = sense_unknown_landmark_clean(L,X(:,i));
        %             m_hat= sense_known_landmark(L,X(:,i));
        
        if opt==1
            for k=size(L,2)
                P(i) = P(i) + gaussian_multi(m_hat(k,1:2)',m(k,:)',R(1:2,1:2))*gaussian_multi(X(3,i),Z(3),0.3);
            end
        else
            prob_land=zeros(size(L,2),1);
            for k=1:size(L,2)
                for l=1:size(L,2)
                    prob_land(l) = prob_land(l)+gaussian_multi(m_hat(l,:)',m(k,:)',R(1:2,1:2))*gaussian_multi(X(3,i),Z(3),0.3);
                end
                P(i) =P(i) + max(prob_land);
            end
        end
    end
    %         end
    %     end
    P1=ones(size(P));
    for j=1:M
        P1(j) = gaussian_multi(X(:,j),Z,R);  %check probabilities
    end
    %     P=P1./sum(P1).*P./sum(P);
    W=P./sum(P);  % adjust weights
    X = low_variance_sampler(X,W); % resample
    %    %plot
    plot(X(1,:),X(2,:),'.k')
    xlim([-10 10])
    ylim([-10 10])
    grid on
    hold on
    scatter(L(1,:),L(2,:),100,col,'filled')
    scatter(Z(1,1),Z(2,1),200,'filled','g')
    hold off
    pause(0.1)
    
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

