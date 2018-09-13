clear
close all

opt=1;

%% Create Movie
% create a movie from plot
create_mov  = false;
if opt==1
    mov_name    = 'KLD.avi';
else
    mov_name    = 'KLD_unknown.avi';
end
fps         = 2;

if (create_mov == true)
    mov             = VideoWriter(mov_name);
    mov.FrameRate   = fps;
    mov.Quality     = 100;
    open(mov);
end

%% Generating Grid
x=-10:0.5:10;
y=-10:0.5:10;
phi=-pi:pi/18:pi;
index=0;
H=zeros(length(x)*length(y)*length(phi),3);
for i=1:length(x)
    for j=1:length(y)
        for k=1:length(phi)
            index=index+1;
            H(index,:)=[x(i) y(j) phi(k)];
        end
    end
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
M=10000;
X=zeros(length(Z),M);
P=ones(M,1);

%KLD parameters
% e=0.4;
e=0.55;
z1d=2.576;
% z1d=0.8389;
% z1d=0.6;
% % z1d=0.3
Mx=inf;
k=1;
bin=zeros(length(H),1);



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
pause(0.1)

if (create_mov == true)
    for i=1:fps
        F = getframe(gcf);
        writeVideo(mov,F);
    end
end

i_all=[];
%% Iterate

for n=1:size(U,1)
    Z=unicycle(U(n,:),Z',dt);
    m = sense_unknown_landmark_clean(L,Z);
    W=P./sum(P);  % adjust weights
    X = low_variance_sampler(X,W);
    P=zeros(M,1);
    bin=zeros(length(H),1);
    i=0;
    while i<Mx %for i=1:M
        i=i+1;
        %%%%%%%%%%%%%%%%%%%%%% drive %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        ri=rand*size(X,2);
        ri=max(1,round(ri));
        X1 = unicycle_model_sample(X(:,ri),U(n,:),dt);
        %%%%%%%%%%%%%%%%%%%% sense %%%%%%%%%%%%%%%%%
        P(i) = gaussian_multi(X1,Z,R);  %check probabilities
  
%         m_hat = sense_unknown_landmark_clean(L,X(:,i));
%         if opt==1
%             for k=size(L,2)
%                 P(i) = P(i) + gaussian_multi(m_hat(k,1:2)',m(k,:)',R(1:2,1:2))*gaussian_multi(X(3,i),Z(3),0.3);
%             end
%         else
%             prob_land=zeros(size(L,2),1);
%             for k=1:size(L,2)
%                 for l=1:size(L,2)
%                     prob_land(l) = prob_land(l)+gaussian_multi(m_hat(l,:)',m(k,:)',R(1:2,1:2))*gaussian_multi(X(3,i),Z(3),0.3);
%                 end
%                 P(i) =P(i) + max(prob_land);
%             end
%         end
        %%%%%%%%%%%%%%% KLD %%%%%%%%%%%%%%%%%%%%%%%%
        index=0;
        finish=0;
        while not(finish)
            index=index+1;
            if index>length(H)
                finish=1;
                break;
            end
            if all(X1'<=H(index,:))  % if X falls into bin
                if not(bin(index))    %if bin is empty
                    k=k+1;
                    bin(index)=1;
                    if k>=2
                        Mx=(k-1)/(2*e)*(1-2/(9*(k-1))+sqrt(2/(9*(k-1)))*z1d)^3;
                    end
                end
                finish=1;
            end 
        end
      X_new(:,i)=X1;  
    end
    X=X_new;
    i_all=[i_all length(X)];
        plot(X(1,:),X(2,:),'.k')
        xlim([-10 10])
        ylim([-10 10])
        hold on
        grid on
        scatter(L(1,:),L(2,:),100,col,'filled')
        scatter(Z(1,1),Z(2,1),200,'filled','g')
        hold off
        pause(0.1)
    
    %     
    %
        if (create_mov == true)
            for i=1:fps
                F = getframe(gcf);
                writeVideo(mov,F);
            end
        end
    
    
    
    
 
    
end


if (create_mov == true)
    close(mov);
end
%
