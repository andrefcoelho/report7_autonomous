clear
close all

%% Create Movie
% create a movie from plot
create_mov  = false;
mov_name    = 'particle_filter.avi';
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
R=[0.1 0.001 0.001;0.001 0.1 0.001;0.001 0.001 1];  %measurement uncertainty
Z=Zi;
U=[10 10;0 20;10 10;20 0;20 0;20 0; 10 10];  %[v_left v_right] %rps
U=[U; U];
%% Generate Initial Particles
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
scatter(Z(1,1),Z(2,1),100,'filled','g')
hold off
pause(1)

if (create_mov == true)
    for i=1:fps
        F = getframe(gcf);
        writeVideo(mov,F);
    end
end

%% Iterate

for i=1:size(U,1)
    %drive
    for j=1:M
        X(:,j) = diff_drive(X(:,j),U(i,:),dt);
    end
    r = 0.2*randn(size(X));
    X=X+r;
    Z = diff_drive(Z,U(i,:),dt);
    
    %plot
    plot(X(1,:),X(2,:),'.k')
    xlim([-10 10])
    ylim([-10 10])
    hold on
    grid on
    scatter(Z(1,1),Z(2,1),100,'filled','g')
    hold off
    pause(1)
    
    if (create_mov == true)
        for i=1:fps
            F = getframe(gcf);
            writeVideo(mov,F);
        end
    end
    
    
    for j=1:M
        P(j) = gaussian_multi(X(:,j),Z,R);  %check probabilities
    end
    W=P./sum(P);  % adjust weights
    X = low_variance_sampler(X,W); % resample
    %    %plot
    plot(X(1,:),X(2,:),'.k')
    xlim([-10 10])
    ylim([-10 10])
    grid on
    hold on
    scatter(Z(1,1),Z(2,1),100,'filled','g')
    hold off
    pause(1)
    
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

