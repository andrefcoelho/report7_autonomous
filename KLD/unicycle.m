function p=unicycle(U,p_init,dt)
v=U(1);
w=U(2);
t_sim = dt;
load_system('unicycle_control')   % load simulink model
input=[v w p_init];
% simulate the action of this input on the plant to produce the output signal
[t,~,output] = sim('unicycle_control',t_sim,[],[t_sim,input]);

p1 = output(end,1:3);
p=p1';
