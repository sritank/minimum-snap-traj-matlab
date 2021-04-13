clear all
clc
% load Optim_planner_data

% l=length(X);
% ubar = 10.5;

time_waypo=[];
mu_waypo = [];
sigma_waypo = [];
res_waypo = [];

trajectory = load('variableData_latest.txt');
N=length(trajectory)/10;
pos = zeros(3,N);
vel = zeros(3,N);
acc = zeros(3,N);
t_trajectory = zeros(1,N);
% Acceleration
for i_trajectory=1:N
acc(1,i_trajectory) = trajectory(6*N + i_trajectory);
acc(2,i_trajectory) = trajectory(7*N + i_trajectory);
acc(3,i_trajectory) = trajectory(8*N + i_trajectory); %1 g acc. is removed before publishing to the px4 control topic
% Velocity
vel(1,i_trajectory) = trajectory(3*N + i_trajectory); %+ reference[6]*(t_elapsed.count()-t_trajectory[i_trajectory]);
vel(2,i_trajectory) = trajectory(4*N + i_trajectory); %+ reference[7]*(t_elapsed.count()-t_trajectory[i_trajectory]);
vel(3,i_trajectory) = trajectory(5*N + i_trajectory); %+ (reference[8]-9.8)*(t_elapsed.count()-t_trajectory[i_trajectory]);
% Position
pos(1,i_trajectory) = trajectory(0*N + i_trajectory); %+ trajectory[3*N + i_trajectory]*(t_elapsed.count()-t_trajectory[i_trajectory]) + 0.5*reference[6]*(t_elapsed.count()-t_trajectory[i_trajectory])*(t_elapsed.count()-t_trajectory[i_trajectory]);
pos(2,i_trajectory) = trajectory(1*N + i_trajectory); %+ trajectory[4*N + i_trajectory]*(t_elapsed.count()-t_trajectory[i_trajectory]) + 0.5*reference[7]*(t_elapsed.count()-t_trajectory[i_trajectory])*(t_elapsed.count()-t_trajectory[i_trajectory]);
pos(3,i_trajectory) = trajectory(2*N + i_trajectory); %+ trajectory[5*N + i_trajectory]*(t_elapsed.count()-t_trajectory[i_trajectory]) + 0.5*(reference[8]-9.8)*(t_elapsed.count()-t_trajectory[i_trajectory])*(t_elapsed.count()-t_trajectory[i_trajectory]);
%Time
t_trajectory(1,i_trajectory) = trajectory(9*N + i_trajectory);
end
ubar = max(sqrt(acc(1,:).^2+acc(2,:).^2+acc(3,:).^2));
% ubar=sqrt(ubar^2-9.8^2);

g = [0;0;-9.8];
g = g/ubar;%(2*9.81); %non-dimensionalizing with respect to max thrust.
% g=g*0;
t_anal_total = 0;
t_num_total = 0;

modified_min_snap;

%% solving
for i=1:2:N-1%l-1

% xf = [X(1,i+1); X(2,i+1); X(3,i+1)]/ubar;
% x01 = [X(1,i); X(2,i); X(3,i)]/ubar;    
% v0 = [X(4,i); X(5,i); X(6,i)]/ubar;
% vf = [X(4,i+1); X(5,i+1); X(6,i+1)]/ubar;


% xf = [pos(1,i+1); pos(2,i+1); pos(3,i+1)]/ubar;
% x01 = [pos(1,i); pos(2,i); pos(3,i)]/ubar;    
% v0 = [vel(1,i); vel(2,i); vel(3,i)]/ubar;
% vf = [vel(1,i+1); vel(2,i+1); vel(3,i+1)]/ubar;

xf = [pos(1,i+2); pos(2,i+2); pos(3,i+2)]/ubar;
x01 = [pos(1,i); pos(2,i); pos(3,i)]/ubar;    
v0 = [vel(1,i); vel(2,i); vel(3,i)]/ubar;
vf = [vel(1,i+2); vel(2,i+2); vel(3,i+2)]/ubar;

u_n1 = [acc(1,i); acc(2,i); acc(3,i)];
u_n2 = [acc(1,i+1); acc(2,i+1); acc(3,i+1)];
% x01 = [3,3,0]';
% xf = [2,2,0]';
% x01 = [1;1];
% x01 = [-1;1];
% xf = [0;0];
% mag = 2;
% % mag = 0.7;
% alpha = -0.57;

% A = [0.5 0.65 0.85 -0.8 -0.57 -0.35 0.25];
% A = [0.5 0.75 0.91 -0.8 -0.5 -0.24 0.25];
% for i=1:2:length(A)
% i=8;
%     alpha = A(i);
% v0 = [mag*cos(alpha*pi),mag*sin(alpha*pi), 0]';
%     v0 = [mag*cos(alpha*pi),mag*sin(alpha*pi)]';

%     vf = v0;%[-1;0];
%     vf = [-1;0];
%     Akulenko_3D
%     time_optim_gravity
    fig1=figure(1)
    
    x_n1 = x01*ubar + v0*ubar*[0:0.01:t_trajectory(i)] + (u_n1+g*ubar)*[0:0.01:t_trajectory(i)].^2/2;
    x_n2 = [pos(1,i+1); pos(2,i+1); pos(3,i+1)] + [vel(1,i+1); vel(2,i+1); vel(3,i+1)]*[0:0.01:t_trajectory(i+1)] + (u_n2+g*ubar)*[0:0.01:t_trajectory(i+1)].^2/2;
    
    
    v_n1 = v0*ubar + (u_n1+g*ubar)*[0:0.01:t_trajectory(i)];
    v_n2 = [vel(1,i+1); vel(2,i+1); vel(3,i+1)] + (u_n2+g*ubar)*[0:0.01:t_trajectory(i+1)];
    
    hold on
    plot3(x_n1(1,:),x_n1(2,:),x_n1(3,:),'--g','linewidth',2);
    g1 = plot3(x_n2(1,:),x_n2(2,:),x_n2(3,:),'--g','linewidth',2);
    hold off
    grid on
    
    fig2=figure(2)
    hold on
%     if i==1
    plot(t_num_total+[0:0.01:t_trajectory(i)],[0:0.01:t_trajectory(i)]*0+u_n1(1),'--m','linewidth',2);
    plot(t_num_total+[0:0.01:t_trajectory(i)],[0:0.01:t_trajectory(i)]*0+u_n1(2),'--g','linewidth',2);
    plot(t_num_total+[0:0.01:t_trajectory(i)],[0:0.01:t_trajectory(i)]*0+u_n1(3),'--b','linewidth',2);
    plot(t_num_total+[0:0.01:t_trajectory(i)],[0:0.01:t_trajectory(i)]*0+norm(u_n1),'--k','linewidth',2);
    
%     else
%         plot([t_trajectory(i-1):0.01:t_trajectory(i)],[t_trajectory(i-1):0.01:t_trajectory(i)]*0+u_n1,'--');
%     end
    p1 = plot(t_num_total+[t_trajectory(i):0.01:t_trajectory(i)+t_trajectory(i+1)],[t_trajectory(i):0.01:t_trajectory(i)+t_trajectory(i+1)]*0+u_n2(1),'--m','linewidth',2);
    p2 = plot(t_num_total+[t_trajectory(i):0.01:t_trajectory(i)+t_trajectory(i+1)],[t_trajectory(i):0.01:t_trajectory(i)+t_trajectory(i+1)]*0+u_n2(2),'--g','linewidth',2);
    p3 = plot(t_num_total+[t_trajectory(i):0.01:t_trajectory(i)+t_trajectory(i+1)],[t_trajectory(i):0.01:t_trajectory(i)+t_trajectory(i+1)]*0+u_n2(3),'--b','linewidth',2);
    p4 = plot(t_num_total+[t_trajectory(i):0.01:t_trajectory(i)+t_trajectory(i+1)],[t_trajectory(i):0.01:t_trajectory(i)+t_trajectory(i+1)]*0+norm(u_n2),'--k','linewidth',2);
    hold off
    grid on
    
    fig3=figure(3)
    hold on
%     if i==1
    plot(t_num_total+[0:0.01:t_trajectory(i)],v_n1(1,:),'--m','linewidth',2);
    plot(t_num_total+[0:0.01:t_trajectory(i)],v_n1(2,:),'--g','linewidth',2);
    plot(t_num_total+[0:0.01:t_trajectory(i)],v_n1(3,:),'--b','linewidth',2);
    plot(t_num_total+[0:0.01:t_trajectory(i)],[0:0.01:t_trajectory(i)]*0+norm(u_n1),'--k','linewidth',2);
    
%     else
%         plot([t_trajectory(i-1):0.01:t_trajectory(i)],[t_trajectory(i-1):0.01:t_trajectory(i)]*0+u_n1,'--');
%     end
    p9 = plot(t_num_total+[t_trajectory(i):0.01:t_trajectory(i)+t_trajectory(i+1)],v_n2(1,:),'--m','linewidth',2);
    p10 = plot(t_num_total+[t_trajectory(i):0.01:t_trajectory(i)+t_trajectory(i+1)],v_n2(2,:),'--g','linewidth',2);
    p11 = plot(t_num_total+[t_trajectory(i):0.01:t_trajectory(i)+t_trajectory(i+1)],v_n2(3,:),'--b','linewidth',2);
    p12 = plot(t_num_total+[t_trajectory(i):0.01:t_trajectory(i)+t_trajectory(i+1)],[t_trajectory(i):0.01:t_trajectory(i)+t_trajectory(i+1)]*0+norm(u_n2),'--k','linewidth',2);
    hold off
    grid on
    
    t_num_total = t_num_total + t_trajectory(i) + t_trajectory(i+1);
%     t_anal_total = t_anal_total + tf;
    

% end
end
% figure
fig2=figure(2)

% legend([p5,p6,p7,p8,p1,p2,p3,p4],'theo u_x', 'theo u_y', 'theo u_z','u bar theo', 'u_x numerical','u_y numerical','u_z numerical','u bar num');
% legend([p1,p2,p3,p4,p5,p6,p7], 'u_x numerical','u_y numerical','u_z numerical','u bar num');
legend([p1,p2,p3,p4], 'u_x numerical','u_y numerical','u_z numerical','u bar num');


xlim([0,30]);
t_traj = t_trajectory([1:2:end-2])+t_trajectory([2:2:end-1]);
xlabel('time(s)');
ylabel('input,u^*, (N)');
set(gca,'Fontsize',25)


fig1=figure(1)
hold on
g2 = plot3(x_traj_arr,y_traj_arr,z_traj_arr,'b','linewidth',2);
xlabel('X(m)');
ylabel('Y(m)');
legend([g1,g2],'NLP path','Modified min snap');
set(gca,'Fontsize',25)
axis equal
hold off

fig3 = figure(3)
hold on
p13 = plot(time_arr, x_vel_arr, 'm', 'linewidth', 2);
p14 = plot(time_arr, y_vel_arr, 'g', 'linewidth', 2);
p15 = plot(time_arr, z_vel_arr, 'b', 'linewidth', 2);
% legend([p13,p14,p15,p16,p9,p10,p11,p12,p17],'theo v_x', 'theo v_y', 'theo v_z','u bar theo', 'v_x numerical','v_y numerical','v_z numerical','u bar num','Hamiltonian');
% legend([p9,p10,p11,p12, p13, p14, p15],'v_x numerical','v_y numerical','v_z numerical','u bar num');
legend([p9, p10, p11, p13, p14, p15],'v_x numerical','v_y numerical','v_z numerical','v_x min snap','v_y min snap','v_z min snap');

xlim([0,30]);
xlabel('time(s)');
ylabel('vel, (m/s)');
set(gca,'Fontsize',25)
hold off


fig4 = figure(4)
bar([traj_flight_times;t_traj]')
hold on
% bar(t_traj)
legend('modified min snap','NLP solution')
xlabel('waypoint');
ylabel('time');
set(gca,'Fontsize',25)

