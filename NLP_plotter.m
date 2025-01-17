clear all
clc
% load Optim_planner_data

% l=length(X);
ubar = 10.5;

time_waypo=[];
mu_waypo = [];
sigma_waypo = [];
res_waypo = [];


waypoints = [31.2082,   61.8123,  121.9200;
            31.2082,  220.6041,  121.9200;
            31.2082,  379.3959,  121.9200;
            31.2082,  538.1877,  121.9200;
            127.0694,  538.1877,  121.9200;
            127.0694,  379.3959,  121.9200;
            127.0694,  220.6041,  121.9200;
            127.0694,   61.8123,  121.9200;
            222.9306,   61.8123,  121.9200;
            222.9306,  220.6041,  121.9200;
            222.9306,  379.3959,  121.9200;
            222.9306,  538.1877,  121.9200;
            318.7918,  538.1877,  121.9200;
            318.7918,  379.3959,  121.9200;
            318.7918,  220.6041,  121.9200;
            318.7918,   61.8123,  121.9200];
    

trajectory = load('variableData.txt');
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

% modified_min_snap;

%% solving
for i=1:1:N-1%l-1

% xf = [X(1,i+1); X(2,i+1); X(3,i+1)]/ubar;
% x01 = [X(1,i); X(2,i); X(3,i)]/ubar;    
% v0 = [X(4,i); X(5,i); X(6,i)]/ubar;
% vf = [X(4,i+1); X(5,i+1); X(6,i+1)]/ubar;


% xf = [pos(1,i+1); pos(2,i+1); pos(3,i+1)]/ubar;
% x01 = [pos(1,i); pos(2,i); pos(3,i)]/ubar;    
% v0 = [vel(1,i); vel(2,i); vel(3,i)]/ubar;
% vf = [vel(1,i+1); vel(2,i+1); vel(3,i+1)]/ubar;

% xf = [pos(1,i+2); pos(2,i+2); pos(3,i+2)]/ubar;
x01 = [pos(1,i); pos(2,i); pos(3,i)]/ubar;    
v0 = [vel(1,i); vel(2,i); vel(3,i)]/ubar;
% vf = [vel(1,i+2); vel(2,i+2); vel(3,i+2)]/ubar;

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
%     x_n2 = [pos(1,i+1); pos(2,i+1); pos(3,i+1)] + [vel(1,i+1); vel(2,i+1); vel(3,i+1)]*[0:0.01:t_trajectory(i+1)] + (u_n2+g*ubar)*[0:0.01:t_trajectory(i+1)].^2/2;
    
    
    v_n1 = v0*ubar + (u_n1+g*ubar)*[0:0.01:t_trajectory(i)];
%     v_n2 = [vel(1,i+1); vel(2,i+1); vel(3,i+1)] + (u_n2+g*ubar)*[0:0.01:t_trajectory(i+1)];
    
    hold on
    plot3(x_n1(1,:),x_n1(2,:),x_n1(3,:),'--g','linewidth',2);
    scatter3(waypoints(:,1),waypoints(:,2),waypoints(:,3),'ob','linewidth',6)
%     g1 = plot3(x_n2(1,:),x_n2(2,:),x_n2(3,:),'--g','linewidth',2);
    hold off
    grid on
    
    fig2=figure(2)
    hold on
%     if i==1
    plot(t_num_total+[0:0.01:t_trajectory(i)],[0:0.01:t_trajectory(i)]*0+u_n1(1),'m','linewidth',2);
    plot(t_num_total+[0:0.01:t_trajectory(i)],[0:0.01:t_trajectory(i)]*0+u_n1(2),'g','linewidth',2);
    plot(t_num_total+[0:0.01:t_trajectory(i)],[0:0.01:t_trajectory(i)]*0+u_n1(3),'b','linewidth',2);
    plot(t_num_total+[0:0.01:t_trajectory(i)],[0:0.01:t_trajectory(i)]*0+norm(u_n1),'k','linewidth',2);
    
%     else
%         plot([t_trajectory(i-1):0.01:t_trajectory(i)],[t_trajectory(i-1):0.01:t_trajectory(i)]*0+u_n1,'--');
%     end
%     p1 = plot(t_num_total+[t_trajectory(i):0.01:t_trajectory(i)+t_trajectory(i+1)],[t_trajectory(i):0.01:t_trajectory(i)+t_trajectory(i+1)]*0+u_n2(1),'--m','linewidth',2);
%     p2 = plot(t_num_total+[t_trajectory(i):0.01:t_trajectory(i)+t_trajectory(i+1)],[t_trajectory(i):0.01:t_trajectory(i)+t_trajectory(i+1)]*0+u_n2(2),'--g','linewidth',2);
%     p3 = plot(t_num_total+[t_trajectory(i):0.01:t_trajectory(i)+t_trajectory(i+1)],[t_trajectory(i):0.01:t_trajectory(i)+t_trajectory(i+1)]*0+u_n2(3),'--b','linewidth',2);
%     p4 = plot(t_num_total+[t_trajectory(i):0.01:t_trajectory(i)+t_trajectory(i+1)],[t_trajectory(i):0.01:t_trajectory(i)+t_trajectory(i+1)]*0+norm(u_n2),'--k','linewidth',2);
    
    hold off
    grid on
    
    fig3=figure(3)
    hold on
%     if i==1
    plot(t_num_total+[0:0.01:t_trajectory(i)],v_n1(1,:),'m','linewidth',2);
    plot(t_num_total+[0:0.01:t_trajectory(i)],v_n1(2,:),'g','linewidth',2);
    plot(t_num_total+[0:0.01:t_trajectory(i)],(v_n1(1,:).^2+(v_n1(2,:).^2)).^0.5,'k','linewidth',2);
%     plot(t_num_total+[0:0.01:t_trajectory(i)],v_n1(3,:),'b','linewidth',2);
%     plot(t_num_total+[0:0.01:t_trajectory(i)],[0:0.01:t_trajectory(i)]*0+norm(u_n1),'k','linewidth',2);
    
%     else
%         plot([t_trajectory(i-1):0.01:t_trajectory(i)],[t_trajectory(i-1):0.01:t_trajectory(i)]*0+u_n1,'--');
%     end
%     p9 = plot(t_num_total+[t_trajectory(i):0.01:t_trajectory(i)+t_trajectory(i+1)],v_n2(1,:),'--m','linewidth',2);
%     p10 = plot(t_num_total+[t_trajectory(i):0.01:t_trajectory(i)+t_trajectory(i+1)],v_n2(2,:),'--g','linewidth',2);
%     p11 = plot(t_num_total+[t_trajectory(i):0.01:t_trajectory(i)+t_trajectory(i+1)],v_n2(3,:),'--b','linewidth',2);
%     p12 = plot(t_num_total+[t_trajectory(i):0.01:t_trajectory(i)+t_trajectory(i+1)],[t_trajectory(i):0.01:t_trajectory(i)+t_trajectory(i+1)]*0+norm(u_n2),'--k','linewidth',2);
    hold off
    grid on
    
    t_num_total = t_num_total + t_trajectory(i);% + t_trajectory(i+1);
%     t_anal_total = t_anal_total + tf;
    

% end
end
