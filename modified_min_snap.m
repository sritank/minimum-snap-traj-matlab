% clear all
% clc

mp = motion_planner;
mp_plus_hg = motion_planner;

step_size=1;
alpha = 0.1;
threshold = alpha*1e-3;
iter_max=300; iter=1;
%waypoints
% load waypos;
% waypoints(1, :) = [0.0, 0.0, 0.5];
% waypoints(2, :) = [0.6, -0.6, 1];
% waypoints(3, :) = [0.0, 0.6, 3];
% waypoints(4, :) = [-0.6, -0.6, 6.0];
% waypoints(5, :) = [1.0, 0.0, 4.0];
% waypoints(6, :) = [2.0, 1.0, 4.0];
% waypoints(7, :) = [1.0, 3.0, 2.0];
% waypoints = waypos;
waypoints = pos(:,[1:2:N])';
% traj_num = size(waypos,1)-1;
traj_num = size(waypoints,1)-1;

% waypoints(1, :) = [0.0, 0.0, 0.0];
% waypoints(2, :) = [1 0 0];
% waypoints(3, :) = [1 2 0];
% waypoints(4, :) = [0 2 0];

%trajectory flight times
% traj_flight_times(1) = 1;
% traj_flight_times(2) = 1;
% traj_flight_times(3) = 1;
% traj_flight_times(4) = 1;
% traj_flight_times(5) = 1;
% traj_flight_times(6) = 1;

traj_flight_times = 6*ones(1,traj_num);
% traj_flight_times = [5.4664    3.9146    5.1870    4.6538    3.4244    7.3537];
% traj_flight_times = [6.5 7 6.5];

G = -1/(traj_num-1)*ones(traj_num,traj_num);
h=1e-6;
for i=1:traj_num
    G(i,i) = 1;
end
% traj_flight_times = [3 6 6];
mp = init(mp, waypoints, traj_flight_times, traj_num, 0);

traj_flight_times_i1 = traj_flight_times;
mp = plan_trajectory(mp);

while(step_size>threshold)
    for i=1:traj_num
        mp_plus_hg = init(mp_plus_hg, waypoints, traj_flight_times+G(i,:)*h, traj_num, 0);
        mp_plus_hg = plan_trajectory(mp_plus_hg);
        del_fT = (mp_plus_hg.fval - mp.fval)/h;
        traj_flight_times_i1 = traj_flight_times_i1 - del_fT*alpha*G(i,:);       
    end
    step_size = norm(traj_flight_times_i1-traj_flight_times,2);
    traj_flight_times = traj_flight_times_i1;
    mp = init(mp, waypoints, traj_flight_times, traj_num, 0);
    mp = plan_trajectory(mp);
    iter=iter+1;
%     mp.fval
    step_size
    if iter>iter_max
        disp('max iterations reached')
        break
    end
end
eta=0.98;
ubar=10.5;
while(1)
    [time_arr, x_acc_arr, y_acc_arr, z_acc_arr] = acc_trajectories(mp);
    
    if max(sqrt(x_acc_arr.^2 + y_acc_arr.^2 + (z_acc_arr+9.81).^2)>ubar)
        break
    end
    traj_flight_times = traj_flight_times*eta;
    mp = init(mp, waypoints, traj_flight_times, traj_num, 0);
    mp = plan_trajectory(mp);
    

end


[time_arr, x_acc_arr, y_acc_arr, z_acc_arr, x_vel_arr,  y_vel_arr,  z_vel_arr, x_traj_arr,y_traj_arr,z_traj_arr] = acc_vel_pos_trajectories(mp);
trim_length=12;

time_arr(end-trim_length:end)=[];
x_acc_arr(end-trim_length:end)=[];
y_acc_arr(end-trim_length:end)=[];
z_acc_arr(end-trim_length:end)=[];
x_vel_arr(end-trim_length:end)=[];
y_vel_arr(end-trim_length:end)=[];
z_vel_arr(end-trim_length:end)=[];
x_traj_arr(end-trim_length:end)=[];
y_traj_arr(end-trim_length:end)=[];
z_traj_arr(end-trim_length:end)=[];

% plot_trajectories(mp);
% figure
% plot(x_acc_arr)

% mp.fval





