classdef motion_planner
    properties
        PLOT_TIMES_PER_SECOND = 50;
        ITERATION_TIMES;
        
        traj_size;
        flight_times;
        
        x_traj_coeffs;
        x_waypoints;
        
        y_traj_coeffs;
        y_waypoints;
        
        z_traj_coeffs;
        z_waypoints;
        fval;
    end
    
    methods
        function ret_obj = init(obj, waypoints, flight_times, traj_size,fval)
            traj_gen = trajectory_generator;
            
            obj.traj_size = traj_size;
            obj.flight_times = flight_times;
            
            obj.x_waypoints = waypoints(1:traj_size+1, 1);
            obj.y_waypoints = waypoints(1:traj_size+1, 2);
            obj.z_waypoints = waypoints(1:traj_size+1, 3);
            obj.fval = fval;
            
            ret_obj = obj;
        end
        
        function ret_obj = plan_trajectory(obj)
            traj_gen = trajectory_generator;
            
            %size(obj.x_waypoints)
            %size(obj.flight_times)
            %disp(obj.traj_size)
            
            %trajectory planning
            [obj.x_traj_coeffs, xfval] = traj_gen.plan_optimized_segment(obj.x_waypoints, obj.flight_times, obj.traj_size);
            [obj.y_traj_coeffs, yfval] = traj_gen.plan_optimized_segment(obj.y_waypoints, obj.flight_times, obj.traj_size);
            [obj.z_traj_coeffs, zfval] = traj_gen.plan_optimized_segment(obj.z_waypoints, obj.flight_times, obj.traj_size);
            
            %disp(obj.x_traj_coeffs);
            %disp(obj.y_traj_coeffs);
            %disp(obj.z_traj_coeffs);
            obj.fval = xfval + yfval + zfval;
            
            ret_obj = obj;
        end
        
        function [traj_arr, vel_arr, acc_arr, time_arr]=plot_1d_trajectory(obj, traj_coeff)
            traj_gen = trajectory_generator;

            traj_arr = zeros(1, obj.ITERATION_TIMES);
            acc_arr = zeros(1, obj.ITERATION_TIMES);
            vel_arr = zeros(1, obj.ITERATION_TIMES);
            time_arr = zeros(1, obj.ITERATION_TIMES);
            
            %plot trajectories
            elapsed_index = 0;
            for i = 1: obj.traj_size
                %calculate i-th trajectory plot times and time step
                traj_plot_times = obj.flight_times(i) * obj.PLOT_TIMES_PER_SECOND;
                time_step = obj.flight_times(i) / traj_plot_times;
                
                %plot i-th trajectory
                for j = 1: traj_plot_times
                    %update trajectory array
                    traj_arr(elapsed_index + j) = traj_gen.calc_7th_polynomial(traj_coeff(i, :), (j-1) * time_step);
                    vel_arr(elapsed_index + j) = traj_gen.calc_6th_polynomial(traj_coeff(i, :), (j-1) * time_step);
                    acc_arr(elapsed_index + j) = traj_gen.calc_5th_polynomial(traj_coeff(i, :), (j-1) * time_step);
                    %update elapsed time array
                    time_arr(elapsed_index + j) = (elapsed_index + j - 1) * time_step;
                end
                
                %accumlate total elapsed time
                elapsed_index = elapsed_index + floor(traj_plot_times);
            end
        end
        
        function plot_trajectories(obj)
            traj_gen = trajectory_generator;
            total_flight_time = traj_gen.get_total_flight_time(obj.flight_times, obj.traj_size);
            obj.ITERATION_TIMES = uint16(total_flight_time * obj.PLOT_TIMES_PER_SECOND);
            
            [x_traj_arr, ~,x_acc_arr, time_arr] = plot_1d_trajectory(obj, obj.x_traj_coeffs);
            [y_traj_arr, ~,y_acc_arr, time_arr] = plot_1d_trajectory(obj, obj.y_traj_coeffs);
            [z_traj_arr, ~,z_acc_arr, time_arr] = plot_1d_trajectory(obj, obj.z_traj_coeffs);
%             x_traj_arr(end)=[];
%             y_traj_arr(end)=[];
%             z_traj_arr(end)=[];
%             time_arr(end)=[];
            mask = find(~x_traj_arr);
            mask1 = find(~y_traj_arr);%==0;
            mask = intersect(mask,mask1);%mask(mask==mask1);
            mask1 = find(~z_traj_arr);
            mask = intersect(mask,mask1);%mask(mask==mask1);
            x_traj_arr(mask)=[];
            y_traj_arr(mask)=[];
            z_traj_arr(mask)=[];
            time_arr(mask)=[];
                        
            figure('Name', 'trajectory');
            %
            subplot (3, 1, 1);
            plot(time_arr, x_traj_arr);
            xlabel('time [s]');
            ylabel('x position [m]');
            grid on;
            %
            subplot (3, 1, 2);
            plot(time_arr, y_traj_arr);
            xlabel('time [s]');
            ylabel('y position [m]');
            grid on;
            %
            subplot (3, 1, 3);
            plot(time_arr, z_traj_arr);
            xlabel('time [s]');
            ylabel('z position [m]');
            grid on;

            figure('Name', 'trajectory x-y');
            plot(x_traj_arr, y_traj_arr);
            grid on;
            
            figure('Name', 'trajectory x-y-z');
            plot3(x_traj_arr, y_traj_arr, z_traj_arr);
            grid on;
            
%             pause;
%             close all;
        end
        
        function [time_arr, x_acc_arr, y_acc_arr, z_acc_arr] = acc_trajectories(obj)
            traj_gen = trajectory_generator;
            total_flight_time = traj_gen.get_total_flight_time(obj.flight_times, obj.traj_size);
            obj.ITERATION_TIMES = uint16(total_flight_time * obj.PLOT_TIMES_PER_SECOND);
            
            [~, ~, x_acc_arr, time_arr] = plot_1d_trajectory(obj, obj.x_traj_coeffs);
            [~, ~, y_acc_arr, time_arr] = plot_1d_trajectory(obj, obj.y_traj_coeffs);
            [~, ~, z_acc_arr, time_arr] = plot_1d_trajectory(obj, obj.z_traj_coeffs);
            
        end
        
        function [time_arr, x_acc_arr, y_acc_arr, z_acc_arr, x_vel_arr,  y_vel_arr,  z_vel_arr, x_traj_arr,y_traj_arr,z_traj_arr] = acc_vel_pos_trajectories(obj)
            traj_gen = trajectory_generator;
            total_flight_time = traj_gen.get_total_flight_time(obj.flight_times, obj.traj_size);
            obj.ITERATION_TIMES = uint16(total_flight_time * obj.PLOT_TIMES_PER_SECOND);
            
            [x_traj_arr, x_vel_arr, x_acc_arr, time_arr] = plot_1d_trajectory(obj, obj.x_traj_coeffs);
            [y_traj_arr, y_vel_arr, y_acc_arr, time_arr] = plot_1d_trajectory(obj, obj.y_traj_coeffs);
            [z_traj_arr, z_vel_arr, z_acc_arr, time_arr] = plot_1d_trajectory(obj, obj.z_traj_coeffs);
            
        end
        
        
    end
end