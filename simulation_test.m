clear;clc;close all;
%% initialize start conditions
% define model target point
% initial conditions current position and velocity and distance towards
% target point 
global max_acc_v;
global max_dec_v;
target_point = 15;
t = 0;

%--------- set up all the robot enviroments----------
%set the robot state 
robot_state = struct('cur_p', 0, 'cur_v', 0, 'target_point', target_point);

total_dist = target_point - robot_state.cur_p;
remain_dist = total_dist;

% import all the robot parameter from csv file
file_name = 'params_adjustment.csv';
params_table = readtable(file_name);
selected_row = 9;
selected_params = params_table(selected_row - 1, 1:14);
robot_params = table2struct(selected_params);

% generate obstacle and set obstacle state
obs_point = genObs(target_point, false);
obstacle_params = struct('obs_point', obs_point, 'obstacle_avoidance_acc', 1.51111);


%----------set up all the necessary variables---------
front_dist_prev = robot_state.target_point - robot_state.cur_p;
std_dev_v = 0.001;
std_dev_t = 0.0005;
dist_mark = 0;
t_mark = 0;
t_dec_mark = 0;
v_mark = 0;
% model
max_v_inque = robot_params.max_v_;
max_acc_v = robot_params.max_acc_v_;
max_dec_v = robot_params.max_dec_v_;
gama_v = [];
gama_t = [];
t_gama = 0;
global dec_list;
global dec_a_temp;
dec_list = [];
original_tarv = [];
itercount = [];
vlist = [];
dlist = [];
curr_pos_list = [];
iter = 0;
max_acc_v_list = [];
max_dec_v_list = [];
insert_status = false;
insert = control_stage.acc;
insert_row = 12;
obs_presence = true;
%%
%------now starts the moving process------
while true
    %cal front distance 
    front_dist = robot_state.target_point - robot_state.cur_p;
   
    
    
    %if front distance is larger than 3.5 set it to 3.5 this is for p(PID)
    %control 
    if abs(front_dist) > 3.5
        delta_s = 3.5;
    else
        delta_s = abs(front_dist);
    end
    
    
    %this block of code is used to test inserting different robot_params in
    %different control stage
    if insert_status
        switch(insert)
        case 'acc'
            if remain_dist/total_dist <= 0.8
                selected_params = params_table(insert_row - 1, 1:14);
                robot_params = table2struct(selected_params);        
            end
        case 'dec_first'
            if remain_dist/total_dist <= (3.5/target_point)
                selected_params = params_table(insert_row - 1, 1:14);
                robot_params = table2struct(selected_params);        
            end
        case 'dec_sec'
            if remain_dist/total_dist < (0.2/target_point)
                selected_params = params_table(insert_row - 1, 1:14);
                robot_params = table2struct(selected_params);        
            end
        end
    end
    
    
    
    %now we can update the acceleration and decceleration here
    %setNormalAcc(robot_params,cur_v);
    max_acc_v = robot_params.acc_init - robot_params.acc_incre *...
                abs((robot_params.max_v_/2 -robot_state.cur_v) / (robot_params.max_v_/2));
    max_dec_v = robot_params.dec_init + (robot_params.dec_incre + 0.25) *...
                ((2.0 * (robot_params.max_v_ / (robot_params.max_v_ + abs(robot_state.cur_v))))^1.5);
    max_acc_v_list = [max_acc_v_list, max_acc_v];
    max_dec_v_list = [max_dec_v_list, max_dec_v];
    dec_list = [dec_list, max_dec_v];
    
    %in case there is an obstacle, we set obstacle avoidance acc/dec here
    if obs_presence
        [robot_params.max_v_, robot_params.max_acc_v_, robot_params.max_dec_v_] =...
            setObstacleAvoidanceAcc(robot_params, obstacle_params, robot_state);
    end
    
    % update max v for dec-start and cal prerequisites for the
    % decceleration such as least distance for current speed
    %for max_v_inque why not just picking the cur_v as the inital decc
    %velocity 
    max_v_inque = max(robot_state.cur_v, max_v_inque);
    deacc_redundent = (1 / robot_params.dec_init) + 0.5;
    dec_start = max_v_inque * max_v_inque / (2 * robot_params.dec_init) + 0.05;
    
    %now enter into firstrange control state 0.4m is a speration point
    if abs(front_dist) > 0.4
        tar_v = delta_s * robot_params.k_dist;
        original_tarv = [original_tarv, tar_v];
        %this is to supress the velocity shooting too high        
        tar_v = min(robot_params.max_v_, tar_v);
        
        %this is to make sure when we need to decelerate we can do it
        %safely
        
        
        
        if abs(front_dist - dec_start) < 0.01
            t_dec_mark = t;
        end
        if abs(front_dist) < dec_start
            v_dec_temp = sqrt(abs(max_v_inque^2 - 2 * robot_params.dec_init * abs((abs(front_dist) - dec_start))));
            tar_v = min(tar_v, v_dec_temp);        
        end              
        %if we are surpassing the target point we go back
        if robot_state.cur_p > robot_state.target_point
            tar_v = -tar_v;
        end
        
        %linearTrapezoid is to make sure the tar_v is within the reach of
        %current speed and accleration limit
        tar_v = linearTrapezoid(robot_params, robot_state, tar_v);
        
%         if tar_v > 1.0 && tar_v < 1.3
%             tar_v = 0.5;
%         end
        %assign current v to tar_v_pre(previous)
        tar_v_pre = tar_v;
        %cal the decceleration val with the tar_v_pre set the minimum
        %required dis is 0.4????
        dec_a_temp = tar_v_pre^2/0.8;
        
        %now assign the current v to tar_v_out which will be used in else
        %condition
        tar_v_out = tar_v;
        
        %what is this temp_dist and tar_v_virtual; what is gamma????
        temp_dist = abs(0.399+0.5*(abs(0.399/0.4)))^robot_params.gamma;  
        tar_v_virtual =  robot_params.k_v_short * temp_dist;
    %here we enter into second range control which is smaller than 0.4
    else
        % this portion of the control is to cover the case when the robot
        % moves towards the end of the target point temp_dist is a manuel
        % made function to relate the current position with the velocity 
        temp_dist = abs(remain_dist+0.5*(abs(remain_dist/0.4)))^robot_params.gamma;
        %here k_v_short is 0.302
        tar_v_k =  robot_params.k_v_short * temp_dist;
        
        
        %here set a condition when the whole dist which is target pos -
        %original robot pos what is 0.28 here????? hard coded?
        if abs(target_point) > 0.4 && tar_v_out>0.28
            tar_v = tar_v - dec_a_temp * robot_params.m_sampling_time;
            tar_v = min(tar_v,tar_v_k);
        else
            tar_v = tar_v_k;
        end
        %assign current target v to target v previous
        tar_v_pre = robot_state.cur_v;
        
        gama_v = [gama_v, tar_v];
        t_gama = t_gama + robot_params.m_sampling_time;
        gama_t = [gama_t, t];
        %surpress the tar_v getting too large
        tar_v = min(robot_params.max_v_, tar_v);
        
        if robot_state.cur_p > target_point
            tar_v = -tar_v;
        end
        
        original_tarv = [original_tarv, tar_v];
        tar_v = linearTrapezoid(robot_params,robot_state, tar_v);
        
    end
    front_dist_prev = front_dist;
    
    %now that the target v has been picked with all the conditions
    %considered, one can assign it to cur_v confidently    
    robot_state.cur_v = tar_v;
    
    t_actual = robot_params.m_sampling_time;
    t = t + t_actual;
    delta_movement = robot_state.cur_v * t_actual;
    robot_state.cur_p = robot_state.cur_p + delta_movement;
    remain_dist = robot_state.target_point - robot_state.cur_p - delta_movement;    
    itercount = [itercount, t];
    %fprintf('%d\n',iter);
    if abs(remain_dist - 0.8)  < 0.01
        dist_mark = remain_dist;
        t_mark = t;
        v_mark = robot_state.cur_v;
    end
    
    if abs(remain_dist - 3.5) < 0.01
        t_mark_dec = t;
    end
        
    curr_pos_list = [curr_pos_list, robot_state.cur_p];
    dlist = [dlist, remain_dist];
    vlist = [vlist, robot_state.cur_v];
    iter = iter + 1;
    %now draw the output 
    if size(vlist,2) > 10 && sum(vlist(end-10:end)) <= 0 %iter>4000
        figure(1);
        subplot(3,1,1);
        plot(itercount, dlist);
        hold on;
        plot(itercount, curr_pos_list);
        
        yline(obstacle_params.obs_point - 3.2, '-', 'within 3.2m' );
        yline(obstacle_params.obs_point - 0.8, '-', 'within 0.8m' );
        yline(obstacle_params.obs_point, '-', 'obstacle point' );
        yline(dist_mark, '-', dist_mark );
        xline(t_mark, '-', ' point' );
        xline(t_dec_mark, '-', 'dec point' );
        title('remaining distance VS time');
        xlabel('time(s)');
        ylabel('remain distance (m)');
        subplot(3,1,2);
        plot(itercount, vlist);
        hold on;
        plot(itercount,original_tarv);
        hold on;
        xi = 0:0.01:t;
        yi = pchip(itercount,vlist,xi);
        plot(xi,yi,itercount,vlist);
        
%         plot(itercount, dec_list);
        xline(t_mark, '-', ' point' );
        xline(t_dec_mark, '-', 'dec point' );
        yline(v_mark, '-', v_mark );
        title('velocity VS time');        
        xlabel('time(s)');
        ylabel('velocity(m/s)');
        legend('actual v out','target v out')
        
        
        %start animation here
        subplot(3,1,3);
        
        h = plot(curr_pos_list(1), 0, 'o', 'MarkerSize' ,20, 'MarkerFaceColor', 'b');
        xline(obstacle_params.obs_point - 3.2, '-', 'within 3.2m' );
        xline(obstacle_params.obs_point - 0.8, '-', 'within 0.8m' );
        xline(obstacle_params.obs_point, '-', 'obstacle point' );
        grid on;
        xlim([0, target_point]);
        ylim([-1.5, 1.5]);
        xlabel('distance(m)');
        title('Animation'); 

        % Animation loop -- (3)
        for i = 1:length(itercount)
            set(h, 'XData', curr_pos_list(i));
            drawnow;
        end
        
        figure(2);
        plot(vlist, max_acc_v_list);
        hold on;
        plot(vlist, max_dec_v_list);
        xlabel('current velocity(m/s)');
        ylabel('acceleration/deceleration(m/s^2)');
        legend('max acc','max dec')
        break
    end 
end