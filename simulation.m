clear;clc;close all;
%% initialize start conditions
% define model target point
% initial conditions current position and velocity and distance towards
% target point 
global target_point;
global cur_p;
global cur_v;
target_point = 7;
cur_p = 0;
cur_v = 0;
t = 0;
global whole_dist;
whole_dist = target_point - cur_p;
remain_dist = whole_dist;
max_v_inque = 0;

% data record lists
v_list = [];
t_list = [];
remain_dist_list = [];

global max_acc_v;
global max_dec_v;


robot_state = struct('cur_p', 0, 'cur_v', 0, 'target_point', target_point);


% import all the robot parameter from csv file
file_name = 'params_adjustment.csv';
params_table = readtable(file_name);
selected_params = params_table(1, :);
robot_params = table2struct(selected_params);

% generate obstacle in between current position and target position
obs_point = genObs(target_point, false);
obstacle_params = struct('obs_point', obs_point, 'obstacle_avoidance_acc', 1.51111);


%----------introduce standard deviation variables------
std_dev_v = 0.001;
std_dev_t = 0.0005;
% model
% setNormalAcc
max_v_inque = robot_params.max_v_;
max_acc_v = robot_params.max_acc_v_;
max_dec_v = robot_params.max_dec_v_;

gama_v = [];
gama_t = [];
t_gama = 0;

global front_dist;
global dec_list;
global dec_a_temp;
dec_list = [];
original_tarv = [];
itercount = [];
vlist = [];
dlist = [];
iter = 0;

while true
    %cal front distance 
    front_dist = target_point - cur_p;
    
    %if front distance is larger than 3.5 set it to 3.5 this is for p(PID)
    %control 
    if abs(front_dist) > 3.5
        delta_s = 3.5;
    else
        delta_s = abs(front_dist);
    end
    
    %now we can update the acceleration and decceleration here
    %setNormalAcc(robot_params,cur_v);
    max_acc_v = robot_params.acc_init - robot_params.acc_incre *...
                abs((robot_params.max_v_/2 -cur_v) / (robot_params.max_v_/2));
    max_dec_v = robot_params.dec_init + (robot_params.dec_incre + 0.25) *...
                ((2.0 * (robot_params.max_v_ / (robot_params.max_v_ + abs(cur_v))))^1.5);
    dec_list = [dec_list, max_dec_v];
    
    %in case there is an obstacle, we set obstacle avoidance acc/dec here
    [robot_params.max_v_, robot_params.max_acc_v_, robot_params.max_dec_v_] =...
        setObstacleAvoidanceAcc(robot_params, obstacle_params, robot_state);
    
    % update max v for dec-start and cal prerequisites for the
    % decceleration such as least distance for current speed
    %for max_v_inque why not just picking the cur_v as the inital decc
    %velocity 
    max_v_inque = max(cur_v, max_v_inque);
    deacc_redundent = (1 / robot_params.dec_init) + 0.5;
    dec_start = max_v_inque * max_v_inque / (2 * robot_params.dec_init) + 0.05;
    
    %now enter into firstrange control state 0.4m is a speration point
    if abs(front_dist) > 0.4
        tar_v = delta_s * robot_params.k_dist;
        original_tarv = [original_tarv, tar_v];
        %this is to make sure when we are need to decelerate we can do it
        %safely
        if abs(front_dist) < dec_start
            v_dec_temp = sqrt(abs(max_v_inque^2 - 2 * robot_params.dec_init * abs((abs(front_dist) - dec_start))));
            tar_v = min(tar_v, v_dec_temp);
        end
        
        %this is to supress the velocity shooting too high        
        tar_v = min(robot_params.max_v_, tar_v);
               
        %if we are surpass the target point we go back
        if cur_p > target_point
            tar_v = -tar_v;
        end
        
        %linearTrapezoid is to make sure the tar_v is with the reach of
        %accleration limit
        tar_v = linearTrapezoid(cur_v, tar_v);
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
        % what is this temp_dist? hard coded?
        temp_dist = abs(remain_dist+0.5*(abs(remain_dist/0.4)))^robot_params.gamma;
        %here k_v_short is 0.302
        tar_v_k =  robot_params.k_v_short * temp_dist;
        
        
        %here set a condition when the whole dist which is target pos -
        %original robot pos what is 0.28 here????? hard coded?
        if abs(whole_dist) > 0.4 && tar_v_out>0.28
            tar_v = tar_v - dec_a_temp * robot_params.m_sampling_time;
            tar_v = min(tar_v,tar_v_k);
        else
            tar_v = tar_v_k;
        end
        %assign current target v to target v previous
        tar_v_pre = cur_v;
        
        gama_v = [gama_v, tar_v];
        t_gama = t_gama + robot_params.m_sampling_time;
        gama_t = [gama_t, t];
        %surpress the tar_v getting too large
        tar_v = min(robot_params.max_v_, tar_v);
        
        if cur_p > target_point
            tar_v = -tar_v;
        end
        
        original_tarv = [original_tarv, tar_v];
        tar_v = linearTrapezoid(cur_v, tar_v);
        
    end
    
    %now that the target v has been picked with all the conditions
    %considered, one can assign it to cur_v confidently    
    cur_v = tar_v;    
    t_actual = robot_params.m_sampling_time;
    t = t + t_actual;
    delta_movement = cur_v * t_actual;
    cur_p = cur_p + delta_movement;
    remain_dist = remain_dist - delta_movement;    
    itercount = [itercount, t];
    %fprintf('%d\n',iter);
    dlist = [dlist, remain_dist];
    vlist = [vlist, cur_v];
    iter = iter + 1;
    %now draw the output 
    if iter > 2000 %size(vlist,2) > 10 && sum(vlist(end-10:end)) == 0
        figure(1);
        subplot(2,1,1);
        plot(itercount, dlist);
        hold on;
        yline(obstacle_params.obs_point + 0.8, '-', 'within 0.8m' );
        yline(obstacle_params.obs_point, '-', 'obstacle point' );
        title('remaining distance VS time');
        xlabel('time(s)');
        ylabel('remain distance (m)');
        subplot(2,1,2);
        plot(itercount, vlist);
        title('velocity VS time');
        xlabel('time(s)');
        ylabel('velocity(m/s)');
        
        break
    end 
end