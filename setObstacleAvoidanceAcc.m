function [max_v_, max_acc_v_, max_dec_v_ ] = setObstacleAvoidanceAcc(robot_params,obstacle_params, robot_state)
    front_dist = robot_state.target_point - robot_state.cur_p;
    cur_p = robot_state.cur_p;
    obs_point = obstacle_params.obs_point ;
    obstacle_avoidance_acc = obstacle_params.obstacle_avoidance_acc;
    max_v_ = robot_params.max_v_;
    max_acc_v_ = robot_params.max_acc_v_;
    max_dec_v_ = robot_params.max_dec_v_;
    
    % no detection
    if obs_point >= robot_state.target_point 
        % do nothing 
        
    % far detection
    elseif (obs_point - cur_p) < 3.2 && (obs_point - cur_p) > 2.3
        if abs(front_dist) > 3.5
            max_v_ = 0.6;
            max_acc_v_ = max(max_acc_v_, obstacle_avoidance_acc * 0.5);
            max_dec_v_ = max(max_dec_v_, obstacle_avoidance_acc * 0.5);
        end
        
    % mid detection
    elseif (obs_point - cur_p) < 2.3 && (obs_point - cur_p) > 0.8
        if abs(front_dist) > 0.8
            max_v_ = 0.2;
            max_acc_v_ = max(max_acc_v_, obstacle_avoidance_acc * 0.72);
            max_dec_v_ = max(max_dec_v_, obstacle_avoidance_acc * 0.72);
        else
            max_v_ = min(max_v_, 0.6);
        end
        
    % short detection
    elseif (obs_point - cur_p) < 0.8
        if abs(front_dist) > 0.8
            max_v_ = 0;
            max_acc_v_ = max(max_acc_v_, obstacle_avoidance_acc * 1.08);
            max_dec_v_ = max(max_dec_v_, obstacle_avoidance_acc * 1.08);
        else
            max_v_ = min(max_v_, 0.4);
        end
    end
    

    
end