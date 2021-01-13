function tar_v = linearTrapezoid(robot_params,robot_state, tar_v_)
%             % linearTrapezoid
%             if true %cur_v^2/(2*max_dec_v_*0.6)<front_dist

global max_acc_v;
global max_dec_v;
m_sampling_time = robot_params.m_sampling_time;
cur_v = robot_state.cur_v;

    if tar_v_ >= 0
        if tar_v_ >= cur_v
            tar_v = min((cur_v + max_acc_v * m_sampling_time), tar_v_);
            %                         tar_v = min(abs(max_v_*sin((cur_v+0.05)/max_v_/pi)), tar_v);
        else 
            tar_v = max((cur_v - max_dec_v * m_sampling_time), tar_v_);
        end
    else
        if tar_v_ <= cur_v
            tar_v = max((cur_v - max_acc_v * m_sampling_time), tar_v_);
        else
            tar_v = min((cur_v + max_dec_v * m_sampling_time), tar_v_);
        end
    end
end