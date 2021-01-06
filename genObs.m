function obs_point = genObs(target_point, generated)

    if generated
        a = 0.6;
        b = target_point;
        obs_point = (b-a).*rand(1) + a;
    else 
        obs_point = target_point/2;
    end

end

