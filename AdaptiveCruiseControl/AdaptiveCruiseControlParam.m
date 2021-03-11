classdef AdaptiveCruiseControlParam
    properties
        g = 9.81;
        m = 1650;
        f0 = 0.1;
        f1 = 5;
        f2 = 0.25;
        vd = 30;
        v0 = 16;
        gamma = 1;
        alph = 0.5;
        c_a = 0.25;
        c_d = 0.25;
        p = 1;
        % default hyperparameters
        p_sb = 10^8;
        omega0 = 1.0
    end
end