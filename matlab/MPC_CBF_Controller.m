classdef MPC_CBF_Controller
    properties
        Q       % Stage cost state weight matrix
        R       % Stage cost control weight matrix
        P       % Final cost state weight matrix
        N       % Horizon (planning & control)
        gamma   % CBF aggression parameter
        xb      % [lbx, ubx]
        ub      % [lbu, ubu]
    end
    methods
        function [controller] = MPC_CBF_Controller(Q, R, P, N, gamma, xb, ub)
            controller.Q = Q;
            controller.R = R;
            controller.P = P;
            controller.N = N;
            controller.gamma = gamma;
            controller.xb = xb;
            controller.ub = ub;
        end
    end
end

