function [x_opt, u_opt] = runMpcStep(robot, controller, obstacle)
    import casadi.*
    % Initialize the opti stack
    opti = Opti();
    
    % Extract a couple major parameters
    nx = robot.nx;
    nu = robot.nu;
    N = controller.N;

    % Define state and control symbolics
    x = opti.variable(nx, N+1);
    u = opti.variable(nu, N);
    
    % Initial constraint and cost
    opti.subject_to(x(:, 1) == robot.x);
    obj = MX(0);
    
    % Loop through all steps
    for k = 1:N
        % Append dynamic model constraints
            opti.subject_to(x(:, k+1) == robot.Ad * x(:, k) + robot.Bd * u(:, k));
        % Add stage cost
        obj = obj + x(:, k)' * controller.Q * x(:, k) + u(:, k)' * controller.R * u(:, k);
    end
    % Loop through all steps
    for k = 1:N
        % Compute value of CBF at current step
        hk = (x(1:2, k) - obstacle.pos)' * (x(1:2, k) - obstacle.pos) - obstacle.r^2;
        % Compute value of CBF at next step
        hkp1 = (x(1:2, k+1) - obstacle.pos)' * (x(1:2, k) - obstacle.pos) - obstacle.r^2;
        % Append CBF constraint
        opti.subject_to(hkp1 - hk + controller.gamma * hk >= 0);
    end
    
    % Add terminal cost
    obj = obj + x(:, end)' * controller.P * x(:, end);
    
    opti.minimize(obj);

    % Add solver options
    options.ipopt.print_level = 0;
    options.print_time = false;

    % Solve the NLP
    opti.solver('ipopt', options);
    sol = opti.solve();
    
    % Retrieve the optimal values
    x_opt = sol.value(x);
    u_opt = sol.value(u);
end