function [x_opt, u_opt] = runMpcStep(robot, controller, obstacle)
    import casadi.*
    
    % Extract a couple major parameters
    nx = robot.nx;
    nu = robot.nu;
    N = controller.N;

    % Define state and control symbolics
    x = SX.sym('x', nx, N+1);
    u = SX.sym('u', nu, N);
    
    % Define cost and constraints
    constraints = [];
    cost = 0;
    
    % Initial constraint
    constraints = [constraints, x(:, 1) == robot.x];
    
    % Loop through all steps
    for k = 1:N
        % Append dynamic model constraints
        constraints = [
            constraints;
            x(:, k+1) == robot.Ad * x(:, k) + robot.Bd * u(:, k)];
        % Add stage cost
        cost = cost + x(:, k)' * controller.Q * x(:, k) + u(:, k)' * controller.R * u(:, k);
    end
    
    % Loop through all steps
    % for k = 1:N
    %     % Compute value of CBF at current step
    %     hk = (x(1:2, k) - obstacle.pos)' * (x(1:2, k) - obstacle.pos) - obstacle.r^2;
    %     % Compute value of CBF at next step
    %     hkp1 = (x(1:2, k+1) - obstacle.pos)' * (x(1:2, k) - obstacle.pos) - obstacle.r^2;
    %     % Append CBF constraint
    %     constraints = [constraints; hkp1 - hk + controller.gamma * hk];
    % end
    
    % Add terminal cost
    cost = cost + x(:, end)' * controller.P * x(:, end);
    
    % Construct nlp
    nlp = struct;
    nlp.x = [reshape(x, nx*(N+1), 1); reshape(u, nu*N, 1)];
    nlp.f = cost;
    nlp.g = constraints;

    % options.ipopt.print_level = 0;
    options.print_time = false;
    
    % Create nlp solver
    solver = nlpsol('F', 'ipopt', nlp, options);
    
    % Solve NLP
    solution = solver( ...
        'x0', [repmat(robot.x, N+1, 1); ones(nu*N, 1)], ...
        'lbx', [repmat(controller.xb(1), nx*(N+1), 1); repmat(controller.ub(1), nu*N, 1)], ...
        'ubx', [repmat(controller.xb(2), nx*(N+1), 1); repmat(controller.ub(2), nu*N, 1)], ...
        'lbg', [zeros(nx*(N+1), 1)], ...
        'ubg', [zeros(nx*(N+1), 1)]);
    
    x_opt = reshape(solution.x(1:nx*(N+1)), nx, N+1);
    u_opt = reshape(solution.x(nx*(N+1)+1:end), nu, N);
end