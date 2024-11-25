function [x_opt, u_opt] = runMpcQpStep(robot, controller, obstacle)
    % Unpack a few parameters
    Q = controller.Q;
    R = controller.R;
    P = controller.P;
    N = controller.N;
    nx = robot.nx;
    nu = robot.nu;

    % Form matrices to define QP
    Q1 = [];
    Q3 = [];
    for k = 1:N
        Q1 = blkdiag(Q1, Q);
        Q3 = blkdiag(Q3, R);
    end

    Qbar = blkdiag(Q1, P, Q3);

    A1 = [eye(nx), zeros(nx, (nx + nu) * N)];
    A21 = zeros(nx*N, nx*(N+1));
    A22 = [];
    for k = 1:N
        rs = nx*(k-1) + 1;
        re = rs + nx - 1;
        cs = nx*(k-1) + 1;
        ce = cs + nx + nx - 1;
        A21(rs:re, cs:ce) = [robot.Ad, -eye(nx)];
        A22 = blkdiag(A22, robot.Bd);
    end
    A = [A1; A21, A22];
    b = [robot.x; zeros(nx*N, 1)];

    lb = [repmat(controller.xb(1), nx*(N+1), 1); repmat(controller.ub(1), nu*N, 1)];
    ub = [repmat(controller.xb(2), nx*(N+1), 1); repmat(controller.ub(2), nu*N, 1)];

    % Solve QP
    options = optimoptions('quadprog', 'Display', 'none');
    W = quadprog(2 * Qbar, zeros(N*(nx+nu)+nx, 1), ...
        [], [], ...
        A, b, ...
        lb, ub, ...
        [], options);

    % Unpack solution
    x_opt = reshape(W(1:nx*(N+1)), nx, N+1);
    u_opt = reshape(W(nx*(N+1)+1:end), nu, N);
end