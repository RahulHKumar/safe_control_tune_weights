clear;

% Parameters
N = 10;
dt = 0.2;
lbx = -5;
ubx = 5;
lbu = -1;
ubu = 1;
Q = 10 * eye(4);
R = eye(2);
P = 100 * eye(4);
gamma = 0.3;
x0 = [-5; -5; 0; 0];

% Define robot, controller, and obstacle
robot = Robot(dt, x0);
controller = MPC_CBF_Controller(Q, R, P, N, gamma, [lbx, ubx], [lbu, ubu]);
obstacle = Obstacle([-2; -2.25], 1.5);

% Run MPC CBF a bunch?
for k = 1:100
    % Solve nlp
    [x_opt, u_opt] = runMpcStep(robot, controller, obstacle);
    % Print
    disp("k: " + num2str(k) + ", x = [" + num2str(full(robot.x')) + "], u = [" + num2str(full(u_opt(:,1)')) + "]");
    % Update robot state
    robot = robot.update(u_opt(:, 1));
    % Log state and control

end

