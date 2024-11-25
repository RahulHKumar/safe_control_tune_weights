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
    [x_opt, u_opt] = runMpcQpStep(robot, controller, obstacle);
    % Update robot state
    robot = robot.update(u_opt(:, 1));
end

x = robot.xlog(1, :)';
y = robot.xlog(2, :)';

fig = figure;
grid on; hold on;
obstacle.draw(fig);
scatter(x, y, Marker="o", MarkerEdgeColor="k", MarkerFaceColor="red");
scatter(x0(1), x0(2), 100, Marker="diamond", MarkerEdgeColor="k", MarkerFaceColor="blue");
scatter(0, 0, 200, Marker="pentagram", MarkerEdgeColor="k", MarkerFaceColor="green");
legend("Obstacle", "Path", "Initial Point", "Goal Point", Interpreter="latex", location="best");
axis([-6, 1, -6, 1], "equal");
