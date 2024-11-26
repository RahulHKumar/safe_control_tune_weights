%% Clean Start
clear;

%% Define Parameters
N = 4;
dt = 0.2;
lbx = -5;
ubx = 5;
lbu = -1;
ubu = 1;
Q = 10 * eye(4);
R = eye(2);
P = 100 * eye(4);
x0 = [-5; -5; 0; 0];

%% Define robot, controller, and obstacle
robot = Robot(dt, x0);
controller = MPC_CBF_Controller(Q, R, P, N, 0, [lbx, ubx], [lbu, ubu]);
obstacle = Obstacle([-2; -2.25], 1.5);

%% Run for gamma = 0.3
Ns = [1, 2, 4, 10];
x = zeros(101, length(Ns));
y = zeros(101, length(Ns));
for i = 1:length(Ns)
    N = Ns(i);
    disp("Running N = " + num2str(N));
    x_opt = ones(robot.nx, N+1);
    u_opt = ones(robot.nu, N);
    controller.gamma = 0.2;
    controller.N = N;
    for k = 1:100
        % Solve nlp
        [x_opt, u_opt] = runMpcStep(robot, controller, obstacle, x_opt, u_opt);
        % Update robot state
        robot = robot.update(u_opt(:, 1));
    end
    x(:, i) = robot.xlog(1, :)';
    y(:, i) = robot.xlog(2, :)';
    robot = robot.reset();
end

%% Create legend entries
labels = cell(length(Ns) + 3, 1);
for i = 1:length(labels)-3
    labels{i} = "$N = " + num2str(Ns(i) + "$");
end
labels{end-2} = "Start";
labels{end-1} = "Goal";
labels{end} = "Obstacle";

%% Plot
fig = figure;
grid on; hold on;
plot(x, y, LineWidth=1.5, LineStyle="--");
scatter(x0(1), x0(2), 50, Marker="diamond", MarkerEdgeColor="k", MarkerFaceColor="blue");
scatter(0, 0, 100, Marker="pentagram", MarkerEdgeColor="k", MarkerFaceColor="green");
obstacle.draw(fig);
legend(labels, Interpreter="latex", location="best");
axis([-9, 1, -6, 1], "equal");
xlabel("$x$ (m)", Interpreter="latex");
ylabel("$y$ (m)", Interpreter="latex");
title("MPC-CBF Trajectories for Varying $N$, $\gamma = " + num2str(0.2) + "$", Interpreter="latex")
set(gca, "TickLabelInterpreter", "latex");
