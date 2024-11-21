clear;
import casadi.*

% Parameters
N = 10;
dt = 0.2;
lbx = -5;
ubx = 5;
lbu = -1;
ubu = 1;
[Ad, Bd] = doubleIntegrator(dt);
Q = 10 * eye(4);
R = eye(2);
P = 100 * eye(4);
obs.pos = [-2; -2.25];
obs.r = 1.5;
gamma = 0.3;
x0 = [-5; 5; 0; 0];

% Define state and control symbolics
x = SX.sym('x', 4, N+1);
u = SX.sym('u', 2, N);

% Define cost and constraints
constraints = [];
cost = 0;

% Initial constraint
constraints = [constraints, x(:, 1) == x0];

% Loop through all steps
for k = 1:N
    % Append dynamic model constraints
    constraints = [constraints; x(:, k+1) - Ad * x(:, k) + Bd * u(:, k)];
    % Add stage cost
    cost = cost + x(:, k)' * Q * x(:, k) + u(:, k)' * R * u(:, k);
end

% Loop through all steps
for k = 1:N
    % Compute value of CBF at current step
    hk = (x(1:2, k) - obs.pos)' * (x(1:2, k) - obs.pos) - obs.r^2;
    % Compute value of CBF at next step
    hkp1 = (x(1:2, k+1) - obs.pos)' * (x(1:2, k) - obs.pos) - obs.r^2;
    % Append CBF constraint
    constraints = [constraints; hkp1 - hk + gamma * hk];
end

% Add terminal cost
cost = cost + x(:, end)' * P * x(:, end);

% Construct nlp
nlp = struct;
nlp.x = [reshape(x, 4*(N+1), 1); reshape(u, 2*N, 1)];
nlp.f = cost;
nlp.g = constraints;

% Create nlp solver
solver = nlpsol('F', 'ipopt', nlp);

% Solve NLP
X_opt = solver( ...
    'x0', zeros(6*N+4,1), ...
    'lbx', [repmat(lbx, 4*(N+1), 1); repmat(lbu, 2*N, 1)], ...
    'ubx', [repmat(ubx, 4*(N+1), 1); repmat(ubu, 2*N, 1)], ...
    'lbg', [zeros(4*(N+1), 1); zeros(N, 1)], ...
    'ubg', [zeros(4*(N+1), 1); inf(N, 1)]);



function [A, B] = doubleIntegrator(dt)
    A = [zeros(2), eye(2); zeros(2, 4)];
    B = [zeros(2); eye(2)];
    sysd = c2d(ss(A,B,eye(4),zeros(4,2)), dt, "ZOH");
    A = sysd.A;
    B = sysd.B;
end

