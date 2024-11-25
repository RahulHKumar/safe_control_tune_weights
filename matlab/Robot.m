classdef Robot
    properties
        Ac      % State transition matrix in continuous time
        Bc      % Input matrix in continuous time
        Ad      % State transition matrix in discrete time
        Bd      % Input matrix in discrete time
        dt      % Time step
        x       % Robot state in R4
        xlog    % Record of all robot states
        ulog    % Record of all controls applied to robot
        nx      % dimension of state vector
        nu      % dimension of control vector
    end
    methods
        function robot = Robot(dt, x0)
            robot.Ac = [zeros(2), eye(2); zeros(2, 4)];
            robot.Bc = [zeros(2); eye(2)];
            robot.dt = dt;
            robot.x = x0;
            robot.nx = 4;
            robot.nu = 2;
            robot.xlog = x0;
            sysd = c2d(ss(robot.Ac,robot.Bc,eye(4),zeros(4,2)), robot.dt, "ZOH");
            robot.Ad = sysd.A;
            robot.Bd = sysd.B;
        end

        function [self] = update(self, u)
            self.x = self.Ad * self.x + self.Bd * u;
            self.xlog(:, end + 1) = full(self.x);
            self.ulog(:, end + 1) = full(u);
        end
    end
end