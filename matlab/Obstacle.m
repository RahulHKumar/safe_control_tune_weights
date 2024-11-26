classdef Obstacle
    properties
        pos % Position in R2
        r   % Radius in m
    end
    methods
        function [obs] = Obstacle(pos, r)
            obs.pos = pos;
            obs.r = r;
        end

        function [] = draw(self, fig)
            figure(fig);
            theta = 0:pi/100:2*pi;
            x = self.r * cos(theta) + self.pos(1);
            y = self.r * sin(theta) + self.pos(2);
            fill(x, y, [0.9 0.9 0.9]);
            plot(x, y, "k");
        end
    end
end