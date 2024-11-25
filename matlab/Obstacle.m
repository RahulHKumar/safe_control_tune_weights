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
    end
end