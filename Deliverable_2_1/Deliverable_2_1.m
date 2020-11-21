function Deliverable_2_1
    clear all; close all; clc;
    quad = Quad();

    %% Steady state
    [xs,us] = quad.trim(); %This can be computed using F=m*g with F_mot=F/4
    disp("Steady state input: us = ["+us(1)+", "+us(2)+", "+us(3)+", "+us(4)+"]");
    sys = quad.linearize(xs, us);
    sys_transformed = sys*inv(quad.T); % just for the sake of trying it
    
    % If we check sys.A, we can see the dependencies of each state variable
    %  depending of the state vector. We can notably see the cross dependencies
    %  on the velocity (angle & position). However, we can especially notice
    %  that the yaw, and the p vector have solely zero on the column. This is
    %  quite logical, if the input vector is the one at which the system is
    %  stable (us mentionned previously). This allows us to determine which are
    %  the possible xs at which the system is perfectly stable.

    %% Up in the air, stable/separable case (like steady state)
    % Let's take for example a case on which we change the position and
    %  we give it some yaw angle:
    xs = [zeros(3,1);0;0;pi/4;zeros(3,1);10;10;10];
    sys2 = quad.linearize(xs, us);
    sys_transformed = sys2*inv(quad.T);

    disp("The system has a non-zero yaw angle and position (x,y,z):");
    if (all(all(sys2.A-sys.A == 0)))
        disp("->Match! => Steady-state (expected)") % the linearized system will match
    else
        disp("->Mismatch! => Not a steady-state (unexepected)")
    end

    %% Non separable case
    xs = [zeros(3,1);pi/6;0;pi/4;zeros(3,1);10;10;10];
    sys3 = quad.linearize(xs, us);
    
    disp("The system has a non-zero roll, yaw and position (x,y,z):");
    if (all(all(sys3.A-sys.A == 0)))
        disp("->Match! => Steady-state (unexepected)")
    else
        disp("->Mismatch! => Not a steady-state (expected)")
        %Of course here in this case, the quad will have non zero velocities
        % on y and z in the case of a roll.
        % Thus the system cannot be separated the way we want.
    end
end