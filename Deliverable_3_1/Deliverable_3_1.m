function Deliverable_3_1
    clear all; close all; clc;

    x0 = zeros(12,1); % Initial state
    x0(12) = 2; % z begins at 2 meters (to settle at 0m)
    x0(10) = 2; % x begins at 2 meters (to settle at 0m)
    x0(11) = 2; % y begins at 2 meters (to settle at 0m)
    x0(6) = -pi/4; % yaw begins at -45deg (to settle at 0deg)
    x = [x0(2);x0(5);x0(7);x0(10)];
    y = [x0(1);x0(4);x0(8);x0(11)];
    z = [x0(9);x0(12)];
    yaw = [x0(3);x0(6)];

    Ts = 1/5;
    quad = Quad(Ts);
    [xs, us] = quad.trim();
    sys = quad.linearize(xs, us);
    [sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);

    % Design MPC controller
    mpc_x = MPC_Control_x(sys_x, Ts);
    mpc_y = MPC_Control_y(sys_y, Ts);
    mpc_z = MPC_Control_z(sys_z, Ts);
    mpc_yaw = MPC_Control_yaw(sys_yaw, Ts);

    %% Simulate
    clear sol
    sol.x(:,1) = x;
    sol.y(:,1) = y;
    sol.z(:,1) = z;
    sol.yaw(:,1) = yaw;
    
    i = 1;
    try
        % Simulate until convergence of all subsystems
        while norm(sol.x(:,end)) > 1e-2 || norm(sol.y(:,end)) > 1e-2 || ...
              norm(sol.z(:,end)) > 1e-2 || norm(sol.yaw(:,end)) > 1e-2
            % Solve MPC problem for current state            
            sol.ux(:,i) = mpc_x.get_u(sol.x(:,i));
            sol.uy(:,i) = mpc_y.get_u(sol.y(:,i));
            sol.uz(:,i) = mpc_z.get_u(sol.z(:,i));
            sol.uyaw(:,i) = mpc_yaw.get_u(sol.yaw(:,i));

            % Apply the optimal input to the system
            sol.x(:,i+1) = mpc_x.A*sol.x(:,i) + mpc_x.B*sol.ux(:,i);
            sol.y(:,i+1) = mpc_y.A*sol.y(:,i) + mpc_y.B*sol.uy(:,i);
            sol.z(:,i+1) = mpc_z.A*sol.z(:,i) + mpc_z.B*sol.uz(:,i);
            sol.yaw(:,i+1) = mpc_yaw.A*sol.yaw(:,i) + mpc_yaw.B*sol.uyaw(:,i);

            i = i + 1;
        end
    catch
        error('---> Initial state is outside the feasible set <---\n');
    end

    %% Plotting the results
    T = @(v) [0:Ts:(length(v)-1)*Ts];
    % x
    figure
    sgtitle("X subsystem")
    hold on; grid on;

    % constraints
    f = [0.035;-0.035];
    m = [-0.3;0.3];
    o = ones(1,size(sol.x,2));

    subplot(3,2,1)
    hold on; grid on;
    plot(T(sol.x(1,:)),sol.x(1,:),'-k','markersize',20,'linewidth',2);
    ylabel('Pitch velocity');
    xlabel('Time [s]');

    subplot(3,2,2)
    hold on; grid on;
    plot(T(sol.x(2,:)),sol.x(2,:),'-k','markersize',20,'linewidth',2);
    plot(T(sol.x(2,:)),f(1)*o,'r','linewidth',2);
    plot(T(sol.x(2,:)),f(2)*o,'r','linewidth',2);
    ylabel('Pitch');
    xlabel('Time [s]');
    
    subplot(3,2,3)
    hold on; grid on;
    plot(T(sol.x(3,:)),sol.x(3,:),'-k','markersize',20,'linewidth',2);
    ylabel('X velocity');
    xlabel('Time [s]');
    
    subplot(3,2,4)
    hold on; grid on;
    plot(T(sol.x(4,:)),sol.x(4,:),'-k','markersize',20,'linewidth',2);
    ylabel('X position');
    xlabel('Time [s]');
    
    subplot(3,2,[5,6])
    o = ones(1,size(sol.ux,2));
    hold on; grid on;
    plot(T(sol.ux),sol.ux,'k','markersize',20,'linewidth',2);
    plot(T(sol.ux),m(1)*o,'r','linewidth',2);
    plot(T(sol.ux),m(2)*o,'r','linewidth',2);
    ylabel('Input on x subsystem');
    xlabel('Time [s]');
    hold off;
    
    % y
    figure
    hold on; grid on;
    sgtitle("Y subsystem");

    % constraints
    f = [0.035;-0.035];
    m = [-0.3;0.3];               
    o = ones(1,size(sol.x,2));

    subplot(3,2,1)
    hold on; grid on;
    plot(T(sol.y(1,:)),sol.y(1,:),'-k','markersize',20,'linewidth',2);
    ylabel('Roll velocity');
    xlabel('Time [s]');

    subplot(3,2,2)
    hold on; grid on;
    plot(T(sol.y(2,:)),sol.y(2,:),'-k','markersize',20,'linewidth',2);
    plot(T(sol.x),f(1)*o,'r','linewidth',2);
    plot(T(sol.x),f(2)*o,'r','linewidth',2);
    ylabel('Roll');
    xlabel('Time [s]');
    
    subplot(3,2,3)
    hold on; grid on;
    plot(T(sol.y(3,:)),sol.y(3,:),'-k','markersize',20,'linewidth',2);
    ylabel('Y velocity');
    xlabel('Time [s]');
    
    subplot(3,2,4)
    hold on; grid on;
    plot(T(sol.y(4,:)),sol.y(4,:),'-k','markersize',20,'linewidth',2);
    ylabel('Y position');
    xlabel('Time [s]');
    
    subplot(3,2,[5,6])
    o = ones(1,size(sol.uy,2));
    hold on; grid on;
    plot(T(sol.uy(1,:)),sol.uy(1,:),'k','markersize',20,'linewidth',2);
    plot(T(sol.uy(1,:)),m(1)*o,'r','linewidth',2);
    plot(T(sol.uy(1,:)),m(2)*o,'r','linewidth',2);
    ylabel('Input on y subsystem');
    xlabel('Time [s]');
    hold off;
    
    % z
    figure
    sgtitle("Z Subsystem");
    hold on; grid on;

    % constraints
    m = [-0.2;0.3];               
    o = ones(1,size(sol.z,2));

    subplot(2,2,1)
    hold on; grid on;
    plot(T(sol.z(1,:)),sol.z(1,:),'-k','markersize',20,'linewidth',2);
    ylabel('Z velocity');
    xlabel('Time [s]');

    subplot(2,2,2)
    hold on; grid on;
    plot(T(sol.z(2,:)),sol.z(2,:),'-k','markersize',20,'linewidth',2);
    ylabel('Z position');
    xlabel('Time [s]');
    
    subplot(2,2,[3,4])
    o = ones(1,size(sol.uz,2));
    hold on; grid on;
    plot(T(sol.uz),sol.uz,'k','markersize',20,'linewidth',2);
    plot(T(sol.uz),m(1)*o,'r','linewidth',2);
    plot(T(sol.uz),m(2)*o,'r','linewidth',2);
    ylabel('Input on z subsystem');
    xlabel('Time [s]');
    hold off;
    
    % yaw
    figure
    sgtitle("Yaw Subsystem");
    hold on; grid on;
    
    % constraints
    m = [-0.2;0.2];               
    o = ones(1,size(sol.yaw,2));

    subplot(2,2,1)
    hold on; grid on;
    plot(T(sol.yaw(1,:)),sol.yaw(1,:),'-k','markersize',20,'linewidth',2);
    ylabel('Yaw velocity [rad/s]');
    xlabel('Time [s]');

    subplot(2,2,2)
    hold on; grid on;
    plot(T(sol.yaw(2,:)),sol.yaw(2,:),'-k','markersize',20,'linewidth',2);
    ylabel('Yaw [rad]');
    xlabel('Time [s]');
    
    subplot(2,2,[3,4])
    o = ones(1,size(sol.uyaw,2));
    hold on; grid on;
    plot(T(sol.uyaw),sol.uyaw,'k','markersize',20,'linewidth',2);
    plot(T(sol.uyaw),m(1)*o,'r','linewidth',2);
    plot(T(sol.uyaw),m(2)*o,'r','linewidth',2);
    ylabel('Input on yaw subsystem');
    xlabel('Time [s]');
    hold off;
end
