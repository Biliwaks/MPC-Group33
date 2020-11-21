function Deliverable_3_2
    clear all; close all; clc;
    
    %% Set initial state and reference
    x0 = zeros(12,1);
    x = [x0(2);x0(5);x0(7);x0(10)];
    y = [x0(1);x0(4);x0(8);x0(11)];
    z = [x0(9);x0(12)];
    yaw = [x0(3);x0(6)];
    
    ref = zeros(4,1); % [X, Y, Z, YAW]'
    ref(1:3) = [-2;-2;-2];
    ref(4) = pi/4;

    %% compute MPCs
    Te=1/5;
    quad = Quad(Te);
    [xs,us] = quad.trim();
    sys = quad.linearize(xs, us);

    [sysx, sysy, sysz, sysyaw] = quad.decompose(sys, xs, us);

    mpc_x = MPC_Control_x(sysx,Te);
    mpc_y = MPC_Control_y(sysy,Te);
    mpc_z = MPC_Control_z(sysz,Te);
    mpc_yaw = MPC_Control_yaw(sysyaw,Te);

    %% Simulate
    clear sol
    sol.x(:,1) = x;
    sol.y(:,1) = y;
    sol.z(:,1) = z;
    sol.yaw(:,1) = yaw;
    R_x = [zeros(size(x,1)-1,1);ref(1)];
    R_y = [zeros(size(y,1)-1,1);ref(2)];
    R_z = [zeros(size(z,1)-1,1);ref(3)];
    R_yaw = [zeros(size(yaw,1)-1,1);ref(4)];
    
    i = 1;
    try
        % Simulate until convergence of all subsystems
        while norm(sol.x(:,end)-R_x) > 1e-2 || norm(sol.y(:,end)-R_y) > 1e-2 ||...
              norm(sol.z(:,end)-R_z) > 1e-2 || norm(sol.yaw(:,end)-R_yaw) > 1e-2
          
            % Solve MPC problem for current state
            sol.ux(:,i) = mpc_x.get_u(sol.x(:,i), ref(1));
            sol.uy(:,i) = mpc_y.get_u(sol.y(:,i), ref(2));
            sol.uz(:,i) = mpc_z.get_u(sol.z(:,i), ref(3));
            sol.uyaw(:,i) = mpc_yaw.get_u(sol.yaw(:,i), ref(4));

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
    T = @(v) [0:Te:(length(v)-1)*Te];
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
    plot(T(sol.y),f(1)*o,'r','linewidth',2);
    plot(T(sol.y),f(2)*o,'r','linewidth',2);
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