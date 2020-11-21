function Deliverable_6_1
    clear all; close all; clc;  
    Ts = 1/5;
    quad = Quad(Ts);
    
    [ctrl, ~] = ctrl_NMPC(quad);
    sim = quad.sim(ctrl);
    quad.plot(sim);
end

function [ctrl, traj] = ctrl_NMPC(quad)
    import casadi.*
    opti = casadi.Opti();
    
    N = 25; % MPC horizon
        
    %−−−−decision variables−−−−−−−−−
    X = opti.variable(12,N+1); % state trajectory variables
    U = opti.variable(4, N); % control trajectory (throttle, brake)
    X0 = opti.parameter(12,1); % initial state
    REF = opti.parameter(4,1); % reference position [x,y,z,yaw]
    
    %%%%%%%%%%%
    % YOUR CODE
    traj = 0; % what is this variable for??

    % Cost parameters
    R=4;
    Qxy = 4; Qz = 6; Qyaw = 3; Qpr = 2;
    Qdxy = 2; Qdz = 3; Qdyaw = 2; Qdpr = 2;
    
    %line 1: x, y -- line 2: z, yaw -- line 3:pitch, yaw and derivatives
    % line 4: dx, dy, dz, dyaw -- line 5: inputs
    opti.minimize(...
        Qxy*(sum((X(10,:)-REF(1)).^2) + sum((X(11,:)-REF(2)).^2)) +...
        Qz*(sum((X(12,:)-REF(3)).^2)) + Qyaw*sum((X(6, :)-REF(4)).^2) +...
        Qdpr*(X(1,:)*X(1,:)'+X(2,:)*X(2,:)') + Qpr*(X(4,:)*X(4,:)'+X(5,:)*X(5,:)')+...
        Qdxy*(sum(X(7,:).^2) + sum(X(8,:).^2)) + Qdz*sum(X(9,:).^2) + Qdyaw*sum(X(3,:).^2)+...
        R*(U(1,:)*U(1,:)'+U(2,:)*U(2,:)'+U(3,:)*U(3,:)'+U(4,:)*U(4,:)'));
        
    for k=1:N % system dynamics
        opti.subject_to(X(:,k+1) == RK4(X(:,k),U(:,k),quad.Ts,@quad.f));  
    end
    
    opti.subject_to(X(:,1) == X0);
    opti.subject_to(0 <= U <= 1.5);
    % YOUR CODE
    %%%%%%%%%%%
    
    ctrl = @(x,ref) evalctrl(x, ref, opti, X0, REF, X, U);
end

function u = evalctrl(x, ref, opti, X0, REF, X, U)
    opti.set_value(X0, x);
    opti.set_value(REF, ref);
    ops = struct('ipopt', struct('print_level',0, 'tol', 1e-3), 'print_time', false);
    opti.solver('ipopt', ops);
    sol = opti.solve();
    assert(sol.stats.success == 1, 'Error computing optimal input');
    u = opti.value(U(:,1));
    opti.set_initial(sol.value_variables());
    opti.set_initial(opti.lam_g, sol.value(opti.lam_g));
end