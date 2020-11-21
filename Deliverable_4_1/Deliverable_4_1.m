function Deliverable_4_1
    clear; close all; clc;
    Ts=1/5;
    
    quad = Quad(Ts);
    [xs,us] = quad.trim();
    sys = quad.linearize(xs, us);
    [sysx, sysy, sysz, sysyaw] = quad.decompose(sys, xs, us);
    
    mpc_x = MPC_Control_x(sysx,Ts);
    mpc_y = MPC_Control_y(sysy,Ts);
    mpc_z = MPC_Control_z(sysz,Ts);
    mpc_yaw = MPC_Control_yaw(sysyaw,Ts);
    
    sim = quad.sim(mpc_x, mpc_y, mpc_z, mpc_yaw);
    quad.plot(sim);
end