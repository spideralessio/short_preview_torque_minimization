function [q, qd] = update_robot(qdd, q0, qd0, dt)
    qd = qd0 + qdd*dt;
    q = q0 + qd0*dt + qdd*(dt^2);
end