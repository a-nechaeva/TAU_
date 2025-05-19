function dz = observer_wrapper(t, z, K, Q, Gamma, Y, QB_YD, C, D, f, m, M, l, g)
    x = z(1:4);
    z_hat = z(5:6);
    
    [dxdt, dz_hat, ~] = pendulum_observer_ode(t, x, z_hat, K, Q, Gamma, Y, QB_YD, C, D, f, m, M, l, g);
    
    dz = [dxdt; dz_hat];
end