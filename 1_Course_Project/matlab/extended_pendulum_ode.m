function dzdt = extended_pendulum_ode(t, z, K, L, C, f, m, M, l, g, A, B)
    x = z(1:4);       % Реальные состояния
    x_hat = z(5:8);   % Оценки наблюдателя
    
    [dxdt, dx_hat, u] = pendulum_ode_obs(t, x, x_hat, K, L, C, f, m, M, l, g, A, B);
    
    dzdt = [dxdt; dx_hat];
end