function dx = sys_wrapper_kalman(t, x, K, L, C, m, M, l, g_nominal, A, B)
    % Разделение состояний:
    % x(1:4) - реальные состояния системы [x1; x2; phi; dphi]
    % x(5:8) - оценки наблюдателя
    
    x_sys = x(1:4);
    x_hat = x(5:8);
    
    % Вызов основной функции с наблюдателем
    [dx_sys, dx_hat, ~] = pendulum_ode_kalman(t, x_sys, x_hat, K, L, C, m, M, l, g_nominal, A, B);
    
    % Объединение производных
    dx = [dx_sys; dx_hat];
end