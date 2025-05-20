function [dxdt, dx_hat, u] = pendulum_ode_kalman(t, x, x_hat, K, L, C, m, M, l, g_nominal, A, B)
    % Параметры системы
    sin_phi = sin(x(3));
    cos_phi = cos(x(3));
    
    % Генерация шумов
    process_noise = 0.001*randn(4,1);     % f(t) ~ N(0,1)
    measurement_noise = 0.01*randn(size(C,1),1); % ξ(t) ~ N(0,0.5)
    
    % Управляющий сигнал (по оценке состояния)
    u = K*x_hat;
    
    % Нелинейная динамика системы с шумом процесса
    dxdt = zeros(4,1);
    dxdt(1) = x(2) + process_noise(1);
    
    dxdt(2) = -(3*cos_phi*sin_phi*g_nominal*l*m + 6*cos_phi*process_noise(2) - ...
              2*x(4)^2*sin_phi*l^2*m + 4*l*u) / (l*(3*cos_phi^2*m - 4*m - 4*M)) + process_noise(2);
    
    dxdt(3) = x(4) + process_noise(3);
    
    dxdt(4) = 3*(cos_phi*sin_phi*x(4)^2*l^2*m^2 - 2*l*m*u*cos_phi - ...
              2*g_nominal*l*m^2*sin_phi - 2*g_nominal*l*m*M*sin_phi - ...
              4*process_noise(4)*m - 4*process_noise(4)*M) / ...
              (l^2*m*(3*m*cos_phi^2 - 4*m - 4*M)) + process_noise(4);
    
    % Наблюдатель Люенбергера (уже содержит L)
    y = C*x + measurement_noise;       % Зашумленные измерения
    dx_hat = A*x_hat + B*u + L*(C*x_hat-y); % Обычный наблюдатель
end