function [dxdt, dz_hat, x_hat] = pendulum_observer_ode(t, x, z_hat, K, Q, Gamma, Y, QB_YD, C, D, f, m, M, l, g)
    % Параметры системы
    sin_phi = sin(x(3));
    cos_phi = cos(x(3));
    
    % Выход системы (измерения)
    y = C * x; % D=0, если нет прямой связи
    
    % Управление (используем оценку состояния)
    x_hat = estimate_x(y, z_hat, Q, C, D, 0); % u=0 пока не вычислено
    u = K * x_hat;
    
    % Основная система (нелинейная)
    dxdt = zeros(4,1);
    dxdt(1) = x(2);
    dxdt(2) = -(3*cos_phi*sin_phi*g*l*m + 6*cos_phi*f - 2*x(4)^2*sin_phi*l^2*m + 4*l*u) / ...
              (l*(3*cos_phi^2*m - 4*m - 4*M));
    dxdt(3) = x(4);
    dxdt(4) = 3*(cos_phi*sin_phi*x(4)^2*l^2*m^2 - 2*l*m*u*cos_phi - 2*g*l*m^2*sin_phi - 2*g*l*m*M*sin_phi - 4*f*m - 4*f*M) / ...
              (l^2*m*(3*m*cos_phi^2 - 4*m - 4*M));

    % Наблюдатель пониженной размерности
    dz_hat = Gamma * z_hat - Y * y + QB_YD * u;
    
    % Обновленная оценка полного состояния
    x_hat = estimate_x(y, z_hat, Q, C, D, u);
end