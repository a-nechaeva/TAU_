function [dxdt, u, w_out] = pendulum_ode_with_tracking(t, x, K, K_g, Gamma_g, Y_g, m, M, l, g,f)
    % Состояния системы
    x1 = x(1); % положение тележки
    x2 = x(2); % скорость тележки
    phi = x(3); % угол маятника
    dphi = x(4); % угловая скорость
    w = x(5:14); % состояния автогенератора возмущений
    
    
    % Параметры системы
    sin_phi = sin(phi);
    cos_phi = cos(phi);
    
    % Генерация возмущения
    g_ = Y_g * w;
    
    % Управляющий сигнал с компенсацией
    u = K * x(1:4) + K_g * w;
    
    % Динамика автогенератора
    dw = Gamma_g * w;
    
    % Уравнения основной системы
    dxdt = zeros(14,1);
    dxdt(1) = x2;
    dxdt(2) = -(3*cos_phi*sin_phi*g*l*m + 6*cos_phi*f - 2*dphi^2*sin_phi*l^2*m + 4*l*u) / ...
              (l*(3*cos_phi^2*m - 4*m - 4*M));
    dxdt(3) = dphi;
    dxdt(4) = 3*(cos_phi*sin_phi*dphi^2*l^2*m^2 - 2*l*m*u*cos_phi - 2*g*l*m^2*sin_phi - 2*g*l*m*M*sin_phi - 4*f*m - 4*f*M) / ...
              (l^2*m*(3*m*cos_phi^2 - 4*m - 4*M));
    
    % Динамика автогенератора
    dxdt(5:14) = dw;
    
    % Дополнительные выходы
    w_out = w;
end