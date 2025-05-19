function [dxdt u] = pendulum_ode(t, x, K, f, m, M, l, g)
    % Параметры системы
    sin_phi = sin(x(3));
    cos_phi = cos(x(3));
    u = K*x;
    % Уравнения системы
    dxdt = zeros(4,1);
    dxdt(1) = x(2);  % \dot{x}_1 = x_2
    
    dxdt(2) = -(3*cos_phi*sin_phi*g*l*m + 6*cos_phi*f - 2*x(4)^2*sin_phi*l^2*m + 4*l*u) / ...
              (l*(3*cos_phi^2*m - 4*m - 4*M));  % \dot{x}_2
    
    dxdt(3) = x(4);  % \dot{x}_3 = x_4
    
    dxdt(4) = 3*(cos_phi*sin_phi*x(4)^2*l^2*m^2 - 2*l*m*u*cos_phi - 2*g*l*m^2*sin_phi - 2*g*l*m*M*sin_phi - 4*f*m - 4*f*M) / ...
              (l^2*m*(3*m*cos_phi^2 - 4*m - 4*M));  % \dot{x}_4
end