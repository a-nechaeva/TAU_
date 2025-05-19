function x_hat = estimate_x(y, z_hat, Q, C, D, u)
    T = [C; Q]; % Преобразующая матрица
    %rhs = [y - D*u; z_hat];
    rhs = [y; z_hat];
    x_hat = pinv(T) * rhs; % Решение системы линейных уравнений
end