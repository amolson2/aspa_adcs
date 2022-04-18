function reg = regulate()
    reg.regulate = @reg;
    reg.plot_momenta = @plot_momenta;
    reg.plot_wheel_momenta = @plot_wheel_momenta;
    reg.decompose = @decompose;
    reg.plot_rotations = @plot_rotations;
    reg.momenta_slope = @momenta_slope;
    reg.analysis = @analysis;
end

function [times, errors, momenta, h_dots, X] = reg(J, w_0, q_c, q_time, M, M_time, T, k_p, k_d)

    vect = vector;
    quat = quaternion;

    % Simulation Parameters
    n = 20000;
    dt = T / n;
    times = linspace(0, T, n);
    
    if isempty(M_time)
        M_n = [1, n];
    else
        M_n = [find(times >= M_time(1), 1), find(times >= M_time(2), 1)];
    end
    
    if isempty(q_time)
        q_c_n = [1, n];
    else
        q_c_n = [find(times >= q_time(1), 1), find(times >= q_time(2), 1)];
    end
    M_ref = M;
    
    errors = zeros(n, 4);
    momenta = zeros(n, 3);
    h_dots = zeros(n, 3);

    % Initial Conditions
    h_0 = [0, 0, 0]';
    q_0 = [0, 0, 0, 1]';
    
    % Initial Basis for display
    X = zeros(n, 3, 3);
    x_0 = eye(3);

    % Reaction Wheel Simulation
    h = h_0;
    q = q_0;
    w = w_0;
    
    fprintf('Simulating ---------------------------------------------------\n');
    fprintf('Total Time : %d, Moment : %0.5d, k_p : %d, k_d : %d\n', ...
        T, norm(M), k_p, k_d);
    fprintf('%-40s : %.5d Nms\n', 'Total Initial Satellite Momenta', ...
            norm(J * w_0));
        
    for t = 1:n
        
        % Controller
        dq = quat.dq(q, q_c);
        if and(t >= q_c_n(1), t <= q_c_n(2))
            L = - k_p * sign(dq(4)) * dq(1:3) - k_d * w;
        else
            L = [0, 0, 0]';
        end

        if and(t >= M_n(1), t <= M_n(2))
            M = M_ref;
        else
            M = [0, 0, 0]';
        end
        
        % Integrations
        w_dot = - inv(J) * (vect.cross(w) * J * w - L - M);
        h_dot = - vect.cross(w) * h - L;
        q_dot = (1/2) * quat.xi(q) * w;

        q = q + q_dot * dt;
        q = q / norm(q);
        w = w + w_dot * dt;
        h = h + h_dot * dt;
                
        X(t, :, :) = quat.A(q) * x_0;
        errors(t, :) = dq;
        momenta(t, :) = h;
        h_dots(t, :) = h_dot;

    end
    
    fprintf('Simulation Complete ------------------------------------------\n\n');
    
end

function none = analysis(times, momenta, h_dots, pyramid, nasa)
    reg = regulate;
    pyramid_slopes = reg.momenta_slope(times, pyramid);
    nasa_slopes = reg.momenta_slope(times, nasa);
    fprintf('%-40s : %.5d Nms\n', ...
        'Final Control Momenta Magnitude', norm(momenta(end, :)));
    fprintf('%-40s : %.5d Nms\n', ...
        'Maximum Control Momenta Magnitude', max(vecnorm(momenta')));
    fprintf('%-40s : %.5d Nm\n', ...
        'Final Control Torque Magnitude', norm(h_dots(end, :)));
    fprintf('%-40s : %.5d Nm\n', ...
        'Maximum Control Torque Magnitude', max(vecnorm(h_dots')));
    fprintf('--------------------------------------------------------------\n');
    fprintf('Pyramid Wheel Configuration:\n');
    fprintf('%-40s : %.5d Nms\n', ...
        'Maximum Momenta Magnitude', max(max(abs(pyramid))));
    fprintf('%-40s : %.5d Nms\n', ...
        'Final Momenta Magnitude', max(pyramid(:, end)));
    fprintf('%-40s : %.5d Nms\n', ...
        'Maximum Momenta Accumulation Rate', max(abs(pyramid_slopes)));
    fprintf('--------------------------------------------------------------\n');
    fprintf('NASA Wheel Configuration:\n');
    fprintf('%-40s : %.5d Nms\n', ...
        'Maximum Momenta Magnitude', max(max(abs(nasa))));
    fprintf('%-40s : %.5d Nms\n', ...
        'Final Momenta Magnitude', max(nasa(:, end)));
    fprintf('%-40s : %.5d Nms\n', ...
        'Maximum Momenta Accumulation Rate', max(abs(nasa_slopes)));
    none = [];
end

function slopes = momenta_slope(times, momenta)
    n = size(momenta);
    n = n(1);
    slopes = zeros(n, 2);
    for i = 1:n
        slopes(i, :) = polyfit(times(:, 500:end), momenta(i, 500:end), 1);
    end
    slopes = slopes(:, 1);
end

function f = plot_momenta(times, momenta, H_dots)
    reg = regulate;
    f = figure('visible', 'off');
    f.Position = [400 200 800 600];
%     title('Maneuver Errors and Total Wheel Momenta')
%     subplot(2, 1, 1);
%     plot(times, errors);
%     grid on;
%     yline(0,'k--');
%     yline(1,'k--');
%     ylabel('Quaternion Errors');
%     xlabel('Time (s)');
%     legend('q_1', 'q_2', 'q_3', 'q_4', 'Location', 'east');
    subplot(2, 1, 1);
    hold on;
    plot(times, momenta);
    grid on;
    yline(0, 'k--');
    ylabel('Total Control Momenta (Nms)');
    xlabel('Time (s)');
    legend('h_1', 'h_2', 'h_3', 'Location', 'east');
    subplot(2, 1, 2);
    hold on;
    plot(times, H_dots);
    grid on;
    ylabel('Total Control Torques (Nm)');
    xlabel('Time (s)');
    legend('T_1', 'T_2', 'T_3', 'Location', 'east');
end

function [pyramid, nasa] = decompose(momenta)
    W_pyramid = [1, -1, 0, 0;
                 1, 1, 1, 1;
                 0, 0, 1, -1] * (1 / sqrt(2));
    W_NASA = [1, 0, 0, 1 / sqrt(3); 
              0, 1, 0, 1 / sqrt(3);
              0, 0, 1, 1 / sqrt(3)];
    pyramid = pinv(W_pyramid) * momenta';
    nasa = pinv(W_NASA) * momenta';
end

function f = plot_wheel_momenta(times, pyramid, nasa)
    f = figure('visible', 'off');
    f.Position = [400 200 800 600];
    title('Wheel Momenta Comparison')
    subplot(2, 1, 1);
    plot(times, pyramid);
    grid on;
    yline(0,'k--');
    ylabel({'Pyramid Configuration', 'Wheel Momenta (Nms)'});
    xlabel('Time (s)');
    legend('wheel 1', 'wheel 2', 'wheel 3', 'wheel 4', 'Location', 'east');
    subplot(2, 1, 2);
    plot(times, nasa);
    grid on;
    yline(0,'k--');
    ylabel({'Nasa Configuration', 'Wheel Momenta (Nms)'});
    xlabel('Time (s)');
    legend('wheel 1', 'wheel 2', 'wheel 3', 'wheel 4', 'Location', 'east');
end

function f = plot_rotations(X)
    origin = [0, 0, 0]';
    X_0 = squeeze(X(1, :, :));
    X_f = squeeze(X(end, :, :));

    f = figure('visible', 'off');
    f.Position = [400 200 700 600];
    title('Satellite Rotation Trajectory')
    hold on; view(3);
    view(135, 30);
    xlim([-1, 1]); ylim([-1, 1]), zlim([-1, 1]);
    plot3(X(:, 1, 1), X(:, 1, 2), X(:, 1, 3), ...
          X(:, 2, 1), X(:, 2, 2), X(:, 2, 3), ...
          X(:, 3, 1), X(:, 3, 2), X(:, 3, 3));
    quiver3(origin, origin, origin, X_0(:, 1), X_0(:, 2), X_0(:, 3), 'b');
    quiver3(origin, origin, origin, X_f(:, 1), X_f(:, 2), X_f(:, 3), 'r');
    legend('X axis rotation', 'Y axis rotation', 'Z axis rotation', ...
        'q_0', 'q_f');
end