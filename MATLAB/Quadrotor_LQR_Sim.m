% Quadrotor Surveillance and Capture Controller
clc;
clear all;
close all;

% Global variables for catching and bringing back the drone
global catched_uav;
global returned_home;
global escaped_uav;
global nest_position;
global out_of_airspace;

%% Define Simulation Parameters
global num_points;
global total_sim_time;
num_points = 500;
total_sim_time = 10;

%% Define system parameters
g = 9.81;   % gravitational acceleration [m/s^2]
l = 0.2;    % distance from the center of mass to each rotor [m]
m = 0.5;    % total mass of the quadrotor [kg]
I = [1.24, 1.24, 2.48]; % mass moment of inertia [kg m^2]
mu = 3.0;   % maximum thrust of each rotor [N]
sigma = 0.01; % proportionality constant relating thrust to torque [m]
I11 = 1.24;
I22 = 1.24;
I33 = 2.48;
p = [g, l, m, sigma];

%% Initial Disturbance force r and Disturbance Moment n
n = [0, 0, 0].';
r = [0, 0, 0].';

%% Non Linear --> Linear Model

% Define state-space matrices
[A, B] = linearize_quadrotor(p, mu, I, n, r);

% Display system matrices
disp("Matrix A");
disp(A);

disp("Matrix B");
disp(B);


%% Quadrotor Linearized around the Hover State. 
A_hover =  [0, 0, 0,            0,           0,    0, 1, 0, 0, 0, 0, 0
            0, 0, 0,            0,           0,    0, 0, 1, 0, 0, 0, 0;
            0, 0, 0,            0,           0,    0, 0, 0, 1, 0, 0, 0;
            0, 0, 0,            0,           0,    0, 0, 0, 0, 1, 0, 0;
            0, 0, 0,            0,           0,    0, 0, 0, 0, 0, 1, 0;
            0, 0, 0,            0,           0,    0, 0, 0, 0, 0, 0, 1;
            0, 0, 0,            0,           g,    0, 0, 0, 0, 0, 0, 0;
            0, 0, 0,           -g,           0,    0, 0, 0, 0, 0, 0, 0;
            0, 0, 0,            0,           0,    0, 0, 0, 0, 0, 0, 0;
            0, 0, 0,            0,           0,    0, 0, 0, 0, 0, 0, 0;
            0, 0, 0,            0,           0,    0, 0, 0, 0, 0, 0, 0;
            0, 0, 0,            0,           0,    0, 0, 0, 0, 0, 0, 0]; 
 
B_hover = [        0,          0,         0,          0;
                   0,          0,         0,          0;
                   0,          0,         0,          0;
                   0,          0,         0,          0;
                   0,          0,         0,          0;
                   0,          0,         0,          0;
                   0,          0,         0,          0;
                   0,          0,         0,          0;
                 1/m,        1/m,       1/m,        1/m;
                   0,      l/I11,         0,     -l/I11;
              -l/I22,          0,     l/I22,          0;
           sigma/I33, -sigma/I33, sigma/I33, -sigma/I33]; 

%% Check controllability
ControllabilityMatrix = ctrb(A, B);
ControllabilityRank = rank(ControllabilityMatrix);
disp("Controllability rank")
disp(ControllabilityRank)

%% Use A Hover ? Commented - Non Linear Dynamics in Use
% If needed can use Linear Dynamics which A_hover, B_hover
A = A_hover;
%B = B_hover;

%% LQR Gain Calculation for Initial Linearization

% Solve the continuous-time algebraic Riccati equation for LQR gains
Q = diag([1000,1000, 1000, 100, 100, 100, 100, 100, 100, 10000, 10000, 10000]);
R1 = eye(4)*0.001 ;
N = 0;
K = lqr(A, B, Q, R1);


%% LQR with integrator 
A_aug = [A, zeros(size(A));];


%% Defintion of Variables

uav_entrypoint = [1;-1;3];
uav_exitpoint = [-5;5;8];

n_disturb = [0.5, 0.5, 0.5].';
r_disturb = [1, 1, 1].';




catched_uav = false;
escaped_uav = false;
returned_home = false;
out_of_airspace = false;

nest_position = [0;0;0];

nest_orientation = [0; 0; 0];
nest_state = [nest_position; nest_orientation; zeros(6, 1)];


% Time vector
t_span = linspace(0, total_sim_time, num_points);


helix_radius = 0.3;
helix_pitch = 0.1;

%% TEST DRone Yet to Complete


% %% 
% 
% % Now use ode45 with surveillance_controller
% [t, z] = ode45(@(t, z) quadrotor_surveillance_controller(t, z, K, mu, p, I, r, n, uav_trajectory), t_span, nest_state);


%% Random UAV Trajectory

% Generate random UAV trajectory
uav_trajectory = generate_random_trajectory_with_spline(uav_entrypoint, uav_exitpoint, total_sim_time, num_points)
% Helical
%uav_trajectory = generate_helical_trajectory(uav_entrypoint, uav_exitpoint, total_sim_time, num_points, helix_radius, helix_pitch);
% Helical with Circle
%uav_trajectory = generate_helix_trajectory([1,1,1], total_sim_time, num_points, helix_radius, helix_pitch);

%% Control Loop Iteration

z_total = [nest_state];
current_state = nest_state;
disp("Starting at Z");
disp(z_total(end))
temp_disturb_init = 0;
time_catch = 0;
time_home = 0;
for k = 2: length(t_span)
    disp("Iteration")
    disp(k)
    disp("CURRENT CATCH STATUS:")
    disp(catched_uav);
    disp("CURRENT RETURN HOME STATUS")
    disp(returned_home);
    time_home = k;
    if returned_home == true 
        
        break;
    end

    if catched_uav == false
        kspan = linspace(t_span(k-1), t_span(k), 50);
        disp("KSPAN")
        disp(kspan(end)-kspan(1))
        uav_position = uav_trajectory(k,:);
        uav_position = uav_position(2:4);
        disp("Current UAV Position")
        disp(uav_position);
        disp("Current Drone Position")
        curr_drone_position = z_total(:,end);
        disp(curr_drone_position);
        z_new = update_states(curr_drone_position, K, mu, p, I, r, n, uav_position, kspan);
        z_total = [z_total,z_new];
        time_catch = k
    end

    if catched_uav == true || out_of_airspace == true
        if temp_disturb_init < 2
            new_uav_trajectory = generate_random_trajectory_with_spline(uav_position.', nest_position, 3, num_points-k);
        end
        if temp_disturb_init < 2
            [A, B] = linearize_quadrotor(p, mu, I, n_disturb, r_disturb);
            Q = diag([1000,1000, 1000, 100, 100, 100, 100, 100, 100, 10000, 10000, 10000]);
            R1 = eye(4)*0.01 ;
            K_new = lqr(A, B, Q, R1);
        end
        disp(size(z_total)); 
        kspan = linspace(t_span(k-1), t_span(k), 50);
        
        uav_position = new_uav_trajectory(k,:);
        uav_position = uav_position(2:4);
        disp("Current UAV Position")
        disp(uav_position);
        disp("Current Drone Position")
        disp("======================================")
        disp("GOING TOWARDS NEST POSITION")
        curr_drone_position = z_total(:,end);
        disp(curr_drone_position);
        z_return = update_states(curr_drone_position, K, mu, p, I, r, n, uav_position, kspan);
        z_total = [z_total,z_return];
        temp_disturb_init = temp_disturb_init + 1;
    end

    
    current_state = z_total(end);

end

%% Plotting results 
t = linspace(0, total_sim_time, k);
z = z_total.';

disp("Sizes of t and z")
disp(size(t))
disp(size(z))
temp_t = min(num_points,k);

t= t(1:temp_t);
z = z(1:temp_t,:);


% figure;
% 
% % Plot position
% subplot(3, 1, 1);
% plot(t, z(:, 1:3));
% title('Position');
% xlabel('Time (s)');
% ylabel('Position (m)');
% legend('x', 'y', 'z');
% 
% % Plot orientation
% subplot(3, 1, 2);
% plot(t, z(:, 4:6));
% title('Orientation');
% xlabel('Time (s)');
% ylabel('Orientation (rad)');
% legend('\phi', '\theta', '\psi');
% 
% % Plot control inputs
% subplot(3, 1, 3);
% u = zeros(length(t), 4);
% for i = 1:length(t)
%     u(i, :) = -K * (z(i, :)' - [uav_exitpoint; zeros(9, 1)]);
% end
% plot(t, u);
% title('Control Inputs');
% xlabel('Time (s)');
% ylabel('Control Input');
% legend('u1', 'u2', 'u3', 'u4');
% Extract Quadrotor Position Data

% Extract Quadrotor Position Data
quadrotor_x = z_total(1, :);
quadrotor_y = z_total(2, :);
quadrotor_z = z_total(3, :);



quadrotor_x_1 = z_total(4, :);
quadrotor_y_1 = z_total(5, :);
quadrotor_z_2 = z_total(6, :);

% Extract UAV Position Data
uav_x = new_uav_trajectory(:, 2);
uav_y = new_uav_trajectory(:, 3);
uav_z = new_uav_trajectory(:, 4);

% Time vector (make sure this aligns with your data)
t_quadrotor = linspace(0, total_sim_time, length(quadrotor_x)); % for quadrotor
t_uav = new_uav_trajectory(:, 1); % assuming the first column is time

% Plotting X Position
figure;
subplot(3, 1, 1);
plot(t_quadrotor, quadrotor_x, 'b', t_uav, uav_x, 'r--');
title('X Position of Quadrotor and UAV');
xlabel('Time (s)');
ylabel('X Position (m)');
legend('Quadrotor', 'UAV');

% Plotting Y Position
subplot(3, 1, 2);
plot(t_quadrotor, quadrotor_y, 'b', t_uav, uav_y, 'r--');
title('Y Position of Quadrotor and UAV');
xlabel('Time (s)');
ylabel('Y Position (m)');
legend('Quadrotor', 'UAV');

% Plotting Z Position
subplot(3, 1, 3);


plot(t_quadrotor, quadrotor_z, 'b', t_uav, uav_z, 'r--');
title('Z Position of Quadrotor and UAV');
xlabel('Time (s)');
ylabel('Z Position (m)');
legend('Quadrotor', 'UAV');


%% Animation

try
    disp("SIze of total figure")
    disp(size(z))
    disp(size(uav_trajectory))
    disp(size(new_uav_trajectory))
    disp(size(uav_trajectory(1:time_catch,:)))
    disp(size(new_uav_trajectory(1:time_home,:)))
    uav_trajectory = [uav_trajectory(1:time_catch,:).', new_uav_trajectory(1:time_home,:).'];
    disp(size(uav_trajectory))
    uav_trajectory = uav_trajectory.';
    disp(size(uav_trajectory))
catch exception
    uav_trajectory = uav_trajectory;
end



animation_fig = figure;

airspace_box_length = 10;

animation_axes = axes('Parent', animation_fig, ...
    'NextPlot', 'add', 'DataAspectRatio', [1 1 1], ...
    'Xlim', airspace_box_length * [-0.5 0.5], ...
    'Ylim', airspace_box_length * [-0.5 0.5], ...
    'Zlim', airspace_box_length * [0 1], ...
    'box', 'on', 'Xgrid', 'on', 'Ygrid', 'on', 'Zgrid', 'on', ...
    'TickLabelInterpreter', 'LaTeX', 'FontSize', 14);

view(animation_axes, 3);

% Enable rotation
rotate3d(animation_fig, 'on');
% Initialize variables to store the history path
uav_history_path = zeros(1, 3);
drone_history_path = zeros(1, 3);

tic;
for k = 1:length(t)
    N = 10;
    Q = linspace(0,2*pi, N)';
    circle = 0.3* l* [cos(Q) sin(Q) zeros(N,1)];
    loc = l *[1 0 0; 0 1 0; -1 0 0 ; 0 -1 0];

    silhouette = plot3(0, 0, 0, '--', 'Color', 0.5 * [1 1 1], 'LineWidth', 1, ...
        'Parent', animation_axes);
    body = plot3(0, 0, 0, 'Color', lines(1), 'LineWidth', 2, ...
        'Parent', animation_axes);

    for i = 1:4
        rotor(i) = plot3(0, 0, 0, 'Color', lines(1), 'LineWidth', 2, ...
        'Parent', animation_axes);
    end

        R = [cos(z(k, 5)) * cos(z(k, 6)), sin(z(k, 4)) * sin(z(k, 5)) * cos(z(k, 6)) - cos(z(k, 4)) * sin(z(k, 6)), sin(z(k, 4)) * sin(z(k, 6)) + cos(z(k, 4)) * sin(z(k, 5)) * cos(z(k, 6));
        cos(z(k, 5)) * sin(z(k, 6)), cos(z(k, 4)) * cos(z(k, 6)) + sin(z(k, 4)) * sin(z(k, 5)) * sin(z(k, 6)), cos(z(k, 4)) * sin(z(k, 5)) * sin(z(k, 6)) - sin(z(k, 4)) * cos(z(k, 6));
        -sin(z(k, 5)), sin(z(k, 4)) * cos(z(k, 5)), cos(z(k, 4)) * cos(z(k, 5))];

    for i = 1:4
        ctr(i, :) = z(k, 1:3) + loc(i, :) * R';
        pose = ones(N, 1) * z(k, 1:3) + (ones(N, 1) * loc(i, :) + circle) * R';
        if k == 1
            rotor(i) = plot3(pose(:, 1), pose(:, 2), pose(:, 3), 'Color', lines(1), 'LineWidth', 2, ...
                'Parent', animation_axes);
        else
            set(rotor(i), 'XData', pose(:, 1), 'YData', pose(:, 2), 'ZData', pose(:, 3));
        end
    end

    set(silhouette, 'XData', [0, z(k, 1), z(k, 1), z(k, 1)], ...
        'YData', [0, 0, z(k, 2), z(k, 2)], ...
        'ZData', [0, 0, 0, z(k, 3)]);

    set(body, 'XData', [ctr([1 3], 1); NaN; ctr([2 4], 1)], ...
        'YData', [ctr([1 3], 2); NaN; ctr([2 4], 2)], ...
        'ZData', [ctr([1 3], 3); NaN; ctr([2 4], 3)]);
    
    % Update history path variables
    uav_history_path = [uav_history_path; uav_position];
    drone_history_path = [drone_history_path; z(k, 1:3)];
    % Plot the UAV history path
    plot3(uav_history_path(:, 1), uav_history_path(:, 2), uav_history_path(:, 3), 'Color', 'b', 'LineWidth', 1);
    
    % Plot the drone history path
    plot3(drone_history_path(:, 1), drone_history_path(:, 2), drone_history_path(:, 3), 'Color', 'r', 'LineWidth', 1);
    
    % Plot the random UAV trajectory
    if k<11
    plot3(uav_trajectory(1:k, 2), uav_trajectory(1:k, 3), uav_trajectory(1:k, 4), 'g', 'LineWidth', 2);
    else
        plot3(uav_trajectory(k-10:k, 2), uav_trajectory(k-10:k, 3), uav_trajectory(k-10:k, 4), 'g', 'LineWidth', 2);
    end
    % Plot the black drone circle at the tip of the UAV trajectory
    drone_tip = plot3(uav_trajectory(k, 2), uav_trajectory(k, 3), uav_trajectory(k, 4), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
    %set(uav_trajectory_plot, 'XData', uav_trajectory(1:k, 2), 'YData', uav_trajectory(1:k, 3), 'ZData', uav_trajectory(1:k, 4));
    
    % Plot the position
    scatter3(z(k, 1), z(k, 2), z(k, 3), 'filled', 'MarkerFaceColor', 'r');

    % Plot the orientation (pitch, roll, yaw)
    pitch = z(k, 4);
    roll = z(k, 5);
    yaw = z(k, 6);

    % Adjust the length of the orientation vectors based on your preferences
    % orientation_length = 0.5;
    % pitch_vector = orientation_length * [cos(pitch) * cos(yaw), cos(pitch) * sin(yaw), -sin(pitch)];
    % roll_vector = orientation_length * [sin(roll) * sin(yaw) - cos(roll) * cos(yaw) * sin(pitch), sin(roll) * cos(yaw) + cos(roll) * sin(yaw) * sin(pitch), cos(roll) * cos(pitch)];
    % yaw_vector = orientation_length * [cos(roll) * sin(yaw) + sin(roll) * cos(yaw) * sin(pitch), cos(roll) * cos(yaw) - sin(roll) * sin(yaw) * sin(pitch), -sin(roll) * cos(pitch)];
    % 
    % % Plot the orientation vectors
    % quiver3(z(k, 1), z(k, 2), z(k, 3), pitch_vector(1), pitch_vector(2), pitch_vector(3), 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    % quiver3(z(k, 1), z(k, 2), z(k, 3), roll_vector(1), roll_vector(2), roll_vector(3), 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    % quiver3(z(k, 1), z(k, 2), z(k, 3), yaw_vector(1), yaw_vector(2), yaw_vector(3), 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    % 

    % Interpolate UAV position from the trajectory at the current time
    uav_position = uav_trajectory(k,1:3);
    % Print the current distance to UAV in x, y, and z
    distance_x = z(k, 1) - uav_position(1);
    distance_y = z(k, 2) - uav_position(2);
    distance_z = z(k, 3) - uav_position(3);

%     distance_str = sprintf('Distance to UAV: %.2f (x), %.2f (y), %.2f (z)', distance_x, distance_y, distance_z);
%     distance_text = text(-2, -2, 1, distance_str, 'FontSize', 12, 'Color', 'r');


    pause(t(k) - toc);
    pause(0.008);

    
    if k< length(t)
        delete(drone_tip);
        
        cla;
    end

end
    


%%% HELPER FUNCTIONS

%% QUADROTOR CONTROL FUNCTION
% Define surveillance controller
function dz = quadrotor_surveillance_controller(t, z, K, mu, p, I, r, n, uav_trajectory)
    
    global catched_uav;
    global returned_home;
    global escaped_uav;
    global nest_position;
    global num_points;
    global total_sim_time;
    global out_of_airspace;

    % State vector definition
    % z = [x, y, z, phi, theta, psi, dx, dy, dz, p, q, r]
    I = diag(I);

    % Rotation matrix mapping body fixed frame C to inertial frame E
    R = [cos(z(5)) * cos(z(6)), sin(z(4)) * sin(z(5)) * cos(z(6)) - cos(z(4)) * sin(z(6)), sin(z(4)) * sin(z(6)) + cos(z(4)) * sin(z(5)) * cos(z(6));
        cos(z(5)) * sin(z(6)), cos(z(4)) * cos(z(6)) + sin(z(4)) * sin(z(5)) * sin(z(6)), cos(z(4)) * sin(z(5)) * sin(z(6)) - sin(z(4)) * cos(z(6));
        -sin(z(5)), sin(z(4)) * cos(z(5)), cos(z(4)) * cos(z(5))];

    % Interpolate UAV position from the trajectory
    uav_position = interp1(uav_trajectory(:, 1), uav_trajectory(:, 2:4), t, 'linear', 'extrap');

    % Check distance between UAV and controller
    distance_threshold = 1.0; % Set your desired distance threshold here
    distance_to_uav = norm(z(1:3) - uav_position);
    distance_x = z(1) - uav_position(1);
    distance_y = z(2) - uav_position(2);
    distance_z = z(3) - uav_position(3);

    d_nestx = z(1) - nest_position(1);
    d_nesty = z(2) - nest_position(2);
    d_nestz = z(3) - nest_position(3);

    disp("STATUS")
    disp("CURRENT DISTANCE TO UAV:")
    disp(distance_to_uav);
    disp("CURRENT CATCH STATUS:")
    disp(catched_uav);
    disp("CURRENT RETURN HOME STATUS")
    disp(returned_home);

    if (distance_x <0.8) && (distance_y<0.8) && (distance_z<0.8)
        catched_uav = true;

        % Break the simulation loop
        return;
    end

    if (catched_uav == true || escaped_uav == true) && (d_nestx < 1 && d_nesty < 1 && d_nestz <1)
        returned_home = true;
        return;
    end

    if (distance_x >10) && (distance_y >10) && (distance_z >10)
        escaped_uav = true;
    end 
    
    if abs(z(1)) > 4.5 || abs(z(2)) >5 || abs(z(3)) > 8
        out_of_airspace = true;
    end
        
    % Continue with the surveillance controller
    % Adjusting thrust output based on feasible limits
    u = max(min(-K * (z - [uav_position'; zeros(9, 1)]), mu), 0);
    % Computing time derivative of the state vector
    % rt = torque vector induced by rotor thrusts
    rt = [(u(2) - u(4)) * p(2);
          (u(3) - u(1)) * p(2);
          (u(1) - u(2) + u(3) - u(4)) * p(4)];

    dz(1:3, 1) = z(7:9);

    dz(4:6, 1) = [z(10) + z(12) * cos(z(4)) * tan(z(5)) + z(11) * sin(z(4)) * tan(z(5));
                 z(11) * cos(z(4)) - z(12) * sin(z(4));
                 (z(12) * cos(z(4)) + z(11) * sin(z(4))) / cos(z(5))];

    dz(7:9, 1) = R * ([0; 0; sum(u)] + r) / p(3) - [0; 0; p(1)];

    dz(10:12, 1) = I \ (rt + n - cross(z(10:12, 1), I * z(10:12, 1)));
end

function z_new = update_states(z_prev, K, mu, p, I, r, n, uav_position, t_span)
    global catched_uav;
    global returned_home;
    global escaped_uav;
    global nest_position;

    I = diag(I);

    % Rotation matrix mapping body fixed frame C to inertial frame E
    R = [cos(z_prev(5)) * cos(z_prev(6)), sin(z_prev(4)) * sin(z_prev(5)) * cos(z_prev(6)) - cos(z_prev(4)) * sin(z_prev(6)), sin(z_prev(4)) * sin(z_prev(6)) + cos(z_prev(4)) * sin(z_prev(5)) * cos(z_prev(6));
        cos(z_prev(5)) * sin(z_prev(6)), cos(z_prev(4)) * cos(z_prev(6)) + sin(z_prev(4)) * sin(z_prev(5)) * sin(z_prev(6)), cos(z_prev(4)) * sin(z_prev(5)) * sin(z_prev(6)) - sin(z_prev(4)) * cos(z_prev(6));
        -sin(z_prev(5)), sin(z_prev(4)) * cos(z_prev(5)), cos(z_prev(4)) * cos(z_prev(5))];

    distance_x = abs(z_prev(1) - uav_position(1));
    distance_y = abs(z_prev(2) - uav_position(2));
    distance_z = abs(z_prev(3) - uav_position(3));

    d_nestx = abs(z_prev(1) - nest_position(1));
    d_nesty = abs(z_prev(2) - nest_position(2));
    d_nestz = abs(z_prev(3) - nest_position(3));

    disp("STATUS")
    disp("CURRENT DISTANCE TO UAV:")
    disp(distance_x);
    disp(distance_y);
    disp(distance_z);




    % Adjusting thrust output based on feasible limits
    u = max(min(-K * (z_prev - [uav_position'; zeros(9, 1)]), mu), 0);
    %u = -K * (z_prev - [uav_position'; zeros(9, 1)]);

    
    disp("Display of Shapes")
    disp(size(z_prev))
    disp(size(uav_position))
    % Computing time derivative of the state vector
    % rt = torque vector induced by rotor thrusts
    rt = [(u(2) - u(4)) * p(2);
          (u(3) - u(1)) * p(2);
          (u(1) - u(2) + u(3) - u(4)) * p(4)];

    dz = zeros(12, 1);
    dz(1:3) = z_prev(7:9);
    dz(4:6) = [z_prev(10) + z_prev(12) * cos(z_prev(4)) * tan(z_prev(5)) + z_prev(11) * sin(z_prev(4)) * tan(z_prev(5));
               z_prev(11) * cos(z_prev(4)) - z_prev(12) * sin(z_prev(4));
               (z_prev(12) * cos(z_prev(4)) + z_prev(11) * sin(z_prev(4))) / cos(z_prev(5))];
    dz(7:9) = R * ([0; 0; sum(u)] + r) / p(3) - [0; 0; p(1)];
    dz(10:12) = I \ (rt + n - cross(z_prev(10:12), I * z_prev(10:12)));
    disp("DZ in each step")
    disp(dz)

    % Integrate using Euler's method
    z_new = z_prev + dz * (t_span(end) - t_span(1));
    
    if (distance_x <0.8) && (distance_y<0.8) && (distance_z<0.8)
        catched_uav = true;
        % Break the simulation loop
        return;
    end

    if (catched_uav == true || escaped_uav == true) && (d_nestx < 0.1 && d_nesty < 0.1 && d_nestz <0.1)
        returned_home = true;
        return;
    end

    if (abs(distance_x) > 10) || (abs(distance_y) >10) || (abs(distance_z) >10)
        escaped_uav = true;
    end 
end


%% TRAJECTORY SPLINE FUNCTION

function trajectory_spline = generate_random_trajectory_with_spline(initial_position, final_position, total_sim_time, num_points)
    % Generate a spline trajectory with respect to time between initial and final positions
    global catched_uav;
    global returned_home;
    global escaped_uav;
    global nest_position;
    global num_points;
    global total_sim_time;
    % Generate time vector
    time_vector = linspace(0, total_sim_time, num_points);

    % Generate random parameter values
    t = linspace(0, 1, num_points);
    
    disp("SHAPES- RANDOM TRAJECTORY")
    disp(size(t.'))
    disp(size((1-t).'))
    disp(size(initial_position))
    disp(size(final_position))
    disp(size((1 - t).' * initial_position.'))
    disp(size(t.' * final_position.'))

    % Interpolate between initial and final positions with respect to time
    trajectory = (1 - t).' * initial_position.' + t.' * final_position.';

    % Add random noise to the trajectory
    noise_factor = 0; % Adjust this factor based on the desired noise level
    noise = noise_factor * randn(num_points, 3);
    trajectory = trajectory + noise;

    % Create a spline interpolation for each coordinate
    spline_x = spline(time_vector, trajectory(:, 1));
    spline_y = spline(time_vector, trajectory(:, 2));
    spline_z = spline(time_vector, trajectory(:, 3));

    % Evaluate the spline at points 1 to length(time_vector)
    spline_points = [ppval(spline_x, time_vector)', ppval(spline_y, time_vector)', ppval(spline_z, time_vector)'];

    % Return the spline trajectory
    trajectory_spline = [time_vector', spline_points];

    % Add time vector as the first column
    % trajectory_spline = [time_vector', spline_x, spline_y, spline_z];
end

%% Function for future expectation
function future_expectations = future_expectation(data, window_size, steps_ahead)
    % Calculate moving average
    moving_avg = filter(ones(1, window_size)/window_size, 1, data, 'valid');

    % Extrapolate future expectations
    future_expectations = zeros(1, steps_ahead);
    for i = 1:steps_ahead
        extrapolated_value = moving_avg(end);  % Assume the last moving average value continues
        future_expectations(i) = extrapolated_value;
        moving_avg = [moving_avg, extrapolated_value];
    end
end


%% Helical trajectory
function trajectory = generate_helical_trajectory(initial_point, final_point, total_sim_time, num_points, radius, pitch)
    % Generate a helical trajectory between initial and final points

    % Generate time vector
    time_vector = linspace(0, total_sim_time, num_points);

    % Parametric equation for a helix
    t = linspace(0, 2 * pi, num_points);
    x_helix = cos(t) * radius;
    y_helix = sin(t) * radius;
    z_helix = t * pitch;

    % Interpolate between initial and final points with respect to time
    trajectory = [(1 - t.' / (2 * pi)) * initial_point(1) + t.' / (2 * pi) * final_point(1), ...
                  (1 - t.' / (2 * pi)) * initial_point(2) + t.' / (2 * pi) * final_point(2), ...
                  (1 - t.' / (2 * pi)) * initial_point(3) + t.' / (2 * pi) * final_point(3)] + [x_helix.', y_helix.', z_helix.'];

    % Add random noise to the trajectory if needed
    noise_factor = 0; % Adjust this factor based on the desired noise level
    noise = noise_factor * randn(num_points, 3);
    trajectory = trajectory + noise;

    % Return the trajectory
    trajectory = [time_vector.', trajectory];
end


function trajectory = generate_helix_trajectory(center, total_sim_time, num_points, radius, pitch)
    % Generate a helix trajectory with respect to time

    % Generate time vector
    time_vector = linspace(0, total_sim_time, num_points);

    % Parametric equation for a helix
    t = linspace(0, 2 * pi, num_points);
    x_helix = center(1) + radius * cos(t);
    y_helix = center(2) + radius * sin(t);
    z_helix = center(3) + pitch * t;

    % Combine helix coordinates into the trajectory
    trajectory = [x_helix.', y_helix.', z_helix.'];

    % Add random noise to the trajectory if needed
    noise_factor = 0; % Adjust this factor based on the desired noise level
    noise = noise_factor * randn(num_points, 3);
    trajectory = trajectory + noise;

    % Return the trajectory
    trajectory = [time_vector.', trajectory];
end

