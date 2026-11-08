%% ============================================================
% Guagua Yuan Weeding Robot
% Straight Line + Strong Curve Following
%
% Mathematical convention:
% +x = robot forward
% +y = robot left
%
% Display convention:
% vertical axis   = world x
% horizontal axis = world y
% +y appears LEFT
%% ============================================================

clear;
clc;
close all;


%% ============================================================
% 1. ROBOT GEOMETRY
%% ============================================================

L = 0.710;       % front-rear wheel center distance [m]
W = 1.160;       % left-right wheel center distance [m]

a = L/2;
b = W/2;

Rw = 0.20;       % wheel rolling radius [m]


%% ============================================================
% 2. VISUAL WHEEL SIZE
%% ============================================================

wheelLength = 0.22;
wheelWidth  = 0.09;


%% ============================================================
% 3. SIMULATION PARAMETERS
%% ============================================================

dt = 0.1;

T = 35;

time = 0:dt:T;

% Slightly slower because curve is tighter
v_cmd = 0.20;       % [m/s]


%% ============================================================
% 4. DESIRED PATH
%
% First:
%       straight vertical line
%
% Then:
%       strong curved section
%
% Since x is plotted vertically:
%
% y = 0
%
% appears as a vertical line.
%% ============================================================

x_straight_end = 2.0;      % straight line length [m]

% Stronger curve
A = 1.00;                  % curve amplitude [m]
k_path = 1.10;             % curvature frequency [1/m]


%% Create reference path for plotting

x_path = linspace(0,8,800);

y_path = zeros(size(x_path));


for j = 1:length(x_path)

    if x_path(j) <= x_straight_end

        % Straight section
        y_path(j) = 0;

    else

        % Curved section
        s = x_path(j) - x_straight_end;

        y_path(j) = ...
            A*(1 - cos(k_path*s));

    end

end


%% ============================================================
% 5. INITIAL WORLD POSE
%% ============================================================

xW = 0;
yW = 0;

% Robot initially faces straight upward (+x)
theta = 0;


%% ============================================================
% 6. PASSIVE JOINT CONFIGURATION
%
% Locked during normal driving
%% ============================================================

phi_L = 0;
phi_R = 0;

phi_L_dot = 0;
phi_R_dot = 0;


%% ============================================================
% 7. HISTORY
%% ============================================================

x_history = [];
y_history = [];


%% ============================================================
% 8. FIGURE
%% ============================================================

figure( ...
    'Color','w', ...
    'Position',[200 80 900 850] ...
);


%% ============================================================
% 9. SIMULATION LOOP
%% ============================================================

for n = 1:length(time)

    t = time(n);


    %% ========================================================
    % A. PATH GEOMETRY
    %% ========================================================

    if xW <= x_straight_end

        %% ----------------------------------------------------
        % STRAIGHT SECTION
        %
        % y = 0
        %
        % dy/dx = 0
        % d2y/dx2 = 0
        %% ----------------------------------------------------

        y_ref = 0;

        dydx = 0;

        d2ydx2 = 0;


    else

        %% ----------------------------------------------------
        % CURVED SECTION
        %
        % y =
        %
        % A [1 - cos(k s)]
        %
        % s = x - x_straight_end
        %% ----------------------------------------------------

        s = xW - x_straight_end;


        y_ref = ...
            A*(1 - cos(k_path*s));


        %% First derivative

        dydx = ...
            A*k_path*sin(k_path*s);


        %% Second derivative

        d2ydx2 = ...
            A*k_path^2*cos(k_path*s);

    end


    %% ========================================================
    % B. PATH CURVATURE
    %% ========================================================

    kappa = ...
        d2ydx2 / ...
        (1 + dydx^2)^(3/2);


    %% ========================================================
    % C. DESIRED GENERALIZED BODY VELOCITY
    %% ========================================================

    vx_des = v_cmd;

    vy_des = 0;


    % Required yaw rate
    %
    % omega = v * curvature

    wz_des = ...
        v_cmd*kappa;


    nu_des = [
        vx_des;
        vy_des;
        wz_des;
        phi_L_dot;
        phi_R_dot
    ];


    %% ========================================================
    % D. PASSIVE JOINT POSITIONS
    %% ========================================================

    P_L_B = [0; b];

    P_R_B = [0;-b];


    %% ========================================================
    % E. WHEEL POSITIONS
    %% ========================================================

    r_FL = [
         a*cos(phi_L);
         b + a*sin(phi_L)
    ];


    r_FR = [
         a*cos(phi_R);
        -b + a*sin(phi_R)
    ];


    r_RL = [
        -a*cos(phi_L);
         b - a*sin(phi_L)
    ];


    r_RR = [
        -a*cos(phi_R);
        -b - a*sin(phi_R)
    ];


    r = [
        r_FL.';
        r_FR.';
        r_RL.';
        r_RR.'
    ];


    %% ========================================================
    % F. PASSIVE JOINT JACOBIAN TERMS
    %% ========================================================

    jL = zeros(4,2);

    jR = zeros(4,2);


    % FL
    jL(1,:) = [
        -a*sin(phi_L), ...
         a*cos(phi_L)
    ];


    % FR
    jR(2,:) = [
        -a*sin(phi_R), ...
         a*cos(phi_R)
    ];


    % RL
    jL(3,:) = [
         a*sin(phi_L), ...
        -a*cos(phi_L)
    ];


    % RR
    jR(4,:) = [
         a*sin(phi_R), ...
        -a*cos(phi_R)
    ];


    %% ========================================================
    % G. BUILD GENERALIZED KINEMATIC MATRIX G
    %% ========================================================

    G = zeros(8,5);


    for i = 1:4

        xi = r(i,1);

        yi = r(i,2);


        jLx = jL(i,1);
        jLy = jL(i,2);

        jRx = jR(i,1);
        jRy = jR(i,2);


        G(2*i-1,:) = [
            1, ...
            0, ...
           -yi, ...
            jLx, ...
            jRx
        ];


        G(2*i,:) = [
            0, ...
            1, ...
            xi, ...
            jLy, ...
            jRy
        ];

    end


    %% ========================================================
    % H. INVERSE KINEMATICS
    %
    % Robot motion -> wheel velocities
    %% ========================================================

    V_wheel_vector = ...
        G*nu_des;


    wheel_velocity = ...
        reshape( ...
        V_wheel_vector, ...
        2, ...
        4 ...
        ).';


    %% ========================================================
    % I. WHEEL DIRECTIONS AND SPEEDS
    %% ========================================================

    beta = zeros(4,1);

    wheel_speed = zeros(4,1);

    wheel_omega = zeros(4,1);


    for i = 1:4

        vix = ...
            wheel_velocity(i,1);


        viy = ...
            wheel_velocity(i,2);


        %% Absolute rolling direction

        beta(i) = ...
            atan2(viy,vix);


        %% Linear wheel speed

        wheel_speed(i) = ...
            hypot(vix,viy);


        %% Wheel angular speed

        wheel_omega(i) = ...
            wheel_speed(i)/Rw;

    end


    %% ========================================================
    % J. LOCAL STEERING ANGLES
    %% ========================================================

    phi_side = [
        phi_L;
        phi_R;
        phi_L;
        phi_R
    ];


    delta = ...
        beta - phi_side;


    delta = ...
        atan2( ...
        sin(delta), ...
        cos(delta) ...
        );


    %% ========================================================
    % K. UPDATE ROBOT WORLD POSE
    %% ========================================================

    x_dot_W = ...
        vx_des*cos(theta) ...
        - vy_des*sin(theta);


    y_dot_W = ...
        vx_des*sin(theta) ...
        + vy_des*cos(theta);


    xW = ...
        xW + x_dot_W*dt;


    yW = ...
        yW + y_dot_W*dt;


    theta = ...
        theta + wz_des*dt;


    %% Save trajectory

    x_history(end+1) = xW;

    y_history(end+1) = yW;


    %% ========================================================
    % L. BODY -> WORLD ROTATION
    %% ========================================================

    R_BW = [
        cos(theta), -sin(theta);
        sin(theta),  cos(theta)
    ];


    robotPosition = [
        xW;
        yW
    ];


    %% ========================================================
    % M. TRANSFORM ROBOT GEOMETRY TO WORLD FRAME
    %% ========================================================

    P_L_W = ...
        robotPosition ...
        + R_BW*P_L_B;


    P_R_W = ...
        robotPosition ...
        + R_BW*P_R_B;


    wheel_W = zeros(4,2);


    for i = 1:4

        p = ...
            robotPosition ...
            + R_BW*r(i,:).';


        wheel_W(i,:) = ...
            p.';

    end


    %% ========================================================
    % N. GLOBAL WHEEL ORIENTATION
    %% ========================================================

    beta_world = ...
        theta + beta;


    side_world = ...
        theta + phi_side;


    %% ========================================================
    % O. DRAW
    %% ========================================================

    clf;

    hold on;

    grid on;

    axis equal;


    % +y points LEFT
    set(gca,'XDir','reverse');


    %% ========================================================
    % REFERENCE PATH
    %% ========================================================

    plot( ...
        y_path, ...
        x_path, ...
        'b--', ...
        'LineWidth',2 ...
    );


    %% ========================================================
    % Highlight straight vertical section
    %% ========================================================

    plot( ...
        [0 0], ...
        [0 x_straight_end], ...
        'b-', ...
        'LineWidth',3 ...
    );


    %% Mark point where curve starts

    plot( ...
        0, ...
        x_straight_end, ...
        'bo', ...
        'MarkerSize',9, ...
        'MarkerFaceColor','b' ...
    );


    text( ...
        -0.1, ...
        x_straight_end+0.15, ...
        'Curve begins', ...
        'FontSize',9 ...
    );


    %% ========================================================
    % ACTUAL ROBOT TRAJECTORY
    %% ========================================================

    plot( ...
        y_history, ...
        x_history, ...
        'k-', ...
        'LineWidth',1.8 ...
    );


    %% ========================================================
    % CENTRAL ROBOT FRAME
    %% ========================================================

    plot( ...
        [P_L_W(2),P_R_W(2)], ...
        [P_L_W(1),P_R_W(1)], ...
        'k-', ...
        'LineWidth',3 ...
    );


    %% Robot center

    plot( ...
        yW, ...
        xW, ...
        'ko', ...
        'MarkerSize',8, ...
        'MarkerFaceColor','k' ...
    );


    %% ========================================================
    % PASSIVE JOINTS
    %% ========================================================

    plot( ...
        P_L_W(2), ...
        P_L_W(1), ...
        'ks', ...
        'MarkerSize',9, ...
        'MarkerFaceColor','w' ...
    );


    plot( ...
        P_R_W(2), ...
        P_R_W(1), ...
        'ks', ...
        'MarkerSize',9, ...
        'MarkerFaceColor','w' ...
    );


    %% ========================================================
    % LEFT SIDE FRAME
    %% ========================================================

    plot( ...
        [wheel_W(1,2),wheel_W(3,2)], ...
        [wheel_W(1,1),wheel_W(3,1)], ...
        'LineWidth',3 ...
    );


    %% ========================================================
    % RIGHT SIDE FRAME
    %% ========================================================

    plot( ...
        [wheel_W(2,2),wheel_W(4,2)], ...
        [wheel_W(2,1),wheel_W(4,1)], ...
        'LineWidth',3 ...
    );


    %% ========================================================
    % DRAW FOUR WHEELS
    %% ========================================================

    wheelNames = [
        "FL";
        "FR";
        "RL";
        "RR"
    ];


    for i = 1:4

        drawWheel( ...
            wheel_W(i,:).', ...
            beta_world(i), ...
            side_world(i), ...
            delta(i), ...
            wheelNames(i), ...
            wheelLength, ...
            wheelWidth ...
        );

    end


    %% ========================================================
    % ROBOT HEADING
    %% ========================================================

    headingLength = 0.50;


    quiver( ...
        yW, ...
        xW, ...
        headingLength*sin(theta), ...
        headingLength*cos(theta), ...
        0, ...
        'k', ...
        'LineWidth',2.5, ...
        'MaxHeadSize',0.8 ...
    );


    %% ========================================================
    % INFORMATION
    %% ========================================================

    infoY = yW + 1.7;


    text( ...
        infoY, ...
        xW + 1.1, ...
        sprintf('v = %.2f m/s',v_cmd) ...
    );


    text( ...
        infoY, ...
        xW + 0.95, ...
        sprintf('\\theta = %.1f deg',rad2deg(theta)) ...
    );


    text( ...
        infoY, ...
        xW + 0.80, ...
        sprintf('\\kappa = %.3f 1/m',kappa) ...
    );


    text( ...
        infoY, ...
        xW + 0.65, ...
        sprintf('\\omega_z = %.3f rad/s',wz_des) ...
    );


    text( ...
        infoY, ...
        xW + 0.45, ...
        sprintf('\\delta_{FL} = %.1f deg',rad2deg(delta(1))) ...
    );


    text( ...
        infoY, ...
        xW + 0.30, ...
        sprintf('\\delta_{FR} = %.1f deg',rad2deg(delta(2))) ...
    );


    text( ...
        infoY, ...
        xW + 0.15, ...
        sprintf('\\delta_{RL} = %.1f deg',rad2deg(delta(3))) ...
    );


    text( ...
        infoY, ...
        xW, ...
        sprintf('\\delta_{RR} = %.1f deg',rad2deg(delta(4))) ...
    );


    %% ========================================================
    % PLOT SETTINGS
    %% ========================================================

    xlabel('y_W [m]   (+ left)');

    ylabel('x_W [m]   (+ forward)');


    title( ...
        sprintf( ...
        'Straight + Strong Curve Following   t = %.1f s', ...
        t ...
        ) ...
    );


    %% Follow robot with camera

    xlim([yW-2.3,yW+2.3]);

    ylim([xW-1.5,xW+2.3]);


    drawnow;

end



%% ============================================================
% LOCAL FUNCTION
% DRAW RECTANGULAR STEERING WHEEL
%% ============================================================

function drawWheel( ...
    center, ...
    beta_world, ...
    side_world, ...
    delta, ...
    name, ...
    wheelLength, ...
    wheelWidth)


    %% Wheel rectangle

    rectangleLocal = [
         wheelLength/2,  wheelWidth/2;
         wheelLength/2, -wheelWidth/2;
        -wheelLength/2, -wheelWidth/2;
        -wheelLength/2,  wheelWidth/2
    ].';


    Rwheel = [
        cos(beta_world), -sin(beta_world);
        sin(beta_world),  cos(beta_world)
    ];


    rectangleWorld = ...
        Rwheel*rectangleLocal + center;


    fill( ...
        rectangleWorld(2,:), ...
        rectangleWorld(1,:), ...
        [0.75 0.75 0.75], ...
        'EdgeColor','k', ...
        'LineWidth',1.5 ...
    );


    %% Side-frame reference direction

    referenceLength = 0.20;


    ref = ...
        referenceLength * ...
        [cos(side_world);sin(side_world)];


    plot( ...
        [center(2),center(2)+ref(2)], ...
        [center(1),center(1)+ref(1)], ...
        'b--', ...
        'LineWidth',1.3 ...
    );


    %% Wheel rolling direction

    arrowLength = 0.28;


    direction = ...
        arrowLength * ...
        [cos(beta_world);sin(beta_world)];


    quiver( ...
        center(2), ...
        center(1), ...
        direction(2), ...
        direction(1), ...
        0, ...
        'r', ...
        'LineWidth',1.7, ...
        'MaxHeadSize',0.8 ...
    );


    %% Steering angle arc

    angleDifference = ...
        atan2( ...
        sin(beta_world-side_world), ...
        cos(beta_world-side_world) ...
        );


    arcAngle = ...
        linspace( ...
        side_world, ...
        side_world + angleDifference, ...
        25 ...
        );


    radius = 0.13;


    arcX = ...
        center(1) ...
        + radius*cos(arcAngle);


    arcY = ...
        center(2) ...
        + radius*sin(arcAngle);


    plot( ...
        arcY, ...
        arcX, ...
        'm-', ...
        'LineWidth',1.2 ...
    );


    %% Wheel label

    text( ...
        center(2)+0.07, ...
        center(1)+0.07, ...
        sprintf( ...
        '%s  %.1f^\\circ', ...
        name, ...
        rad2deg(delta) ...
        ), ...
        'FontSize',8 ...
    );

end