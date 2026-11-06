% %% ============================================================
% 
% clear;
% clc;
% close all;
% 
% 
% %% ============================================================
% % 1. ROBOT GEOMETRY
% %% ============================================================
% 
% L = 0.710;       % Front-rear wheel center distance [m]
% W = 1.160;       % Left-right wheel center distance [m]
% 
% a = L/2;         % 0.355 m
% b = W/2;         % 0.580 m
% 
% % Effective wheel rolling radius
% % Replace this with actual measured rolling radius later
% Rw = 0.20;       % [m]
% 
% 
% %% ============================================================
% % 2. PASSIVE JOINT CONFIGURATION
% %% ============================================================
% 
% phi_L = deg2rad(0);      % Left passive joint angle
% phi_R = deg2rad(0);      % Right passive joint angle
% 
% 
% %% ============================================================
% % 3. DESIRED GENERALIZED BODY VELOCITY
% %
% % nu =
% % [ vx
% %   vy
% %   wz
% %   phi_L_dot
% %   phi_R_dot ]
% %% ============================================================
% 
% vx_des = 0.00;           % [m/s]
% vy_des = 0.00;           % [m/s]
% wz_des = 0.10;           % [rad/s]
% 
% % Normal driving:
% % passive joints locked
% phi_L_dot_des = 0;
% phi_R_dot_des = 0;
% 
% nu_des = [
%     vx_des;
%     vy_des;
%     wz_des;
%     phi_L_dot_des;
%     phi_R_dot_des
% ];
% 
% 
% %% ============================================================
% % 4. PASSIVE JOINT POSITIONS
% %% ============================================================
% 
% P_L = [0;  b];
% P_R = [0; -b];
% 
% 
% %% ============================================================
% % 5. WHEEL POSITIONS
% %
% % r_i = [x_i; y_i]
% %% ============================================================
% 
% r_FL = [
%      a*cos(phi_L);
%      b + a*sin(phi_L)
% ];
% 
% r_FR = [
%      a*cos(phi_R);
%     -b + a*sin(phi_R)
% ];
% 
% r_RL = [
%     -a*cos(phi_L);
%      b - a*sin(phi_L)
% ];
% 
% r_RR = [
%     -a*cos(phi_R);
%     -b - a*sin(phi_R)
% ];
% 
% 
% % Store as:
% %
% % row = wheel
% % column 1 = x
% % column 2 = y
% 
% r = [
%     r_FL.';
%     r_FR.';
%     r_RL.';
%     r_RR.'
% ];
% 
% 
% %% ============================================================
% % 6. PASSIVE JOINT JACOBIAN TERMS
% %
% % jL(i,:) = velocity contribution due to phi_L_dot
% % jR(i,:) = velocity contribution due to phi_R_dot
% %% ============================================================
% 
% jL = zeros(4,2);
% jR = zeros(4,2);
% 
% 
% % FL
% jL(1,:) = [
%     -a*sin(phi_L), ...
%      a*cos(phi_L)
% ];
% 
% 
% % FR
% jR(2,:) = [
%     -a*sin(phi_R), ...
%      a*cos(phi_R)
% ];
% 
% 
% % RL
% jL(3,:) = [
%      a*sin(phi_L), ...
%     -a*cos(phi_L)
% ];
% 
% 
% % RR
% jR(4,:) = [
%      a*sin(phi_R), ...
%     -a*cos(phi_R)
% ];
% 
% 
% %% ============================================================
% % 7. BUILD GENERALIZED KINEMATIC MATRIX G
% %
% % For each wheel:
% %
% % [vix]   [1  0  -yi  jLx  jRx] [vx        ]
% % [viy] = [0  1   xi  jLy  jRy] [vy        ]
% %                                  [wz        ]
% %                                  [phiL_dot  ]
% %                                  [phiR_dot  ]
% %% ============================================================
% 
% G = zeros(8,5);
% 
% for i = 1:4
% 
%     xi = r(i,1);
%     yi = r(i,2);
% 
%     jLx = jL(i,1);
%     jLy = jL(i,2);
% 
%     jRx = jR(i,1);
%     jRy = jR(i,2);
% 
% 
%     G(2*i-1,:) = [
%         1, ...
%         0, ...
%        -yi, ...
%         jLx, ...
%         jRx
%     ];
% 
% 
%     G(2*i,:) = [
%         0, ...
%         1, ...
%         xi, ...
%         jLy, ...
%         jRy
%     ];
% 
% end
% 
% 
% %% ============================================================
% % 8. INVERSE KINEMATICS
% %
% % Desired robot motion
% %        ↓
% % wheel center velocity
% %% ============================================================
% 
% V_wheel_vector = G * nu_des;
% 
% 
% % Convert:
% %
% % [vx_FL
% %  vy_FL
% %  vx_FR
% %  vy_FR
% %  ...]
% %
% % into:
% %
% % [vx_FL vy_FL
% %  vx_FR vy_FR
% %  vx_RL vy_RL
% %  vx_RR vy_RR]
% 
% wheel_velocity = reshape( ...
%     V_wheel_vector, ...
%     2, ...
%     4 ...
%     ).';
% 
% 
% %% ============================================================
% % 9. CALCULATE ABSOLUTE WHEEL DIRECTIONS beta
% %% ============================================================
% 
% beta = zeros(4,1);
% 
% wheel_linear_speed = zeros(4,1);
% 
% wheel_omega = zeros(4,1);
% 
% 
% for i = 1:4
% 
%     vix = wheel_velocity(i,1);
%     viy = wheel_velocity(i,2);
% 
%     % Absolute wheel rolling direction
%     beta(i) = atan2(viy, vix);
% 
%     % Wheel linear speed
%     wheel_linear_speed(i) = hypot(vix, viy);
% 
%     % Hub motor angular speed
%     wheel_omega(i) = ...
%         wheel_linear_speed(i) / Rw;
% 
% end
% 
% 
% %% ============================================================
% % 10. CONVERT beta TO LOCAL STEERING ANGLE delta
% %
% % beta = phi + delta
% %
% % therefore:
% %
% % delta = beta - phi
% %% ============================================================
% 
% phi_side = [
%     phi_L;
%     phi_R;
%     phi_L;
%     phi_R
% ];
% 
% 
% delta = beta - phi_side;
% 
% 
% % Wrap steering angles to [-pi, pi]
% 
% delta = atan2( ...
%     sin(delta), ...
%     cos(delta) ...
% );
% 
% 
% %% ============================================================
% % 11. FORWARD KINEMATICS
% %
% % Pretend beta / delta and wheel_omega
% % are measurements from real sensors.
% %% ============================================================
% 
% % Recover absolute wheel direction
% % from:
% %
% % beta = phi + delta
% 
% beta_measured = phi_side + delta;
% 
% 
% % Wrap to [-pi, pi]
% 
% beta_measured = atan2( ...
%     sin(beta_measured), ...
%     cos(beta_measured) ...
% );
% 
% 
% %% ============================================================
% % 12. RECONSTRUCT WHEEL VELOCITY FROM ENCODERS
% %
% % vi =
% %
% % Rw * Omega_i *
% %
% % [ cos(beta_i)
% %   sin(beta_i) ]
% %% ============================================================
% 
% V_measured = zeros(8,1);
% 
% 
% for i = 1:4
% 
%     wheel_vx = ...
%         Rw * wheel_omega(i) ...
%         * cos(beta_measured(i));
% 
%     wheel_vy = ...
%         Rw * wheel_omega(i) ...
%         * sin(beta_measured(i));
% 
% 
%     V_measured(2*i-1) = wheel_vx;
% 
%     V_measured(2*i)   = wheel_vy;
% 
% end
% 
% 
% %% ============================================================
% % 13. GENERALIZED FORWARD KINEMATICS
% %
% % V = G * nu
% %
% % therefore:
% %
% % nu = pinv(G) * V
% %% ============================================================
% 
% nu_est = pinv(G) * V_measured;
% 
% 
% vx_est       = nu_est(1);
% vy_est       = nu_est(2);
% wz_est       = nu_est(3);
% phiLdot_est  = nu_est(4);
% phiRdot_est  = nu_est(5);
% 
% 
% %% ============================================================
% % 14. DISPLAY RESULTS
% %% ============================================================
% 
% wheel_name = [
%     "FL";
%     "FR";
%     "RL";
%     "RR"
% ];
% 
% 
% wheel_rpm = ...
%     wheel_omega * 60/(2*pi);
% 
% 
% ResultTable = table( ...
%     wheel_name, ...
%     r(:,1), ...
%     r(:,2), ...
%     rad2deg(beta), ...
%     rad2deg(delta), ...
%     wheel_linear_speed, ...
%     wheel_omega, ...
%     wheel_rpm, ...
%     'VariableNames', ...
%     { ...
%     'Wheel', ...
%     'x_m', ...
%     'y_m', ...
%     'Beta_deg', ...
%     'Delta_deg', ...
%     'LinearSpeed_mps', ...
%     'Omega_radps', ...
%     'RPM' ...
%     } ...
% );
% 
% 
% disp(' ');
% disp('==============================================');
% disp('INVERSE KINEMATICS RESULTS');
% disp('==============================================');
% 
% disp(ResultTable);
% 
% 
% disp(' ');
% disp('==============================================');
% disp('DESIRED GENERALIZED VELOCITY');
% disp('==============================================');
% 
% fprintf('vx        = %.6f m/s\n', vx_des);
% fprintf('vy        = %.6f m/s\n', vy_des);
% fprintf('wz        = %.6f rad/s\n', wz_des);
% fprintf('phiL_dot  = %.6f rad/s\n', phi_L_dot_des);
% fprintf('phiR_dot  = %.6f rad/s\n', phi_R_dot_des);
% 
% 
% disp(' ');
% disp('==============================================');
% disp('FORWARD KINEMATICS ESTIMATION');
% disp('==============================================');
% 
% fprintf('vx        = %.6f m/s\n', vx_est);
% fprintf('vy        = %.6f m/s\n', vy_est);
% fprintf('wz        = %.6f rad/s\n', wz_est);
% fprintf('phiL_dot  = %.6f rad/s\n', phiLdot_est);
% fprintf('phiR_dot  = %.6f rad/s\n', phiRdot_est);
% 
% 
% disp(' ');
% disp('==============================================');
% disp('KINEMATIC MATRIX RANK');
% disp('==============================================');
% 
% fprintf('rank(G) = %d\n', rank(G));
% 
% 
% %% ============================================================
% % 15. PLOT ROBOT
% %
% % Important:
% %
% % Mathematically:
% % +x_B = forward
% % +y_B = left
% %
% % For visualization:
% % vertical axis   = x_B
% % horizontal axis = y_B
% %% ============================================================
% 
% figure;
% 
% hold on;
% grid on;
% axis equal;
% 
% 
% % Make +y_B appear to the left
% set(gca,'XDir','reverse');
% 
% 
% %% Robot center
% 
% plot( ...
%     0, ...
%     0, ...
%     'ko', ...
%     'MarkerSize',10, ...
%     'MarkerFaceColor','k' ...
% );
% 
% 
% %% Passive joints
% 
% plot( ...
%     P_L(2), ...
%     P_L(1), ...
%     'ks', ...
%     'MarkerSize',10 ...
% );
% 
% plot( ...
%     P_R(2), ...
%     P_R(1), ...
%     'ks', ...
%     'MarkerSize',10 ...
% );
% 
% 
% %% Central body link
% 
% plot( ...
%     [P_L(2), P_R(2)], ...
%     [P_L(1), P_R(1)], ...
%     'LineWidth',2 ...
% );
% 
% 
% %% Left side module
% 
% plot( ...
%     [r_FL(2), r_RL(2)], ...
%     [r_FL(1), r_RL(1)], ...
%     'LineWidth',2 ...
% );
% 
% 
% %% Right side module
% 
% plot( ...
%     [r_FR(2), r_RR(2)], ...
%     [r_FR(1), r_RR(1)], ...
%     'LineWidth',2 ...
% );
% 
% 
% %% Wheel centers
% 
% for i = 1:4
% 
%     plot( ...
%         r(i,2), ...
%         r(i,1), ...
%         'o', ...
%         'MarkerSize',12, ...
%         'LineWidth',2 ...
%     );
% 
% end
% 
% 
% %% Wheel direction arrows
% 
% arrow_scale = 0.25;
% 
% for i = 1:4
% 
%     quiver( ...
%         r(i,2), ...
%         r(i,1), ...
%         arrow_scale*sin(beta(i)), ...
%         arrow_scale*cos(beta(i)), ...
%         0, ...
%         'LineWidth',2, ...
%         'MaxHeadSize',0.8 ...
%     );
% 
% end
% 
% 
% %% Labels
% 
% text( ...
%     r_FL(2), ...
%     r_FL(1), ...
%     '  FL' ...
% );
% 
% text( ...
%     r_FR(2), ...
%     r_FR(1), ...
%     '  FR' ...
% );
% 
% text( ...
%     r_RL(2), ...
%     r_RL(1), ...
%     '  RL' ...
% );
% 
% text( ...
%     r_RR(2), ...
%     r_RR(1), ...
%     '  RR' ...
% );
% 
% 
% xlabel('y_B [m]   (+ left)');
% 
% ylabel('x_B [m]   (+ forward)');
% 
% 
% title( ...
%     'Guagua Yuan Weeding Robot - Kinematic Model' ...
% );
% 
% 
% xlim([-1 1]);
% 
% ylim([-0.8 0.8]);



%%========================================================================



%% ============================================================

clear;
clc;
close all;


%% ============================================================
% 1. ROBOT GEOMETRY
%% ============================================================

L = 0.710;       % Front-rear wheel center distance [m]
W = 1.160;       % Left-right wheel center distance [m]

a = L/2;         % 0.355 m
b = W/2;         % 0.580 m

% Effective wheel rolling radius
% Replace this with actual measured rolling radius later
Rw = 0.20;       % [m]


%% ============================================================
% 2. PASSIVE JOINT CONFIGURATION
%% ============================================================

phi_L = deg2rad(0);      % Left passive joint angle
phi_R = deg2rad(0);      % Right passive joint angle


%% ============================================================
% 3. DESIRED GENERALIZED BODY VELOCITY
%
% nu =
% [ vx
%   vy
%   wz
%   phi_L_dot
%   phi_R_dot ]
%% ============================================================

vx_des = 0.50;           % [m/s]
vy_des = 0.50;           % [m/s]
wz_des = 0.00;           % [rad/s]

% Normal driving:
% passive joints locked
phi_L_dot_des = 0;
phi_R_dot_des = 0;

nu_des = [
    vx_des;
    vy_des;
    wz_des;
    phi_L_dot_des;
    phi_R_dot_des
];


%% ============================================================
% 4. PASSIVE JOINT POSITIONS
%% ============================================================

P_L = [0;  b];
P_R = [0; -b];


%% ============================================================
% 5. WHEEL POSITIONS
%
% r_i = [x_i; y_i]
%% ============================================================

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


% Store as:
%
% row = wheel
% column 1 = x
% column 2 = y

r = [
    r_FL.';
    r_FR.';
    r_RL.';
    r_RR.'
];


%% ============================================================
% 6. PASSIVE JOINT JACOBIAN TERMS
%
% jL(i,:) = velocity contribution due to phi_L_dot
% jR(i,:) = velocity contribution due to phi_R_dot
%% ============================================================

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


%% ============================================================
% 7. BUILD GENERALIZED KINEMATIC MATRIX G
%
% For each wheel:
%
% [vix]   [1  0  -yi  jLx  jRx] [vx        ]
% [viy] = [0  1   xi  jLy  jRy] [vy        ]
%                                  [wz        ]
%                                  [phiL_dot  ]
%                                  [phiR_dot  ]
%% ============================================================

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


%% ============================================================
% 8. INVERSE KINEMATICS
%
% Desired robot motion
%        ↓
% wheel center velocity
%% ============================================================

V_wheel_vector = G * nu_des;


% Convert:
%
% [vx_FL
%  vy_FL
%  vx_FR
%  vy_FR
%  ...]
%
% into:
%
% [vx_FL vy_FL
%  vx_FR vy_FR
%  vx_RL vy_RL
%  vx_RR vy_RR]

wheel_velocity = reshape( ...
    V_wheel_vector, ...
    2, ...
    4 ...
    ).';


%% ============================================================
% 9. CALCULATE ABSOLUTE WHEEL DIRECTIONS beta
%% ============================================================

beta = zeros(4,1);

wheel_linear_speed = zeros(4,1);

wheel_omega = zeros(4,1);


for i = 1:4

    vix = wheel_velocity(i,1);
    viy = wheel_velocity(i,2);

    % Absolute wheel rolling direction
    beta(i) = atan2(viy, vix);

    % Wheel linear speed
    wheel_linear_speed(i) = hypot(vix, viy);

    % Hub motor angular speed
    wheel_omega(i) = ...
        wheel_linear_speed(i) / Rw;

end


%% ============================================================
% 10. CONVERT beta TO LOCAL STEERING ANGLE delta
%
% beta = phi + delta
%
% therefore:
%
% delta = beta - phi
%% ============================================================

phi_side = [
    phi_L;
    phi_R;
    phi_L;
    phi_R
];


delta = beta - phi_side;


% Wrap steering angles to [-pi, pi]

delta = atan2( ...
    sin(delta), ...
    cos(delta) ...
);


%% ============================================================
% 11. FORWARD KINEMATICS
%
% Pretend beta / delta and wheel_omega
% are measurements from real sensors.
%% ============================================================

% Recover absolute wheel direction
% from:
%
% beta = phi + delta

beta_measured = phi_side + delta;


% Wrap to [-pi, pi]

beta_measured = atan2( ...
    sin(beta_measured), ...
    cos(beta_measured) ...
);


%% ============================================================
% 12. RECONSTRUCT WHEEL VELOCITY FROM ENCODERS
%
% vi =
%
% Rw * Omega_i *
%
% [ cos(beta_i)
%   sin(beta_i) ]
%% ============================================================

V_measured = zeros(8,1);


for i = 1:4

    wheel_vx = ...
        Rw * wheel_omega(i) ...
        * cos(beta_measured(i));

    wheel_vy = ...
        Rw * wheel_omega(i) ...
        * sin(beta_measured(i));


    V_measured(2*i-1) = wheel_vx;

    V_measured(2*i)   = wheel_vy;

end


%% ============================================================
% 13. GENERALIZED FORWARD KINEMATICS
%
% V = G * nu
%
% therefore:
%
% nu = pinv(G) * V
%% ============================================================

nu_est = pinv(G) * V_measured;


vx_est       = nu_est(1);
vy_est       = nu_est(2);
wz_est       = nu_est(3);
phiLdot_est  = nu_est(4);
phiRdot_est  = nu_est(5);


%% ============================================================
% 14. DISPLAY RESULTS
%% ============================================================

wheel_name = [
    "FL";
    "FR";
    "RL";
    "RR"
];


wheel_rpm = ...
    wheel_omega * 60/(2*pi);


ResultTable = table( ...
    wheel_name, ...
    r(:,1), ...
    r(:,2), ...
    rad2deg(beta), ...
    rad2deg(delta), ...
    wheel_linear_speed, ...
    wheel_omega, ...
    wheel_rpm, ...
    'VariableNames', ...
    { ...
    'Wheel', ...
    'x_m', ...
    'y_m', ...
    'Beta_deg', ...
    'Delta_deg', ...
    'LinearSpeed_mps', ...
    'Omega_radps', ...
    'RPM' ...
    } ...
);


disp(' ');
disp('==============================================');
disp('INVERSE KINEMATICS RESULTS');
disp('==============================================');

disp(ResultTable);


disp(' ');
disp('==============================================');
disp('DESIRED GENERALIZED VELOCITY');
disp('==============================================');

fprintf('vx        = %.6f m/s\n', vx_des);
fprintf('vy        = %.6f m/s\n', vy_des);
fprintf('wz        = %.6f rad/s\n', wz_des);
fprintf('phiL_dot  = %.6f rad/s\n', phi_L_dot_des);
fprintf('phiR_dot  = %.6f rad/s\n', phi_R_dot_des);


disp(' ');
disp('==============================================');
disp('FORWARD KINEMATICS ESTIMATION');
disp('==============================================');

fprintf('vx        = %.6f m/s\n', vx_est);
fprintf('vy        = %.6f m/s\n', vy_est);
fprintf('wz        = %.6f rad/s\n', wz_est);
fprintf('phiL_dot  = %.6f rad/s\n', phiLdot_est);
fprintf('phiR_dot  = %.6f rad/s\n', phiRdot_est);


disp(' ');
disp('==============================================');
disp('KINEMATIC MATRIX RANK');
disp('==============================================');

fprintf('rank(G) = %d\n', rank(G));

%% ============================================================
% 15. PLOT ROBOT
%
% IMPORTANT:
%
% Kinematic model is NOT changed.
%
% Coordinate definition:
%
% +x_B = robot forward
% +y_B = robot left
%
% Visualization:
%
% vertical axis   = x_B
% horizontal axis = y_B
%% ============================================================

figure( ...
    'Color','w', ...
    'Position',[200 100 850 750], ...
    'Name','Guagua Yuan Weeding Robot Kinematic Model');


hold on;
grid on;
axis equal;


%% ============================================================
% VISUAL SETTINGS
%% ============================================================

% Make +y_B appear toward the LEFT
set(gca,'XDir','reverse');


%% ------------------------------------------------------------
% Wheel visualization parameters
%% ------------------------------------------------------------

% Rectangle length:
% along wheel rolling direction
wheelLength = 0.28;          % [m]

% Rectangle width:
% physical tire width
wheelWidth = 0.10;           % [m]

% Wheel direction arrow length
wheelArrowLength = 0.20;     % [m]

% Arrow line width
wheelArrowLineWidth = 2.2;


%% ------------------------------------------------------------
% Wheel colors
%
% FL = red
% FR = blue
% RL = green
% RR = purple
%% ------------------------------------------------------------

wheelColor = [

    0.85, 0.25, 0.25;       % FL

    0.20, 0.45, 0.85;       % FR

    0.20, 0.65, 0.30;       % RL

    0.65, 0.30, 0.75        % RR

];


wheelNames = {

    'FL';

    'FR';

    'RL';

    'RR'

};


%% ============================================================
% ROBOT CENTER
%% ============================================================

plot( ...
    0, ...
    0, ...
    'ko', ...
    'MarkerSize',10, ...
    'MarkerFaceColor','k');


%% ============================================================
% PASSIVE JOINTS
%% ============================================================

plot( ...
    P_L(2), ...
    P_L(1), ...
    'ks', ...
    'MarkerSize',10, ...
    'LineWidth',1.5, ...
    'MarkerFaceColor','w');


plot( ...
    P_R(2), ...
    P_R(1), ...
    'ks', ...
    'MarkerSize',10, ...
    'LineWidth',1.5, ...
    'MarkerFaceColor','w');


%% ============================================================
% CENTRAL BODY LINK
%% ============================================================

plot( ...
    [P_L(2), P_R(2)], ...
    [P_L(1), P_R(1)], ...
    '-', ...
    'Color',[0.15 0.15 0.15], ...
    'LineWidth',3);


%% ============================================================
% LEFT SIDE MODULE
%% ============================================================

plot( ...
    [r_FL(2), r_RL(2)], ...
    [r_FL(1), r_RL(1)], ...
    '-', ...
    'Color',[0.25 0.25 0.25], ...
    'LineWidth',3);


%% ============================================================
% RIGHT SIDE MODULE
%% ============================================================

plot( ...
    [r_FR(2), r_RR(2)], ...
    [r_FR(1), r_RR(1)], ...
    '-', ...
    'Color',[0.25 0.25 0.25], ...
    'LineWidth',3);


%% ============================================================
% RECTANGULAR WHEELS + DIRECTION ARROWS
%
% IMPORTANT:
%
% No kinematic calculations are changed here.
%
% r(i,:)  = wheel center position calculated previously
% beta(i) = absolute wheel rolling direction calculated
%           previously by inverse kinematics
%
% Rectangle orientation = beta(i)
% Arrow direction       = beta(i)
%% ============================================================

for i = 1:4


    %% --------------------------------------------------------
    % Wheel center
    %
    % Mathematical body coordinates:
    %
    % [x_B
    %  y_B]
    %% --------------------------------------------------------

    wheelCenter = [

        r(i,1);

        r(i,2)

    ];


    %% --------------------------------------------------------
    % Rectangle in wheel-local coordinate
    %
    % Local x-axis:
    % wheel rolling direction
    %
    % Local y-axis:
    % wheel width direction
    %% --------------------------------------------------------

    wheelLocal = [

         wheelLength/2, ...
         wheelLength/2, ...
        -wheelLength/2, ...
        -wheelLength/2;

         wheelWidth/2, ...
        -wheelWidth/2, ...
        -wheelWidth/2, ...
         wheelWidth/2

    ];


    %% --------------------------------------------------------
    % Rotation matrix
    %
    % Rotate rectangular wheel according to beta(i)
    %% --------------------------------------------------------

    Rwheel = [

        cos(beta(i)), -sin(beta(i));

        sin(beta(i)),  cos(beta(i))

    ];


    %% --------------------------------------------------------
    % Convert rectangle from wheel frame
    % to robot body frame
    %% --------------------------------------------------------

    wheelCorners = ...
        Rwheel * wheelLocal ...
        + wheelCenter;


    %% --------------------------------------------------------
    % Draw rectangular wheel
    %
    % IMPORTANT:
    %
    % MATLAB plot:
    %
    % horizontal axis = y_B
    % vertical axis   = x_B
    %
    % Therefore:
    %
    % XData = wheelCorners(2,:)
    % YData = wheelCorners(1,:)
    %% --------------------------------------------------------

    patch( ...
        wheelCorners(2,:), ...
        wheelCorners(1,:), ...
        wheelColor(i,:), ...
        'FaceAlpha',0.55, ...
        'EdgeColor',wheelColor(i,:), ...
        'LineWidth',2.2);


    %% --------------------------------------------------------
    % Wheel center marker
    %% --------------------------------------------------------

    plot( ...
        r(i,2), ...
        r(i,1), ...
        'ko', ...
        'MarkerSize',4, ...
        'MarkerFaceColor','k');


    %% --------------------------------------------------------
    % Wheel direction arrow
    %
    % Wheel direction vector:
    %
    % [ cos(beta)
    %   sin(beta) ]
    %
    % But plot axes are:
    %
    % horizontal = y_B
    % vertical   = x_B
    %
    % Therefore:
    %
    % U = sin(beta)
    % V = cos(beta)
    %% --------------------------------------------------------

    arrowU = ...
        wheelArrowLength ...
        * sin(beta(i));


    arrowV = ...
        wheelArrowLength ...
        * cos(beta(i));


    quiver( ...
        r(i,2), ...
        r(i,1), ...
        arrowU, ...
        arrowV, ...
        0, ...
        'Color','k', ...
        'LineWidth',wheelArrowLineWidth, ...
        'MaxHeadSize',0.8);


    %% --------------------------------------------------------
    % Wheel label
    %% --------------------------------------------------------

    text( ...
        r(i,2) + 0.09, ...
        r(i,1) + 0.03, ...
        sprintf( ...
        '%s\n\\beta = %.1f°\n\\delta = %.1f°', ...
        wheelNames{i}, ...
        rad2deg(beta(i)), ...
        rad2deg(delta(i))), ...
        'FontSize',9, ...
        'FontWeight','bold', ...
        'Color',wheelColor(i,:));

end


%% ============================================================
% BODY-FRAME FORWARD DIRECTION
%
% This is only a coordinate reference.
% It does NOT change robot kinematics.
%% ============================================================

bodyArrowLength = 0.32;


quiver( ...
    0, ...
    0, ...
    0, ...
    bodyArrowLength, ...
    0, ...
    'Color',[0.15 0.15 0.15], ...
    'LineWidth',1.6, ...
    'MaxHeadSize',0.7);


text( ...
    -0.03, ...
    bodyArrowLength + 0.04, ...
    '+x_B', ...
    'FontSize',9, ...
    'FontWeight','bold');


%% ============================================================
% AXIS LABELS
%% ============================================================

xlabel( ...
    'y_B [m]   (+ left)', ...
    'FontSize',11);


ylabel( ...
    'x_B [m]   (+ forward)', ...
    'FontSize',11);


%% ============================================================
% TITLE
%% ============================================================

title( ...
    'Guagua Yuan Weeding Robot - Kinematic Model', ...
    'FontSize',13, ...
    'FontWeight','bold');


%% ============================================================
% DISPLAY RANGE
%% ============================================================

xlim([-1 1]);

ylim([-0.8 0.8]);


set(gca, ...
    'FontSize',10, ...
    'LineWidth',1.0);


box on;