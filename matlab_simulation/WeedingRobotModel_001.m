% %% ================================
% % Guagua Yuan Weeding Robot
% % Basic Kinematic Parameters
% % ================================
% 
% clear;
% clc;
% close all;
% 
% %% Geometry
% 
% L = 0.710;       % front-rear wheel center distance [m]
% W = 1.160;       % left-right wheel center distance [m]
% 
% a = L/2;         % 0.355 m
% b = W/2;         % 0.580 m
% 
% %% Passive joint angle
% phi_L = deg2rad(0);
% phi_R = deg2rad(0);
% 
% %% Passive joint positions
% P_L = [0; b];
% P_R = [0;-b];
% 
% %% Wheel positions
% 
% r_FL = [
%     a*cos(phi_L);
%     b + a*sin(phi_L)
%     ];
% 
% r_RL = [
%     -a*cos(phi_L);
%     b - a*sin(phi_L)
%     ];
% 
% r_FR = [
%     a*cos(phi_R);
%     -b + a*sin(phi_R)
%     ];
% 
% r_RR = [
%     -a*cos(phi_R);
%     -b - a*sin(phi_R)
%     ];
% 
% %% Display
% disp('FL position:');
% disp(r_FL);
% 
% disp('FR position:');
% disp(r_FR);
% 
% disp('RL position:');
% disp(r_RL);
% 
% disp('RR position:');
% disp(r_RR);
% 
% %% Plot robot
% 
% figure;
% hold on;
% grid on;
% axis equal;
% 
% plot(0,0,'ko','MarkerSize',10,'MarkerFaceColor','k');
% 
% plot(P_L(1),P_L(2),'ks','MarkerSize',10);
% plot(P_R(1),P_R(2),'ks','MarkerSize',10);
% 
% plot([r_FL(1),r_RL(1)], ...
%     [r_FL(2),r_RL(2)], ...
%     'LineWidth',2);
% 
% plot([r_FR(1),r_RR(1)], ...
%     [r_FR(2),r_RR(2)], ...
%     'LineWidth',2);
% 
% plot(r_FL(1),r_FL(2),'o','MarkerSize',12);
% plot(r_FR(1),r_FR(2),'o','MarkerSize',12);
% plot(r_RL(1),r_RL(2),'o','MarkerSize',12);
% plot(r_RR(1),r_RR(2),'o','MarkerSize',12);
% 
% text(r_FL(1),r_FL(2),' FL');
% text(r_FR(1),r_FR(2),' FR');
% text(r_RL(1),r_RL(2),' RL');
% text(r_RR(1),r_RR(2),' RR');
% 
% xlabel('x_B [m]');
% ylabel('y_B [m]');
% 
% title('Guagua Yuan Weeding Robot Kinematic Geometry');
% 
% xlim([-1 1]);
% ylim([-1 1]);



%%
%% ============================================================
% Guagua Yuan Sweet Potato Weeding Robot
% Generalized Forward / Inverse Kinematics
%
% Coordinate:
% +x_B : Robot forward
% +y_B : Robot left
% +z_B : Up
%
% Wheel order:
% 1 = FL
% 2 = FR
% 3 = RL
% 4 = RR