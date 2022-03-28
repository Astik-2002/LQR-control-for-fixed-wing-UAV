%LQR controller for skysurfer with integral action
%longitudinal controller
A = [-0.08 0.5 0 -9.81;
     -0.85 -18.3 21.2 0;
     -0.094 -34.1 -30.2 0;
     0 0 1 0];
B = [2;-444;-823;0];
Q = eye(8);
q = eye(4);
R = 0.2.*eye(1);
%temporary measure, will include kalman filter later
%C = [0 1 0 0;
 %    0 0 0 1];
C = eye(4); 
D = 0;
sys = ss(A,B,C,D);
k1 = lqi(sys,Q,R);
Aaug = [A zeros(4);C zeros(4)];
Baug = [B;zeros(4,1)];
Caug = [C zeros(4)];
Daug = 0;
k = lqr(Aaug,Baug,eye(8),0.2*eye(1));
%Lateral_Directional Controller
%X' = AX+BU; Y = X
% A_lat = [Y_beta   Y_phi    Y_p    Y_r;
%           L_beta   L_phi    L_p    L_r;
%           N_beta   N_phi    N_p    N_r;
%           0     1      0        0];
% B_lat = [Y_a Y_r; 
%           L_a L_r; 
%           N_a N_r;
%           0   0];
A1 = [-0.127 -0.162 -0.0775 1.71;0 0 1 0;29.0 2.71 -8.7 8.55;-20.2 1.46 -2.33 -2.06];
B1 = [1.72 -7.11;0 0;-327 -28.8;-46.7 81.0];
q1 = eye(4);
c1 = [0 1 0 0];
d1 = zeros(1,2);
sys_lat = ss(A1,B1,c1,d1);
Q1 = [1 0 0 0 0; 0 10 0 0 0;0 0 1 0 0;0 0 0 1 0;0 0 0 0 1];
r1 = [1 0;0 1];
k_lat = lqr(A1,B1,q1,r1);
A1aug = [-0.127 -0.162 -0.0775 1.71 0;0 0 1 0 0;29.0 2.71 -8.7 8.55 0;-20.2 1.46 -2.33 -2.06 0;0 0 0 0 0];
B1aug = [1.72 -7.11;0 0;-327 -28.8;-46.7 81.0;0 0];
C1aug = [C zeros(4,1)];
D1aug = zeros(5,2);
k_lati = lqi(sys_lat,Q1,r1);
