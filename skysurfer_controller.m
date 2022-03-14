%LQRcontroller design of skysurfer V4 UAV
%parameters obtained fro TU_Delft repository
%Longitudinal controller
% X' = AX+BU; Y = X
% A_long = [X_u   X_w    X_q    X_theta;
%           Z_u   Z_w    Z_q    Z_theta;
%           m_u   m_w    m_q    m_theta;
%           0     0      1        0];
% B_long = [X_e X_t; 
%           Z_e Z_t; 
%           M_e M_t;
%           0   0];
% X = [U;
%      W;
%      Q;
%      Theta];
A = [-0.08 0.5 0 -9.81;
     -0.85 -18.3 21.2 0;
     -0.094 -34.1 -30.2 0;
     0 0 1 0];
B = [2;-444;-823;0];
Q = eye(8);
q = eye(4);
R = 0.2.*eye(1);
C = eye(4);
D = [0;0;0;0];
sys = ss(A,B,C,D);
K_long = lqr(A,B,q,R);
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
r1 = 0.2.*eye(2);
c1 = eye(4);
d1 = 0;
k_lat = lqr(A1,B1,q1,r1);

