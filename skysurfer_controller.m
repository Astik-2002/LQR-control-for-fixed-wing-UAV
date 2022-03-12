%LQG controller design of skysurfer V4 UAV
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
%N = rscale(A,B,C,D,K_long); 
%disp(K_long)
%K1 = [-2.3095   -1.0631   -1.9550  -43.9565];
%K2 = [0.2681    2.2180    0.0000    0.0921];
%Lateral_Directional Controller

