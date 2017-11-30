% This script sets the controller parameters for the SLS 3-D Printer

% ==========================================
% System.m analysis
% ==========================================

% =====================[Open loop transfer functions]========================
% The open loop transfer function maps voltage to radians/s
% Motor 0
g_q0 = tf_elec0 * TConst0 * tf_mech0;
h_q0 = BackEMF0;
oltf_q0 = tf_amp0 * (g_q0 / (1 + g_q0 * h_q0)) / tf('s'); 

% Simplified (these are verified to be correct)
ol_q0 = zpk(minreal(oltf_q0));

% Motor 1
g_q1 = tf_elec1 * TConst1 * tf_mech1;
h_q1 = BackEMF1;
oltf_q1 = tf_amp1 * (g_q1 / (1 + g_q1 * h_q1)) / tf('s');

% Simplified (also verified to be correct)
ol_q1 = zpk(minreal(oltf_q1));

% =====================[Finding KU and starting PID values]========================
% === Q0 ===

% Start by placing the zero at where we want the poles to cancel out
zeroes_q0 = [1, 1.95, 96.77];

% PID provides us with one pole at zero and two zeroes we can use
pid_zp_q0 = tf(zeroes_q0, [1, 0]);

% The resulting KGH after adding two zeroes and one pole at zero
kgh_q0 = pid_zp_q0 * ol_q0;

% Find ultimate gain when system becomes unstable
KU_q0 = margin(tf(kgh_q0));

% Find starting gains
Kd_q0       = KU_q0 / 2;
Kp_q0       = Kd_q0 * zeroes_q0(2);
Ki_q0       = Kd_q0 * zeroes_q0(3);
startPID_q0 = [Kp_q0, Ki_q0, Kd_q0];

% The PID transfer function should be
% PID = Kd * (s^2 + (Kp/Kd)s + (Ki/Kd)s)
%       --------------------------------
%                        s
pid_q0 = Kd_q0 * tf([1, (Kp_q0 / Kd_q0), (Ki_q0 / Kd_q0)], [1, 0]);

% Open loop transfer function of q0 with PID
ol_pid_q0 = pid_q0 * ol_q0;

% === Q1 ====
% Same thing
zeroes_q1 = [1, 49.17, 0];
kgh_q1    = ol_q1 * tf(zeroes_q1, [1, 0]);
KU_q1     = margin(minreal(tf(kgh_q1)));
Kd_q1       = KU_q1 / 2;
Kp_q1       = Kd_q1 * zeroes_q1(2);
Ki_q1       = Kd_q1 * zeroes_q1(3);
startPID_q1 = [Kp_q1, Ki_q1, Kd_q1];

% Open loop transfer function of q1 with PID
kgh_q1 = Kd_q1 * kgh_q1;

% =====================[Closed loop transfer functions]========================
% Transfer functions without PID or PID = [1 0 0] (Verified: correct)
cl_q0 = ol_q0 / (1 + ol_q0);
cl_q1 = ol_q1 / (1 + ol_q1);

% Transfer function with starting PID
cl_start_pid_q0 = ol_pid_q0 / (1 + ol_pid_q0);
cl_start_pid_q1 = kgh_q1 / (1 + kgh_q1);

% ================
% CONTROLLER GAINS
% ================

% Enter optimized PID values here.
% No more than 3 significant figures per gain value.
PID0 = [1 0 0];
PID1 = [1 0 0];

% PID0 = startPID_q0;
% PID0 = [0.0871, 4.3210, 0.0447];    % Trial 1 (starting)
% PID0 = [0.0671, 4.3210, 0.0447];    % Trial 2 (decreased overshoot)
% PID0 = [0.0771, 4.5210, 0.0447];    % Trial 3 (decrease risetime)
% PID0 = [0.0791, 4.8210, 0.0447];    % Trial 4 (decrease risetime)
% PID0 = [0.0821, 5.3210, 0.0447];    % Trial 5 (decrease risetime)
% PID0 = [0.0821, 5.3210, 0.0517];    % Trial 6 (decrease overshoot)
% PID0 = [0.0921, 5.3210, 0.0517];    % Trial 7 (decrease risetime)
% PID0 = [0.102, 5.3210, 0.0517];    % Trial 8 (decrease risetime)
% PID0 = [0.102, 5.1210, 0.0537];    % Trial 8 (decrease settletime and overshoot)
% PID0 = [0.122, 5.1210, 0.0537];    % Trial 8 (decrease risetime)
% Not working

% Different approach, K = K at breakaway point - K = Kd = 0.0717
% Kd_q0 = 0.0717;
Kd_q0 = 0.0648;
Kp_q0 = Kd_q0 * zeroes_q0(2);
Ki_q0 = Kd_q0 * zeroes_q0(3);
startPID_q0 = [Kp_q0, Ki_q0, Kd_q0];
PID0 = startPID_q0;
PID0 = [0.1464 6.2707 0.0648];  % -risetime
PID0 = [0.1664 6.2707 0.0648];  % -risetime
PID0 = [0.166 6.47 0.0648];  % -risetime
PID0 = [0.166 6.47 0.0660];  % -overshoot -settletime
PID0 = [0.166 6.47 0.0780];  % -overshoot -settletime
PID0 = [0.166 7.47 0.0780];  % -risetime -overshoot
PID0 = [0.166 7.47 0.0880];  % -overshoot -settletime
PID0 = [0.166 7.47 0.0940];  % -overshoot -settletime
PID0 = [0.166 8.47 0.0940];  % -risetime -overshoot
% PID0 = [0.175 8.47 0.0940];  % -risetime
% PID0 = [0.166 9.47 0.106];  % -overshoot -settletime
% PID0 = [0.166 10.47 0.126];  % -overshoot -settletime
% PID0 = [0.156 10.47 0.126];  % -overshoot -settletime
% PID0 = [0.156 13.47 0.156];  % -overshoot -settletime
% PID0 = [0.145 13.47 0.156];  % -overshoot -settletime
% PID0 = [0.145 16.47 0.156];  % -overshoot -settletime
% PID0 = [0.075 16.47 0.156];  % -overshoot -settletime
% PID0 = [0.075 16.00 0.178];  % -overshoot -settletime

% K at breakaway point: K = Kd = 0.0011
% Kd_q1 = 0.0011;
% Kd_q1 = 0.00863;
Kd_q1 = 0.0028;
Kp_q1 = Kd_q1 * zeroes_q1(2);
Ki_q1 = Kd_q1 * zeroes_q1(3);
startPID_q1 = [Kp_q1, Ki_q1, Kd_q1];
PID1 = startPID_q1;

% Enter feedback sensor values here.
% The feedback gain maps voltage (V) from [-5, 5] to angles (rad) [-pi, pi]
% The gain has the units (rad/V)
FB0 = (SensAng * RadPerDeg) / SensV;    % rad
FB1 = FB0;

% =====================
% Set-Point Time Vector
% =====================

% The Time Vector is stored in a variable called "Time".
% It's initial value is equally spaced for all samples and is
% set in TRAJECTORY.M
%
% Redefine this vector here to optimize the build time of the part.
% You can define it analytically or type in the elements manually but
% you must not change the length because it must contain one value for
% each Xd/Yd position pair.
% In the Matlab window, enter "length(Time)" to see how big it is.

% The Time vector must range from 0 to TotalTime
Time       = 0:SampleTime:TotalTime;       % DO NOT CHANGE TotalTime
% Time = 0:0.125:TotalTime;
% Time = 0:0.25:(0.25 * 160);

% ==========================================
% Final PID transfer functions
% ==========================================
% Transfer function with PID
PID0_tf = PID0(3) * tf([1, (PID0(1) / PID0(3)), (PID0(2) / PID0(3))], [1, 0]);
g_cl_q0 = PID0_tf * ol_q0;
cltf_q0 = g_cl_q0 / (1 + g_cl_q0);
% step(cltf_q0);
% hold on;
% disp(stepinfo(cltf_q0));

% Motor 1
PID1_tf = tf([1, (PID1(1) / PID1(3)), (PID1(2) / PID1(3))], [1, 0]);
g_cl_q1 = PID1_tf * ol_q1;
cltf_q1 = g_cl_q1 / (1 + g_cl_q1);
% step(cltf_q1);
% hold on;
% disp(stepinfo(cltf_q1));

%some useful commands 
%[k,poles] = rlocfind(sys)