% This script sets the model parameters for the SLS 3-D Printer

% Example: Specifying a Dynamics Block
% n = [1 2 3];
% d = [4 5 6];
% Transfer Function = (s^2 + 2s + 3) / (4s^2 + 5s + 6)

% ========================
% PHYSICAL UNIT CONVERSION
% ========================
% Example: if you decide to work in (Kg), all masses must be represented
%          in (Kg) but the spec sheet may provide masses in (g)
% Amplifier circuit elements (units normalized)
R1_ = R1 * 1e6;
R2_ = R2;
C_  = C * 1e-6;
L_  = L * 1e-3;

% Prefix conversion
MILLIS_TO = 1e3;
TO_MILLIS = 1e-3;

% Geometry conversion
link_iR     = LinkR1 * 1e-3;    % (m)
link_oR     = LinkR2 * 1e-3;    % (m)
link_depth  = LinkD * 1e-3;     % (m)
link_offset = LinkOff * 1e-3;   % (m)

% Material and spring constant
alum_density = RhoAl * 1e3;         % (kg/m^3)
spring_k     = SpringK * 1e-3;      % (Nm/rev)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Over-write the default values from DEFAULT.m %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ==========================
% Choose Motors
% ==========================

% Motor Unit Conversions
% ----------------------


% ==========================
% Motor Parameters
% ==========================

% Maximum Current
% ---------------


% =============================
% Q0 : Rotation about y-axis
% =============================


% =====================[Amplifier Dynamics]========================
% Transfer function coefficients
Amp0n0 = (C_ * R2_ * R1_) - L_;
Amp0d0 = C_ * R1_ * R2_;
Amp0d1 = L_ * C_ * R1_;

% Transfer Function Recomputation
% TF= ______Amp0n0______ = ___C_ * R1_ * R2____
%      Amp0d1*s + Amp0d0   L_ * C_ * R1_*s + C_ * R1_ * R2_

Amp0n   = Amp0n0;
Amp0d   = [Amp0d1 Amp0d0];
AmpSat0 = Big;

% =====================[Electrical Motor Dynamics]========================
% This specifies the transfer function for the electric motor
% INPUT: voltage (V)
% OUTPUT: current (A)
% TF= ______Elec0d0______ = ___1_______________
%      Elec0d1*s + Elec0d0    TermL*s + TermR 
Elec0n = 1;

Elec0d0 = MotorParam(TermR);
Elec0d1 = MotorParam(TermL) / MILLIS_TO; %convert mH to H
Elec0d = [Elec0d1 Elec0d0];

% =====================[Torque Const & Back EMF]========================
% TORQUE CONSTANT
% - Between the output of electric dynamics and input of mechanical dynamics
% - Torque = K_T * Current
% - (Nm)   = K_T * (A) 
%
% - K_T has units (Nm/A)
% - `MotorParam(TorqueK)` has units (mNm/A)
% - need to convert to (Nm/A)
TConst0 = MotorParam(TorqueK) * MILLIS_TO;

% SPEED CONSTANT
% - Feedback gain
% - Between the output speed (rad/s) to electric dynamic input (V)
% - Back EMF = K_V * Speed
% - (V)      = K_V * (rad/s)
%
% - K_V has units (V*s/rad)
% - `MotorParam(SpdK)` has units (rpm/V)

% First we convert from (rpm/V) to ((rad/s)/V)
BackEMF0_inv = MotorParam(SpdK) * RadPSecPerRPM;

% Then convert from ((rad/s)/V) to (V/(rad/s))
BackEMF0 = 1 / BackEMF0_inv;

% =====================[Mechanical Motor Dynamics]========================

% =====================[Moment of Inertia Calculations]========================
% Moment of Inertia (J) is contributed by:
% - Wrist frame (aluminium)
% - Rotor
% - Inner motor (q1)
% - Counter weight (aluminium) 

% Moment of inertia due to the wrist frame
ring_volume = (link_oR^2 - link_iR^2) * pi * link_depth;    % Volume (m^3)
ring_mass   = ring_volume * alum_density;                   % Mass   (kg)

% Moment of inertial from the cylindrical shell given by formula:
% J = (m/12)*(3*(r_1^2+r_2^2)+h^2)
% Where:
% - 'J' is moment of inertia    (kgm^2)
% - 'm' is mass                 (kg)
% - 'r_1' is inner radius       (m)
% - 'r_2' is outer radius       (m)
% - 'h' is height / length      (m)
ring_J = (ring_mass / 12) * (3 * (link_iR^2 + link_oR^2) + link_depth^2);

% Moment of inertia from motor's internal rotor
rotor_J = MotorParam(RotJ) * 1e3 * 1e-4;     %(kgm^2)

motor_length  = MotorParam(Length) * 1e-3;       % (m)
motor_radius  = MotorParam(OuterDiam) / 2 * 1e-3; % (m)
motor_weight  = MotorParam(Weight) * 1e-3;       % (kg)
motor_density = motor_weight / (motor_radius^2 * pi * motor_length);    % (kg/m^3)


% The moment of inertia for a rod turning on its end is given as:
% J = (1/3)ml^2
% Where:
% - 'J' is moment of inertia    (kgm^2)
% - 'm' is mass                 (kg)
% - 'l' is the length of rod    (m)
q1_extended_mass = motor_weight + (motor_density * link_offset * motor_radius^2 * pi); % (kg)
q1_extended_J    = q1_extended_mass*(link_offset + motor_length)^2 / 3;    % (kgm^2)
q1_imaginary_J   = (motor_density * link_offset * (motor_radius)^2 * pi) * link_offset^2 / 3;
q1_weight_J      = q1_extended_J - q1_imaginary_J;

% The moment of inertia from the counter weight
counter_weight_J = q1_weight_J;

%now find the value of B which is the same as q1
B_motor_q0 = MotorParam(SpdTorqueGrad);       % rpm/mNm
B_motor_q0 = B_motor_q0 * 1e-3;               % rpm/Nm
B_motor_q0 = B_motor_q0 * RadPSecPerRPM;      % (rad/s)/Nm		
B_motor_q0 = 1 / B_motor_q0;                  % Nm/(rad/s)

% Putting it all together
J_0 = ring_J + rotor_J + q1_weight_J + counter_weight_J;
B_0 = B_motor_q0;
K_0 = 0;

Mech0n  = [1, 0];
Mech0d  = [J_0, B_0, K_0];
JntSat0 = Big;


% =====================[Sensor Dynamics]========================
% TODO: Check work
Sens0    = 0;
SensSat0 = SensV;


% =====================[Static Friction]========================
% TODO: Check work
StFric0 = uSF;


% =============================
% Q1 : Rotation about x-axis (Only carrying the laser)
% =============================

% =====================[Amplifier Dynamics]========================
% Since the same amplifier is used on both motors, the transfer function is the same

Amp1n   = Amp0n;
Amp1d   = Amp0d;
AmpSat1 = Big;

% =====================[Electrical Motor Dynamics]========================
% Since the same electric motor is used on both motors, the transfer function is the same

Elec1n = Elec0n;
Elec1d = Elec0d;

% =====================[Torque Const & Back EMF]========================
% NOTE: Current using identical values from q0 motor
TConst1  = TConst0;
BackEMF1 = BackEMF0;


% =====================[Mechanical Motor Dynamics]========================
% Transfer function is given as:
% TF(S) = ______s______
%         Js^2 + Bs + K

% unit of J is kg*m^2
% unit of B is kg*m^2/s^2
% Get rotor inertia and do unit conversion
% TODO: FiX
J_rotor = MotorParam(RotJ); % g/cm^2
J_rotor = J_rotor * 1e3;    % kg/cm^2
J_rotor = J_rotor * 1e-4;    % kg/m^2

% Motor speed torque gradient and do unit conversion
B_motor = MotorParam(SpdTorqueGrad);    % rpm/mNm
B_motor = B_motor * 1e-3;               % rpm/Nm
B_motor = B_motor * RadPSecPerRPM;      % (rad/s)/Nm
B_motor = 1 / B_motor;                  % Nm/(rad/s)

J_1 = J_rotor;
B_1 = B_motor;
K_1 = 0; %friction is negligible once motor starts moving

Mech1n  = [1, 0];
Mech1d  = [J B K];
JntSat1 = Big;


% =====================[Sensor Dynamics]========================
% TODO:
Sens1    = 0;
SensSat1 = SensV;


% =====================[Static Friction]========================
% TODO:
StFric1 = uSF;

% ==========================================
% Transfer Functions
% ==========================================
% Compute transfer functions from above values and perform system analysis
% You may prefer to put this section in a separate .m file

% Amplifier Transfer Function
tf_amp = tf(Amp0n, Amp0d);
tf_elec = tf(Elec0n, Elec0d);