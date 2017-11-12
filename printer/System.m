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

%rmp to rad/s conversion

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
% TODO:

% J related to motor inertia, ring inertia, and motor and counterweight inertia
%find J due to the ring..
ring_mass=(LinkR2-LinkR1)*1e-3*LinkD*1e-3 %mass in g
ring_J=mass/12*(3*((LinkR2*1e-3)^2+(LinkR1*1e-3)^2)+LinkD^2)  %ring_J in g/m^2
ring_J=ring_J*1e3 %ring_J in kg/m^2

%find the J of the motor as it rotates. Same as q1
motor_J=MotorParam(RotJ)* 1e3* 1e4 % motor_J in kg/m^2
motor_density= MotorParam(Weight) / ((MotorParam(OuterDiam)/2)^2*pi*MotorParam(Length))

%use superposition to find J due to the the weight of q1 and J due to the counter weight
q1_weight_J = (Weight+motor_density*LinkOff*(MotorParam(OuterDiam)/2)^2*pi)*(LinkOff+Length)^2/3 - motor_density*LinkOff*(MotorParam(OuterDiam)/2)^2*pi*LinkOff^2/3
counter_weight_J= q1_weight_J

%now find the value of B which is the same as q1
B_motor_q0 = MotorParam(SpdTorqueGrad);    % rpm/mNm
B_motor_q0 = B_motor_q0 * 1e-3;               % rpm/Nm
B_motor_q0 = B_motor_q0 * RadPSecPerRPM;      % (rad/s)/Nm
B_motor_q0 = 1 / B_motor_q0;                  % Nm/(rad/s)

%still need to put J's and B's into an array....But need some confirmation first :\

Mech0n  = [1];
Mech0d  = [1];
JntSat0 = Big;


% =====================[Sensor Dynamics]========================
% TODO:
Sens0    = 0;
SensSat0 = Big;


% =====================[Static Friction]========================
% TODO:
StFric0 = 0;




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
J_rotor = MotorParam(RotJ); % g/cm^2
J_rotor = J_rotor * 1e3;    % kg/cm^2
J_rotor = J_rotor * 1e4;    % kg/m^2

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
SensSat1 = Big;


% =====================[Static Friction]========================
% TODO:
StFric1 = 0;



% ==========================================
% Transfer Functions
% ==========================================
% Compute transfer functions from above values and perform system analysis
% You may prefer to put this section in a separate .m file

% Amplifier Transfer Function
tf_amp = tf(Amp0n, Amp0d);
tf_elec = tf(Elec0n, Elec0d);