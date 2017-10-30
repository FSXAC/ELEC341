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

% Angular/linear uit conversion
RPM_TO_RAD_PER_SEC = 2 * pi / 60;
RAD_PER_SEC_TO_RPM = 1 / RPM_TO_RAD_PER_SEC;


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
Amp0n   = Amp0n0;
Amp0d   = [Amp0d1 Amp0d0];
AmpSat0 = Big;

% =====================[Electrical Motor Dynamics]========================
% This specifies the transfer function for the electric motor
% INPUT: voltage (V)
% OUTPUT: current (A)
Elec0n = 1;

Elec0d0 = MotorParam(TermR);
Elec0d1 = MotorParam(TermL) * MILLIS_TO;
Elec0d = [Elec0d0, Elec0d1];

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
BackEMF0_inv = MotorParam(SpdK) * RPM_TO_RAD_PER_SEC;

% Then convert from ((rad/s)/V) to (V/(rad/s))
BackEMF0 = 1 / BackEMF0_inv;

% =====================[Mechanical Motor Dynamics]========================



% =====================[Sensor Dynamics]========================



% =====================[Static Friction]========================


% =============================
% Q1 : Rotation about x-axis
% =============================

% =====================[Amplifier Dynamics]========================
% Since the same amplifier is used on both motors, the transfer function is the same
Amp1n   = Amp0n;
Amp1d   = Amp0d;
AmpSat1 = Big;

% =====================[Electrical Motor Dynamics]========================
Elec1n = Elec0n;
Elec1d = Elec0d;

% =====================[Torque Const & Back EMF]========================
% NOTE: Current using identical values from q0 motor
TConst1  = TConst0;
BackEMF1 = BackEMF0;


% =====================[Mechanical Motor Dynamics]========================



% =====================[Sensor Dynamics]========================



% =====================[Static Friction]========================




% ==========================================
% Transfer Functions
% ==========================================
% Compute transfer functions from above values and perform system analysis
% You may prefer to put this section in a separate .m file

% Amplifier Transfer Function
tf_amp = tf(Amp0n, Amp0d);
