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
Amp0n   = [Amp0n0];
Amp0d   = [Amp0d1, Amp0d0];
AmpSat0 = Big;

% =====================[Electrical Motor Dynamics]========================
Elec0n = [1];
Elec1n = [1];


% =====================[Torque Const & Back EMF]========================



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



% =====================[Torque Const & Back EMF]========================



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
