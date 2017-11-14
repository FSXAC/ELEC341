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
alum_density = RhoAl * 1e3;                     % (kg/m^3)
spring_k     = SpringK * 1e-3 / (2 * pi);       % (Nm/rad)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Over-write the default values from DEFAULT.m %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ==========================
% Choose Motors
% ==========================


% =====================[Motor Parameter Unit Conversion]========================
motor_param = ...
[
    MotorParam(NomV)                        % (V)
    MotorParam(NoLoadSpd) * RadPSecPerRPM   % (rad/s)
    MotorParam(NoLoadCurr) * 1e-3           % (A)
    MotorParam(NomSpd) * RadPSecPerRPM      % (rad/s)
    MotorParam(NomTorque) * 1e-3            % (Nm)
    MotorParam(NomCurr) * 1e-3              % (A)
    MotorParam(StallTorque) * 1e-3          % (Nm)
    MotorParam(StallCurr)                   % (A)
    MotorParam(MaxEff)                      % (%)

    MotorParam(TermR)                       % (Ohms)
    MotorParam(TermL) * 1e-3                % (H)
    MotorParam(TorqueK) * 1e-3              % (Nm/A)
    MotorParam(SpdK) * RadPSecPerRPM        % ((rad/s)/V)
    MotorParam(SpdTorqueGrad) * 1e3 * RadPSecPerRPM     % ((rad/s)/Nm)
    MotorParam(MechTimeK) * 1e-3            % (s)
    MotorParam(RotJ) * 1e-3 * 1e-4          % (kgm^2)

    MotorParam(ThermRhous)                  % (K/W)
    MotorParam(ThermRwind)                  % (K/W)
    MotorParam(ThermTCwind)                 % (s)
    MotorParam(ThermTCmot)                  % (s)
    MotorParam(AmbTemp)                     % (degC)
    MotorParam(MaxTemp)                     % (degC)

    MotorParam(MaxSpdBall) * RadPSecPerRPM  % (rad/s)
    MotorParam(AxialPlayBall) * 1e-3        % (m)
    MotorParam(RadPlayBall) * 1e-3          % (m)
    MotorParam(MaxAxLdBall)                 % (N)
    MotorParam(MaxFBall)                    % (N)
    MotorParam(MaxRadLdBall)                % (N)

    MotorParam(NoPolePair)                  % ()
    MotorParam(NoCommSeg)                   % ()
    MotorParam(Weight) * 1e-3               % (kg)

    MotorParam(OuterDiam) * 1e-3            % (m)
    MotorParam(Length) * 1e-3               % (m)
];


% ==========================
% Motor Parameters
% ==========================
% Maximum Current
% ---------------



% =============================
% Q0 : Rotation about y-axis
% =============================
% =====================[Amplifier Dynamics]========================
% TODO: Check calculations
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

Elec0d0 = motor_param(TermR);                % (Ohms)
Elec0d1 = motor_param(TermL) * 1e3;          % (H)
Elec0n  = 1;
Elec0d  = [Elec0d1, Elec0d0];

% =====================[Torque Const & Back EMF]========================
% TORQUE CONSTANT
% The gain between the output of electric dynamics and input of mechanical dynamics
% Equation: Torque = K_T    * Current
% Units:    (Nm)   = (Nm/A) * (A)
TConst0 = motor_param(TorqueK);     % (Nm/A)

% SPEED CONSTANT
% The gain between the output speed and induced back-EMF
% Equation: Back EMF = K_V         * Speed
% Units:    (V)      = (V/(rad/s)) * (rad/s)
BackEMF0 = 1 / motor_param(SpdK);   % (V/(rad/s))

% =====================[Mechanical Motor Dynamics]========================
% Transfer function is given as:
% TF(S) = ______s______
%         Js^2 + Bs + K
% The unit of J is kgm^2
% The unit of B is kgm^2/s
% The unit of K is kgm^2/s^2

% ======== Moment of Inertia Calculations ========
% Moment of Inertia (J) is contributed by:
% - Wrist frame (aluminium)
% - Rotor
% - Inner motor (q1)
% - Counter weight (aluminium) 

% === Moment of inertia due to the wrist frame ===
% Compute the mass of the 6061 cylindrical shell / ring
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

% === Moment of inertia from motor's internal rotor ===
q0_rotor_J = motor_param(RotJ); % (kgm^2)

% === Moment of inertia from the inner motor (q1) and the counter weight ===
% Find the density of the motor by treating it as a cylindrical rod
motor_length  = motor_param(Length);                % (m)
motor_radius  = motor_param(OuterDiam) / 2;         % (m)
motor_volume  = motor_radius^2 * pi * motor_length; % (m^3)
motor_mass    = motor_param(Weight);                % (kg)
motor_density = motor_mass / motor_volume;          % (kg/m^3)

% Find the mass of the motor if it were a cylinder that extends to the center
q1_extended_volume = link_offset * motor_radius^2 * pi;                 % (m^3)
q1_extended_mass   = motor_mass + (motor_density * q1_extended_volume); % (kg)

% Find the moment due to the motor and the imaginary extension:
% The moment of inertia for a rod turning on its end is given as:
% J = (1/3)ml^2
% Where:
% - 'J' is moment of inertia    (kgm^2)
% - 'm' is mass                 (kg)
% - 'l' is the length of rod    (m)
% Computing the J from the motor and the imaginary extension
q1_extended_length = link_offset + motor_length;                        % (m)
q1_extended_J      = q1_extended_mass * q1_extended_length^2 / 3;       % (kgm^2)

% Compute the J from just the extension
q1_imaginary_mass = motor_density * link_offset * motor_radius^2 * pi;  % (kg)
q1_imaginary_J    = q1_imaginary_mass * link_offset^2 / 3;              % (kgm^2)

% The moment of inertia by the motor is a superposition
q1_J = q1_extended_J - q1_imaginary_J;                                  % (kgm^2)

% The moment of inertia from the counter weight is the same as motor
counter_J = q1_J;                                                       % (kgm^2)

% ========= End of Moment of Inertia Calculations ========

% Finding 'B' via speed-torque gradient
q0_B = 1 / motor_param(SpdTorqueGrad);          % (Nm/(rad/s))

% Spring behavior associated with motor q0
q0_K = spring_k;                                % (Nm/rad)

% Putting it all together
J_0 = ring_J + q0_rotor_J + q1_J + counter_J;   % (kgm^2)
B_0 = q0_B;                                     % ((rad/s)/Nm)  === (kgm^2/s)
K_0 = q0_K;                                     % (Nm/rad)      === (kgm^2/s^2)

% Insert coefficients into transfer function
Mech0n  = [1, 0];
Mech0d  = [J_0, B_0, K_0];

% Motor q0 has unlimited joint limit
JntSat0 = Big;

% =====================[Sensor Dynamics]========================
% TODO: Check work
% Sensor gain maps angle (in radians) to voltages (V)
Sens0    = 0;
SensSat0 = SensV;

% =====================[Static Friction]========================
% TODO: Check work (not quite)
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
% The unit of J is kgm^2
% The unit of B is kgm^2/s
% The unit of K is kgm^2/s^2

% Get rotor inertia and do unit conversion
J_rotor = MotorParam(RotJ); % gcm^2
J_rotor = J_rotor * 1e3;    % kgcm^2
J_rotor = J_rotor * 1e-4;   % kgm^2

% Motor speed torque gradient and do unit conversion
B_motor = MotorParam(SpdTorqueGrad);    % rpm/mNm
B_motor = B_motor * 1e3;                % rpm/Nm
B_motor = B_motor * RadPSecPerRPM;      % (rad/s)/Nm
B_motor = 1 / B_motor;                  % Nm/(rad/s) === kgm^2/s

% Moment of inertia only depends on the rotor
J_1 = J_rotor;

% Friction
B_1 = B_motor;

% Inner motor has no spring behavior
K_1 = 0;

% Q1 mechanical dynamics transfer function
Mech1n  = [1, 0];
Mech1d  = [J_1, B_1, K_1];

% Maximum the joint can turn
JntSat1 = JntLim * RadPerDeg;           % (rad)


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