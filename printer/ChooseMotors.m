AMAX12_p75W_SB;
motor_1 = MotorParam;
AMAX16_2W_SB;
motor_2 = MotorParam;
AMAX19_2p5W_SB;
motor_3 = MotorParam;
AMAX22_5W_SB;
motor_4 = MotorParam;
AMAX22_6W_SB;
motor_5 = MotorParam;

% Find the best motor for q1
motors_param = [motor_1 motor_2 motor_3 motor_4 motor_5];

% Name motors
motors_name = categorical({'AMAX12_p75W_SB', 'AMAX16_2W_SB',...
    'AMAX19_2p5W_SB', 'AMAX22_5W_SB', 'AMAX22_6W_SB'
});

% Use loop to find the step response for each motor for q1
% Fix q0 to the default motor
q0 = motor_4 .* motor_param_unit_convert;

% Try each motor
riseTimes = zeros(1, 5);
settleTimes = zeros(1, 5);
overshoots = zeros(1, 5);
for i = 1:size(motors_param, 2)
    q1 = motors_param(1:end, i) .* motor_param_unit_convert;
    System;
    Control;

    % Compare system step responses of q0
    stepRes = stepinfo(ol_q0);
    disp(stepRes);

    % Record values
    riseTimes(i) = stepRes.RiseTime;
    settleTimes(i) = stepRes.SettlingTime;
    overshoots(i) = stepRes.Overshoot;
end

% Compare these motors at large
figure;
subplot(2, 2, 1);
bar(motors_name, riseTimes);
title('Rise time');
subplot(2, 2, 2);
bar(motors_name, settleTimes);
title('Settle time');
subplot(2, 2, 3);
bar(motors_name, overshoots);
title('Overshoot');
ylim([0, 100]);