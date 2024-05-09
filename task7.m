run("task6.m")
%% Constructing reduced model

rows_to_keep_task7 = [3,4];
cols_to_keep_task7 = [3,4];
A_two_states_task7 = A_ac_long(rows_to_keep_task7,cols_to_keep_task7);
disp("Matrix A pre reduction:");
disp(A_ac_long);
disp("Matrix A Task 7 reduced:");
disp(A_two_states_task7);
B_two_states_task7 = B_ac_long(rows_to_keep_task7,:);
disp("Matrix B Task 7 reduced:");
disp(B_two_states_task7);
C_two_states_task7 = C_ac_long(rows_to_keep_task7,cols_to_keep_task7);
disp("Matrix C Task 7 reduced:");
disp(C_two_states_task7);
D_two_states_task7 = D_ac_long(rows_to_keep_task7,:);
disp("Matrix D Task 7 reduced:");
disp(D_two_states_task7);
task7_states_reduced = ["alpha" ; "q"] 

%% State Space/Transfer Fucntions for Model Comparison

task7_ss_reduced = ss(A_two_states_task7,B_two_states_task7,C_two_states_task7,D_two_states_task7);
task7_ss_extended = Sys_Long_6;
task7_tf_reduced = minreal(tf(task7_ss_reduced));
task7_tf_extended = minreal(tf(task7_ss_extended));


%% Comparison

t_op_task7 = 100;
[y_short_reduced_task7, t_short_reduced_task7] = step(task7_ss_reduced, 0:0.001:t_op_task7);
[y_short_extended_task7, t_short_extended_task7] = step(task7_ss_extended, 0:0.001:t_op_task7);

figure;

plot(t_short_reduced_task7, y_short_reduced_task7(:, 1))
xlabel('Time [s]');
ylabel('Theta [rad]');
title('Pitch Angle response for Short Period:');

hold on
plot(t_short_extended_task7, y_short_extended_task7(:, 3));
xlabel('Time [s]');
ylabel('Theta [rad]');
hold off
legend("Reduced Model", "Extended Model");

plot(t_short_reduced_task7, y_short_reduced_task7(:, 2))
xlabel('Time [s]');
ylabel('q [rad/s]');
title('Pitch Rate response for Short Period:');

hold on
plot(t_short_extended_task7, y_short_extended_task7(:, 4));
xlabel('Time [s]');
ylabel('q [rad/s]');
hold off
legend("Reduced Model", "Extended Model");
hold off

set(gcf, 'units', 'points', 'position', [0, 0, 800, 400]);
%% Transfer Function derived from Stab. Criteria

ft_2_m = 0.3048
Natural_freq_criteria = 0.03 * velocity * ft_2_m
theta_2_criteria = 1/ (0.75 * Natural_freq_criteria)
damping_criteria = 0.5
kq = task7_tf_reduced.Numerator{2, 1}

Expected_tf_after_controller = minreal(tf([0 kq(1,3)*theta_2_criteria kq(1,3)], [1 2*damping_criteria*Natural_freq_criteria Natural_freq_criteria^2]))
poles_expected = pole(Expected_tf_after_controller)
Expected_tf_after_controller
%% Pole Placement

K_task7 = place(task7_ss_reduced.A,task7_ss_reduced.B,poles_expected)
A_gained_task7 = task7_ss_reduced.A - task7_ss_reduced.B*K_task7
sys_closed_task7_ss = ss(A_gained_task7, task7_ss_reduced.B, task7_ss_reduced.C, task7_ss_reduced.D);
sys_closed_task7_tf = tf(sys_closed_task7_ss)

sys_closed_task7_tf_exp = feedback(task7_tf_reduced,K_task7)
sys_closed_task7_ss_exp = ss(sys_closed_task7_tf_exp)

velocity_gust = 4.572;
alpha_induced = atan2(velocity_gust, velocity*ft_2_m)
elevator_max_deflec  = K_task7(1,1) * alpha_induced


%% New Transfer functions

s = tf('s');
T_prev = sys_closed_task7_tf.Numerator{2,1}(1,2) / sys_closed_task7_tf.Numerator{2,1}(1,3)

prev_zero = (1 + T_prev * s);
Design_zero = (1 + theta_2_criteria * s);

tf_lead_lag_prefilter_task7 = Design_zero/prev_zero
final_model_task7 = minreal(tf_lead_lag_prefilter_task7*sys_closed_task7_tf(2))
final_model_task7_ss = ss(final_model_task7)

final_model_task7_tf_exp = feedback(final_model_task7,1,+1)
final_model_task7_ss_exp = ss(final_model_task7_tf_exp)

T_current_task7 = final_model_task7_tf_exp.Numerator{1, 1}(1,2) / final_model_task7_tf_exp.Numerator{1, 1}(1,3)
%% CAP Gibson Criteria

CAP_design = Natural_freq_criteria^2 / ((velocity*ft_2_m/gd) * (1/theta_2_criteria))
CAP_current = final_model_task7_tf_exp.Denominator{1,1}(1,3) / ((velocity*ft_2_m/gd) * (1/T_current_task7))

DB_design = theta_2_criteria - ((2*damping_criteria) / Natural_freq_criteria)
DB_current = T_current_task7 - ((2*((final_model_task7_tf_exp.Denominator{1,1}(1,2)))/(2*sqrt(final_model_task7_tf_exp.Denominator{1,1}(1,3)))) / sqrt(final_model_task7_tf_exp.Denominator{1,1}(1,3)))

max_q_over_steady_state_q = min(y_short_reduced_task7(:,2)) / y_short_reduced_task7(100000,2)
%% 
%