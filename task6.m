% Defining certain quantities required for the task:
% altitude = 10000; %[ft]
% velocity = 350; %[ft/s]


% Defining certain quantities required for the task, THESE SHOULD BE UPDATED IN FINDF16DYNAMICS FIRST:
%altitude = 10000; %[ft] 
%velocity = 350; %[ft/s]

%% Longitudinal:
%Finding A_original
rows_to_remove = [1,6];
cols_to_remove = [1,6];
A_original_long = SS_long_lo.A;
A_original_long(rows_to_remove, :) = [];
A_original_long(:,cols_to_remove) = [];
%disp(A_ac);
%Finding A_ac/B_ac
remove_ac_row = 5;
remove_ac_col = 5;
B_ac_long = A_original_long(1:4,5);
a_elev = A_original_long(5,5);
A_original_long(:,remove_ac_col) = [];
A_original_long(remove_ac_row,:) = [];
A_ac_long = A_original_long; 
% disp(A_ac_long);
% disp(B_ac_long);
% disp(a_elev);

%Finding C_original
cols_to_remove2 = [1,6];
rows_to_remove2 = 1;
C_original_long = SS_long_lo.C;

C_original_long(rows_to_remove2,:) = [];
C_original_long(:,cols_to_remove2) = [];
%Finding C_ac/D_ac
C_ac_long = C_original_long(1:4,1:4);
D_ac_long = C_original_long(:,5); 
% disp(C_ac_long);
% disp(D_ac_long);

%Eigenmotion Analysis
p = pi;
EigenValues_Long = eig(A_ac_long);
mag_sp = abs(EigenValues_Long(3));
mag_ph = abs(EigenValues_Long(1));
Nat_freq_sp = mag_sp;
Nat_freq_ph = mag_ph;
Damp_ratio_sp = -real(EigenValues_Long(3))/Nat_freq_sp;
Damp_ratio_ph = -real(EigenValues_Long(1))/Nat_freq_ph;
Damped_nat_freq_sp = Nat_freq_sp*sqrt(1-Damp_ratio_sp^2);
Damped_nat_freq_ph = Nat_freq_ph*sqrt(1-Damp_ratio_ph^2);
Perio_sp = (2*p)/Damped_nat_freq_sp;
Perio_ph = (2*p)/Damped_nat_freq_ph;
T_half_sp = log(0.5)/real(EigenValues_Long(3));
T_half_ph = log(0.5)/real(EigenValues_Long(1));

t_end = 100;

Sys_Long_6 = ss(A_ac_long, B_ac_long, C_ac_long, D_ac_long);

% SHORT PERIOD RESPONSES
[y_short, t_short] = impulse(Sys_Long_6, 0:0.001:t_end);

% Creating a new figure for short period responses
figure;

% Plotting subplots for each variable
subplot(2, 2, 1);
plot(t_short, y_short(:, 1));
xlabel('Time [s]');
ylabel('Theta [rad]');
title('Pitch Angle response for Short Period');

subplot(2, 2, 2);
plot(t_short, y_short(:, 2));
xlabel('Time [s]');
ylabel('Vt');
title('Velocity response for Short Period');

subplot(2, 2, 3);
plot(t_short, y_short(:, 3));
xlabel('Time [s]');
ylabel('Alpha [rad/s]');
title('Angle of Attack response for Short Period');

subplot(2, 2, 4);
plot(t_short, y_short(:, 4));
xlabel('Time [s]');
ylabel('q [rad/s]');
title('Pitch Rate response for Short Period');

% Adjusting overall figure settings
set(gcf, 'units', 'points', 'position', [0, 0, 800, 400]);


% PHUGOID RESPONSES
x0 = [0, 0, 0.1, 0];
[y_phugoid, t_phugoid] = initial(Sys_Long_6, x0, 0:0.001:t_end);

% Creating a new figure for phugoid responses
figure;

% Plotting subplots for each variable
subplot(2, 2, 1);
plot(t_phugoid, y_phugoid(:, 1));
xlabel('Time [s]');
ylabel('Theta [rad]');
title('Pitch Angle response for Phugoid');

subplot(2, 2, 2);
plot(t_phugoid, y_phugoid(:, 2));
xlabel('Time [s]');
ylabel('Vt');
title('Velocity response for Phugoid');

subplot(2, 2, 3);
plot(t_phugoid, y_phugoid(:, 3));
xlabel('Time [s]');
ylabel('Alpha [rad/s]');
title('Angle of Attack response for Phugoid');

subplot(2, 2, 4);
plot(t_phugoid, y_phugoid(:, 4));
xlabel('Time [s]');
ylabel('q [rad/s]');
title('Pitch Rate response for Phugoid');

% Adjusting overall figure settings
set(gcf, 'units', 'points', 'position', [0, 0, 800, 400]);

%transfer_function_long = minreal(tf(Sys_Long_6));

% time_step2 = 0.05; %[s]
% final_time2 = 100; % Desired final time 
% % Simulate the system response with specified time vector and time step
% figure(1);
% [y1, t1] = step(minreal(transfer_function_long(1,1)), 0:time_step2:final_time2);
% plot(t1,y1);
% title('Response for theta');
% figure(2);
% [y2, t2] = step(transfer_function_long(2,1), 0:time_step2:final_time2);
% plot(t2,y2);
% title('Response for Vt');
% figure(3);
% [y3, t3] = step(transfer_function_long(3,1), 0:time_step2:final_time2);
% plot(t3,y3);
% title('Response for alpha');
% figure(4);
% [y4, t4] = step(transfer_function_long(4,1), 0:time_step2:final_time2);
% plot(t4,y4);
% title('Response for q');



%% Lateral:
% reduction Lateral 


A_long_ori = SS_lat_lo.A ;
rows_to_remove = [2,3,7] ;
A_long_ori(rows_to_remove, :) = [];

cols_to_remove = [2,3,7] ;
A_long_ori(: , cols_to_remove) = [];

%Now we will obtain Aa/c and Ba/c 
rows_to_remove = [5,6];
cols_to_remove = [5,6];
Aa_c = A_long_ori;
Aa_c(rows_to_remove,:) = [];
Aa_c(:,cols_to_remove) = [];

row_ind = [1,2,3,4];
col_ind = [5,6];
Ba_c = A_long_ori(row_ind, col_ind);

%%%%%%%%_____________%%%%%%%


C_long_ori = SS_lat_lo.C;
cols_to_remove = [2,3,7];
C_long_ori(:, cols_to_remove) = [];
rows_to_remove = [2,3];
cols_to_remove = [5,6];
Ca_c = C_long_ori;
Ca_c(rows_to_remove,:) = [];
Ca_c(:,cols_to_remove) = [];

row_ind = [1,2,3,4];
col_ind = [5,6];

Da_c = C_long_ori(row_ind, col_ind);


eigenvalues = eig(Aa_c);

% Display the eigenvalues
disp('Eigenvalues of the matrix A:');
disp(eigenvalues);

%First two eigenvalues correspond to Dutch Roll, third one to spiral and
%4th to Aperiodic roll

%% PERIODIC
%DUTCH ROLL
wn_dr =  abs(eigenvalues(1));
damping_dr = -real(eigenvalues(1))/wn_dr;
P_dr = 2*pi / imag(eigenvalues(1));
T_half_dr = log(0.5)/ real(eigenvalues(1));

%% APERIODIC
% APERIODIC ROLL
wn_r = - abs(eigenvalues(4));
T_r = 1/wn_r;
T_half_r = log(0.5)/ real(eigenvalues(4));


% SPIRAL
wn_s = - abs(eigenvalues(3));
T_s = 1/wn_s;
T_half_s = log(0.5)/ real(eigenvalues(3));

%_____________________________________________________________________

% CREATING TRANSFER FUNCTIONS

sys_Lat_6 = ss(Aa_c, Ba_c, Ca_c, Da_c);

% Dutch ROll RESPONSES
[y_dr, t_dr] = impulse(sys_Lat_6, 0:0.001:30);
y_dr = y_dr(:, 4, 2);
figure;
plot(t_dr, y_dr);
xlabel('Time [s]');
ylabel('r [rad/s]');
title('Yaw Rate response for Dutch Roll');

% Aperiodic Roll RESPONSES
x0 = [0, 0, 0.1, 0];
[y_ar, t_ar] = initial(sys_Lat_6, x0, 0:0.001:40);
y_ar = y_ar(:, 3);
figure;
plot(t_ar, y_ar);
xlabel('Time [s]');
ylabel('r [rad/s]');
title('Roll Rate response for Aperiodic Roll');

% Spiral RESPONSES
x0 = [0.1, 0, 0, 0];
[y, t] = initial(sys_Lat_6, x0, t_end);
[y_sp, t_sp] = impulse(sys_Lat_6, 0:0.001:100);
figure;
plot(t_sp, y_sp(:, 1));
xlabel('Time [s]');
ylabel('r [rad/s]');
title('Spiral motion charactestics in','simulation with initial bank angle');



% % Convert state-space system to transfer function
% tf_sys = minreal(tf(sys));
% disp(tf_sys)
% 
% %%%%%%TASK 7 HERE
% 
% 
% rows_to_remove = [1,2];
% cols_to_remove = [1,2];
% A_a_c_sp = A_ac_long ;
% A_a_c_sp(rows_to_remove, :) = [];
% A_a_c_sp(:,cols_to_remove) = []; 
% 
% B_a_c_sp = B_ac_long ;
% B_a_c_sp(rows_to_remove, :) = [];
% 
% 
% C_a_c_sp = C_ac_long;
% C_a_c_sp(rows_to_remove, :) = [];
% C_a_c_sp(:,cols_to_remove) = []; 
% 
% 
% D_a_c_sp = D_ac_long ;
% D_a_c_sp(rows_to_remove, :) = [];
% 
% 
% Sys_Long_sp = ss(A_a_c_sp, B_a_c_sp, C_a_c_sp, D_a_c_sp);
% transfer_function_sp = minreal(tf(Sys_Long_sp));
% 
% time_step = 0.05; %[s]
% final_time = 16; % Desired final time 
% % Simulate the system response with specified time vector and time step
% figure(1);
% [y1, t1] = step(transfer_function_sp(1,1), 0:time_step:final_time);
% [y3, t3] = step(ransfer_function_long(3,1), 0:time_step:final_time);
% plot(t3,y3);
% legend("4 model");
% hold on;
% plot(t1,y1);
% legend("2 model");
% hold on;
% xlabel('Time [s]');
% ylabel('Angle of Attack [Rad]');
% 
% 
















