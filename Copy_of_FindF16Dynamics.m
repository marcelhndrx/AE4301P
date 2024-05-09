% FindF16Dynamics.m

function FindF16Dynamics(xa)
    clear;

    %%
    gd = 9.80665;

    %%
    global fi_flag_Simulink

    newline = sprintf('\n');

    %% Set constant altitude and velocity
    altitude = 15000;
    velocity = 500;

    FC_flag = 1; % Trim for steady wings-level flight

    %% Initial guess for trim
    %%
    thrust = 5000;          % thrust, lbs
    elevator = -0.09;       % elevator, degrees
    alpha = 8.49;              % AOA, degrees
    rudder = -0.01;             % rudder angle, degrees
    aileron = 0.01;            % aileron, degrees

    %% Find trim for Hifi model
    %%
    disp('Trimming High Fidelity Model:');
    fi_flag_Simulink = 1;
    [trim_state_hi, trim_thrust_hi, trim_control_hi, dLEF, xu_hi] = trim_F16(thrust, elevator, alpha, aileron, rudder, velocity, altitude, FC_flag);

    %% Find the state space model for the hifi model
    %%
    trim_state_lin = trim_state_hi; trim_thrust_lin = trim_thrust_hi; trim_control_lin = trim_control_hi;
    operating_point = operpoint('LIN_F16Block');
    operating_point.Inputs(1).u = trim_thrust_lin; operating_point.Inputs(2).u = trim_control_lin(1);
    operating_point.Inputs(3).u = trim_control_lin(2); operating_point.Inputs(4).u = trim_control_lin(3);

    SS_hi = linearize('LIN_F16Block');

    disp(' ');
    %% Find trim for lofi model
    %%
    disp('Trimming Low Fidelity Model:');
    fi_flag_Simulink = 0;
    [trim_state_lo, trim_thrust_lo, trim_control_lo, dLEF, xu_lo] = trim_F16(thrust, elevator, alpha, aileron, rudder, velocity, altitude, FC_flag);

    %% Find the state space model for the lofi model
    %%
    trim_state_lin = trim_state_lo; trim_thrust_lin = trim_thrust_lo; trim_control_lin = trim_control_lo;
    operating_point = operpoint('LIN_F16Block');
    operating_point.Inputs(1).u = trim_thrust_lin; operating_point.Inputs(2).u = trim_control_lin(1);
    operating_point.Inputs(3).u = trim_control_lin(2); operating_point.Inputs(4).u = trim_control_lin(3);

    SS_lo = linearize('LIN_F16Block');

    % (The rest of your code)

    %%%%%%%%%%%%%%%%%%%%%%%
    %% Longitudinal Direction
    %%%%%%%%%%%%%%%%%%%%%%%

    long_states = [3 5 7 8 11 13 14];
    long_inputs = [1 2];
    long_outputs = [3 5 7 8 11];

    SS_long_lo = ss(SS_lo.A(long_states,long_states), SS_lo.B(long_states,long_inputs), SS_lo.C(long_outputs,long_states), SS_lo.D(long_outputs,long_inputs));
    SS_long_hi = ss(SS_hi.A(long_states,long_states), SS_hi.B(long_states,long_inputs), SS_hi.C(long_outputs,long_states), SS_hi.D(long_outputs,long_inputs));

    SS_long_lo.StateName = SS_lo.StateName(long_states);
    SS_long_hi.StateName = SS_hi.StateName(long_states);

    SS_long_lo.InputName= SS_lo.InputName(long_inputs);
    SS_long_hi.InputName= SS_hi.InputName(long_inputs);

    %%%%%%%%%%%%%%%%%%%%
    %% Lateral Direction
    %%%%%%%%%%%%%%%%%%%%

    lat_states = [4 6 7 9 10 12 13 15 16];
    lat_inputs = [1 3 4];
    lat_outputs = [4 6 7 9 10 12];
    SS_lat_lo = ss(SS_lo.A(lat_states,lat_states), SS_lo.B(lat_states,lat_inputs), SS_lo.C(lat_outputs,lat_states), SS_lo.D(lat_outputs,lat_inputs));
    SS_lat_hi = ss(SS_hi.A(lat_states,lat_states), SS_hi.B(lat_states,lat_inputs), SS_hi.C(lat_outputs,lat_states), SS_hi.D(lat_outputs,lat_inputs));

    SS_lat_lo.StateName = SS_lo.StateName(lat_states);
    SS_lat_hi.StateName = SS_hi.StateName(lat_states);

    SS_lat_lo.InputName= SS_lo.InputName(lat_inputs);
    SS_lat_hi.InputName= SS_hi.InputName(lat_inputs);

    %% Calculate and Plot Transfer Function for Different xa Values
    CalculateTransferFunction(SS_lo);

end

function CalculateTransferFunction(SS_lo)

    % List of values for xa
    xa_values = [0, 5, 5.9, 6, 7, 15];  % Add or modify values as needed

    % Plot settings
    figure;

    % Iterate over the list of xa values
    for xa = xa_values
        % Initiating xa and running relevant script
        run("FindF16Dynamics.m");

        % Defining transfer function between an and delta_e
        sys = tf(SS_lo);
        sys_de_an = sys(19, 2);
        sys_de_an_minreal = minreal(sys_de_an);

        % Update the elevator deflection (delta_e) value in the transfer function
        sys_de_an_minreal.Numerator{1}(2) = xa;

        % Display the updated transfer function
        disp('Transfer Function after Minreal:');
        disp(sys_de_an_minreal);

        % Step response with a step amplitude of -5 degrees for δe
        time_step = 0.0001; % Desired time step
        final_time = 5;      % Desired final time
        t = 0:time_step:final_time;

        % Input signal (step in δe)
        u_delta_e = -5 * ones(size(t));

        % Simulate the system response
        y = lsim(sys_de_an_minreal, u_delta_e, t);

        % Plot the system response for the current xa value
        plot(t, y, 'LineWidth', 1.5, 'DisplayName', ['xa = ', num2str(xa)]);
        hold on;
    end

    title('Step Response of the System to δe = -5° for Different Accelerometer Positions');
    xlabel('Time [s]');
    ylabel('Normal Acceleration [g]');
    grid on;
    legend('show');
    xlim([0, final_time]);

end
