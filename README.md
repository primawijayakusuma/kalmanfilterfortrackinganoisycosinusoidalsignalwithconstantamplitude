# kalmanfilterfortrackinganoisycosinusoidalsignalwithconstantamplitude
This research addresses the critical need for accurate signal tracking in diverse applications by focusing on the application of the Kalman Filter to the challenging task of tracing noisy cosinusoidal signals with constant amplitudes. 

Code : 

% Kalman Filter for Tracking a Noisy Cosinusoidal Signal with Constant Amplitude

% Define system parameters
freq = 1;               % frequency of the signal
amplitude = 1;          % constant amplitude of the signal
phase = pi/4;           % phase of the signal
sigma_x = 0.1;          % standard deviation of the process noise
sigma_z = 0.1;          % standard deviation of the measurement noise

% Define time vector
t = linspace(0,10,5001); % 5001 equally spaced points from 0 to 10

% Generate true signal and add noise to get measurement
x_true = amplitude*cos(2*pi*freq*t + phase); % true signal
x_meas = x_true + sigma_z*randn(size(t));   % measurement signal

% Estimation of Initial Value Using Least Squares
X = [ones(length(t),1), sin(2*pi*freq*t)', cos(2*pi*freq*t)'];
x_est(:,1) = X\transpose(x_meas);

% Initialize Kalman filter parameters
A = [1 0 0; 0 cos(2*pi*freq) -sin(2*pi*freq); 0 sin(2*pi*freq) cos(2*pi*freq)];
B = [0; 0; 0];
H = [1 0 0];
Q = sigma_x^2 * eye(3);
R = sigma_z^2;
P(:,:,1) = eye(3);

% Kalman filter iteration
for i = 2:length(t)
    % Prediction
    x_pred(:,i) = A*x_est(:,i-1) + B*0;
    P_pred(:,:,i) = A*P(:,:,i-1)*transpose(A) + Q;
    
    % Update
    K = P_pred(:,:,i)*transpose(H)/(H*P_pred(:,:,i)*transpose(H) + R);
    x_est(:,i) = x_pred(:,i) + K*(x_meas(i) - H*x_pred(:,i));
    P(:,:,i) = (eye(3) - K*H)*P_pred(:,:,i);
end

% Plot the results
figure;
plot(t,x_true,'r-',t,x_meas,'b.',t,x_est(1,:),'g--');
legend('True Signal','Measurement','Estimated Signal');
xlabel('Time');
ylabel('Signal Amplitude');
title('Kalman Filter for Tracking a Noisy Cosinusoidal Signal');

second frequency : 

% Kalman Filter for Tracking a Noisy Cosinusoidal Signal with Constant Amplitude

% Define system parameters
freq = 3;               % frequency of the signal
amplitude = 1;          % constant amplitude of the signal
phase = pi/4;           % phase of the signal
sigma_x = 0.2;          % standard deviation of the process noise
sigma_z = 0.1;          % standard deviation of the measurement noise

% Define time vector
t = linspace(0,10,5001); % 5001 equally spaced points from 0 to 10

% Generate true signal and add noise to get measurement
x_true = amplitude*cos(2*pi*freq*t + phase); % true signal
x_meas = x_true + sigma_z*randn(size(t));   % measurement signal

% Estimation of Initial Value Using Least Squares
X = [ones(length(t),1), sin(2*pi*freq*t)', cos(2*pi*freq*t)'];
x_est(:,1) = X\transpose(x_meas);

% Initialize Kalman filter parameters
A = [1 0 0; 0 cos(2*pi*freq) -sin(2*pi*freq); 0 sin(2*pi*freq) cos(2*pi*freq)];
B = [0; 0; 0];
H = [1 0 0];
Q = sigma_x^2 * eye(3);
R = sigma_z^2;
P(:,:,1) = eye(3);

% Kalman filter iteration
for i = 2:length(t)
    % Prediction
    x_pred(:,i) = A*x_est(:,i-1) + B*0;
    P_pred(:,:,i) = A*P(:,:,i-1)*transpose(A) + Q;
    
    % Update
    K = P_pred(:,:,i)*transpose(H)/(H*P_pred(:,:,i)*transpose(H) + R);
    x_est(:,i) = x_pred(:,i) + K*(x_meas(i) - H*x_pred(:,i));
    P(:,:,i) = (eye(3) - K*H)*P_pred(:,:,i);
end

% Plot the results
figure;
plot(t,x_true,'r-',t,x_meas,'b.',t,x_est(1,:),'g--');
legend('True Signal','Measurement','Estimated Signal');
xlabel('Time');
ylabel('Signal Amplitude');
title('Kalman Filter for Tracking a Noisy Cosinusoidal Signal');

