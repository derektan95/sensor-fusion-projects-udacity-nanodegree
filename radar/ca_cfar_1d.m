% Implement 1D CFAR using lagging cells on the given noise and target scenario.

% Close and delete all currently open figures
close all;

%% Generating signal
% Data_points
Ns = 1000;

% Generate random noise
signal=abs(randn(Ns,1));

%Targets location. Assigning bin 100, 200, 300 and 700 as Targets with the amplitudes of 8, 9, 4, 11.
signal([100 ,200, 300, 700])=[8 15 7 13];

% %plot the output
% figure (1)
% plot(signal);

%% Applying CFAR
% TODO: Apply CFAR to detect the targets by filtering the noise.

% 1. Define the following:
% 1a. Training Cells
% 1b. Guard Cells 
% Must be even (sums leading and trailing cells)
T = 12;
G = 4;      

% Offset : Adding room above noise threshold for desired SNR 
offset = 6;

% Vector to hold threshold values 
threshold_cfar = [];

%Vector to hold final signal after thresholding
signal_cfar = [];

% 2. Slide window across the signal length
for i = 1:(Ns-(G+T+1))     

    % 2. - 5. Determine the noise threshold by measuring it within the training cells
    leading_training_cells = signal(i : i+ T/2 -1);
    trailing_training_cells = signal(i+(G + T/2 + 1) : i+(G + T));
    training_cells = vertcat(leading_training_cells, trailing_training_cells);
    noise_threshold = offset * mean(training_cells);
    threshold_cfar = [threshold_cfar, {noise_threshold}];
    
    % 6. Measuring the signal within the CUT
    signal_CUT = signal(i+ T/2 + G/2);

    % 8. Filter the signal above the threshold
    if (signal_CUT < noise_threshold)
        signal_CUT = 0;
    end

    signal_cfar = [signal_cfar, {signal_CUT}];
end

% plot(signal, 'b--');
hold on, plot(signal, 'b--');
hold on, plot (cell2mat(circshift(threshold_cfar, G/2)), 'r--', 'LineWidth',2);
hold on, plot (cell2mat(circshift(signal_cfar, G/2)), 'g--', 'LineWidth',4);
legend('Signal', 'CFAR THreshold', 'detection');
