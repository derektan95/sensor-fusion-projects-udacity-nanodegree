% Define key parameters
c = 3 * 10^8;
R_max = 300;    % max range of radar in m
range_res = 1;  % range resolution in m


% TODO : Find the Bsweep of chirp for 1 m resolution
B_sweep = c / (2 * range_res);          % Max - Min Freq in 1 chirp

% TODO : Calculate the chirp time based on the Radar's Max Range
T_sweep = 5.5 * 2 * R_max / c;          % Time taken for 1 chirp

% TODO : define the frequency shifts 
beat_freq_arr = [0, 1.1e6, 13e6, 24e6];       % Frequency difference betwween received vs emitted signal in MHz

% Display the calculated range
calculated_range = (c * T_sweep * beat_freq_arr) / (2 * B_sweep);       % Range of target
disp(calculated_range);