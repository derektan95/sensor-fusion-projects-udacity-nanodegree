% Doppler Velocity Calculation
c = 3*10^8;         %speed of light
frequency = 77e9;   % Radar Operating Frequency in Hz

% TODO : Calculate the wavelength
wavelength = frequency / c;


% TODO : Define the doppler shifts in Hz using the information from above
% (detected by FMCW Radar)
doppler_frequency_shifts = [3e3, -4.5e3, 11e3, -3e3];


% TODO : Calculate the velocity of the targets  fd = 2*vr/lambda
rel_vel = doppler_frequency_shifts * wavelength / 2;



% TODO: Display results
disp(rel_vel);