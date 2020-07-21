%Operating frequency (Hz)
fc = 77.0e9;

%Transmitted power (W)
P_s = 3e-3;

%Antenna Gain (linear)
G =  10000;

%Minimum Detectable Power
P_e = 1e-10;

% Radar Cross Section (RCS) of a car
RCS = 100;

%Speed of light
c = 3*10^8;

%TODO: Calculate the wavelength
wavelength = c / fc;


%TODO : Measure the Maximum Range a Radar can see (in m)
range = ( (P_s* G^2 * wavelength^2 * RCS) / (P_e * (4*pi)^3) ) ^(1/4);
disp(range);
