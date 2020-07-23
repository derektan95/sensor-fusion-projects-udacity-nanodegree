Fs = 1000;            % Sampling frequency                    
T = 1/Fs;             % Sampling period       
L = 1500;             % Length of signal
t = (0:L-1)*T;        % Time vector

% TODO: Form a signal containing a 77 Hz sinusoid of amplitude 0.7 and a 43Hz sinusoid of amplitude 2.
signal = 0.7*cos(2*pi*77*t) + 2*cos(2*pi*43*t);

% Corrupt the signal with noise 
corrupted_signal = signal + 2*randn(size(t));

% Plot the noisy signal in the time domain. It is difficult to identify the frequency components by looking at the signal X(t). 
figure (1);
plot(1000*t(1:50) ,corrupted_signal(1:50))
title('Signal Corrupted with Zero-Mean Random Noise')
xlabel('t (milliseconds)')
ylabel('X(t)')

% TODO : Compute the Fourier transform of the signal. 
signal_fft = fft(corrupted_signal);

% TODO : Compute the two-sided spectrum P2. Then compute the single-sided spectrum P1 based on P2 and the even-valued signal length L.
signal_fft = abs(signal_fft / L);      % Normalize results by length
P1  = signal_fft(1:L/2 +1);

% Plotting
figure (2);
f = Fs*(0:L/2)/L;
plot(f,P1) 
title('Single-Sided Amplitude Spectrum of X(t)')
xlabel('f (Hz)')
ylabel('|P1(f)|')

