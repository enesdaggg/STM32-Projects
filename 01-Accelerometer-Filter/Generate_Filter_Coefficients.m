% Generate_Filter_Coefficients.m
% Generates IIR Butterworth filter coefficients for 2nd, 4th, and 8th order
% filters with Fc=5Hz and Fs=100Hz

clc;
clear;
close all;

% Filter Parameters
Fs = 100;           % Sampling frequency (Hz)
Fc = 5;             % Cutoff frequency (Hz)
Wn = Fc/(Fs/2);     % Normalized cutoff frequency

fprintf('=== IIR Butterworth Filter Coefficient Generator ===\n');
fprintf('Sampling Frequency (Fs): %d Hz\n', Fs);
fprintf('Cutoff Frequency (Fc): %d Hz\n', Fc);
fprintf('Normalized Cutoff (Wn): %.4f\n\n', Wn);

% Generate filters of different orders
orders = [2, 4, 8];

for i = 1:length(orders)
    order = orders(i);
    
    % Design Butterworth filter
    [b, a] = butter(order, Wn, 'low');
    
    fprintf('--- %dth Order Butterworth Filter ---\n', order);
    fprintf('Numerator Coefficients (B):\n');
    fprintf('const float B_COEFFS_%d[%d] = {', order, order+1);
    for j = 1:length(b)
        fprintf('%.6ff', b(j));
        if j < length(b)
            fprintf(', ');
        end
    end
    fprintf('};\n\n');
    
    fprintf('Denominator Coefficients (A):\n');
    fprintf('const float A_COEFFS_%d[%d] = {', order, order+1);
    for j = 1:length(a)
        fprintf('%.6ff', a(j));
        if j < length(a)
            fprintf(', ');
        end
    end
    fprintf('};\n\n');
    
    % Frequency response analysis
    [H, F] = freqz(b, a, 1024, Fs);
    mag_dB = 20*log10(abs(H));
    
    % Find -3dB point
    idx_3dB = find(mag_dB >= -3, 1, 'last');
    fc_actual = F(idx_3dB);
    
    fprintf('Actual -3dB cutoff frequency: %.2f Hz\n', fc_actual);
    fprintf('Attenuation at Nyquist (50Hz): %.2f dB\n\n', mag_dB(end));
end

% Plot frequency responses for comparison
figure('Name', 'Filter Frequency Responses', 'Position', [100, 100, 1200, 800]);

for i = 1:length(orders)
    order = orders(i);
    [b, a] = butter(order, Wn, 'low');
    [H, F] = freqz(b, a, 1024, Fs);
    mag_dB = 20*log10(abs(H));
    phase_deg = angle(H) * 180/pi;
    
    % Magnitude plot
    subplot(2, 1, 1);
    plot(F, mag_dB, 'LineWidth', 2, 'DisplayName', sprintf('%dth Order', order));
    hold on;
    
    % Phase plot
    subplot(2, 1, 2);
    plot(F, phase_deg, 'LineWidth', 2, 'DisplayName', sprintf('%dth Order', order));
    hold on;
end

% Format magnitude plot
subplot(2, 1, 1);
grid on;
xlabel('Frequency (Hz)');
ylabel('Magnitude (dB)');
title('Magnitude Response Comparison');
legend('Location', 'southwest');
xlim([0, 50]);
ylim([-100, 5]);
yline(-3, '--r', '-3dB', 'LabelHorizontalAlignment', 'left');
xline(Fc, '--k', 'Fc=5Hz', 'LabelVerticalAlignment', 'bottom');

% Format phase plot
subplot(2, 1, 2);
grid on;
xlabel('Frequency (Hz)');
ylabel('Phase (degrees)');
title('Phase Response Comparison');
legend('Location', 'southwest');
xlim([0, 50]);

fprintf('=== Filter Design Complete ===\n');
fprintf('Copy the coefficients above into your main.c file.\n');
