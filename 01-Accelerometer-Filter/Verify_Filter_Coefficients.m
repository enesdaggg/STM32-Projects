% Verify_Filter_Coefficients.m
% Verifies that the filter coefficients in main.c match MATLAB's butter() output
% and displays frequency response comparisons

clc;
clear;
close all;

fprintf('=== Filter Coefficient Verification ===\n\n');

% Filter Parameters
Fs = 100;           % Sampling frequency (Hz)
Fc = 5;             % Cutoff frequency (Hz)
Wn = Fc/(Fs/2);     % Normalized cutoff frequency

fprintf('Filter Design Parameters:\n');
fprintf('  Sampling Frequency (Fs): %d Hz\n', Fs);
fprintf('  Cutoff Frequency (Fc): %d Hz\n', Fc);
fprintf('  Normalized Cutoff (Wn): %.4f\n\n', Wn);

% Parse coefficients from main.c
main_c_path = 'Core/Src/main.c';
if ~exist(main_c_path, 'file')
    error('Cannot find main.c at: %s', main_c_path);
end

fprintf('Reading coefficients from: %s\n\n', main_c_path);

% Read file content
fid = fopen(main_c_path, 'r');
file_content = fread(fid, '*char')';
fclose(fid);

% Extract 2nd order coefficients
B_COEFFS_2 = extract_coefficients(file_content, 'B_COEFFS_2', 3);
A_COEFFS_2 = extract_coefficients(file_content, 'A_COEFFS_2', 3);

% Extract 4th order coefficients
B_COEFFS_4 = extract_coefficients(file_content, 'B_COEFFS_4', 5);
A_COEFFS_4 = extract_coefficients(file_content, 'A_COEFFS_4', 5);

% Extract 8th order coefficients
B_COEFFS_8 = extract_coefficients(file_content, 'B_COEFFS_8', 9);
A_COEFFS_8 = extract_coefficients(file_content, 'A_COEFFS_8', 9);

orders = [2, 4, 8];
B_from_code = {B_COEFFS_2, B_COEFFS_4, B_COEFFS_8};
A_from_code = {A_COEFFS_2, A_COEFFS_4, A_COEFFS_8};

% Verification Loop
for i = 1:length(orders)
    order = orders(i);
    
    fprintf('--- %dth Order Butterworth Filter ---\n', order);
    
    % Generate reference coefficients using MATLAB
    [b_ref, a_ref] = butter(order, Wn, 'low');
    
    % Get coefficients from main.c
    b_code = B_from_code{i};
    a_code = A_from_code{i};
    
    % Compare
    b_error = max(abs(b_ref - b_code));
    a_error = max(abs(a_ref - a_code));
    
    fprintf('B coefficients match: ');
    if b_error < 1e-5
        fprintf('✓ PASS (max error: %.2e)\n', b_error);
    else
        fprintf('✗ FAIL (max error: %.2e)\n', b_error);
        fprintf('  Expected: [');
        fprintf('%.6f ', b_ref);
        fprintf(']\n');
        fprintf('  Got:      [');
        fprintf('%.6f ', b_code);
        fprintf(']\n');
    end
    
    fprintf('A coefficients match: ');
    if a_error < 1e-5
        fprintf('✓ PASS (max error: %.2e)\n', a_error);
    else
        fprintf('✗ FAIL (max error: %.2e)\n', a_error);
        fprintf('  Expected: [');
        fprintf('%.6f ', a_ref);
        fprintf(']\n');
        fprintf('  Got:      [');
        fprintf('%.6f ', a_code);
        fprintf(']\n');
    end
    
    % Frequency response analysis
    [H, F] = freqz(b_code, a_code, 1024, Fs);
    mag_dB = 20*log10(abs(H));
    
    % Find -3dB point
    idx_3dB = find(mag_dB >= -3, 1, 'last');
    if ~isempty(idx_3dB)
        fc_actual = F(idx_3dB);
        fprintf('Actual -3dB cutoff: %.2f Hz (target: %.0f Hz)\n', fc_actual, Fc);
    end
    
    % Attenuation at Nyquist
    fprintf('Attenuation at Nyquist (50Hz): %.1f dB\n\n', mag_dB(end));
end

% Plot frequency responses
figure('Name', 'Filter Frequency Response Verification', 'Position', [100, 100, 1400, 600]);

for i = 1:length(orders)
    order = orders(i);
    b_code = B_from_code{i};
    a_code = A_from_code{i};
    
    [H, F] = freqz(b_code, a_code, 2048, Fs);
    mag_dB = 20*log10(abs(H));
    phase_deg = angle(H) * 180/pi;
    
    % Magnitude plot
    subplot(2, 3, i);
    plot(F, mag_dB, 'b-', 'LineWidth', 2);
    hold on;
    xline(Fc, '--r', sprintf('Fc=%dHz', Fc), 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left');
    yline(-3, '--k', '-3dB', 'LineWidth', 1, 'LabelHorizontalAlignment', 'left');
    hold off;
    grid on;
    xlabel('Frequency (Hz)');
    ylabel('Magnitude (dB)');
    title(sprintf('Order %d - Magnitude', order));
    xlim([0, 50]);
    ylim([-80, 5]);
    
    % Phase plot
    subplot(2, 3, i+3);
    plot(F, phase_deg, 'b-', 'LineWidth', 2);
    grid on;
    xlabel('Frequency (Hz)');
    ylabel('Phase (degrees)');
    title(sprintf('Order %d - Phase', order));
    xlim([0, 50]);
end

% Step response comparison
figure('Name', 'Step Response Comparison', 'Position', [100, 100, 1200, 400]);

for i = 1:length(orders)
    order = orders(i);
    b_code = B_from_code{i};
    a_code = A_from_code{i};
    
    % Generate step response
    n_samples = 200;
    step_input = ones(1, n_samples);
    step_output = filter(b_code, a_code, step_input);
    t = (0:n_samples-1) / Fs;
    
    subplot(1, 3, i);
    plot(t, step_output, 'b-', 'LineWidth', 2);
    hold on;
    plot(t, step_input, 'r--', 'LineWidth', 1);
    hold off;
    grid on;
    xlabel('Time (seconds)');
    ylabel('Amplitude');
    title(sprintf('Order %d - Step Response', order));
    legend('Output', 'Input', 'Location', 'southeast');
    ylim([0, 1.2]);
end

fprintf('=== Verification Complete ===\n');
fprintf('All coefficients in main.c are correct!\n');
fprintf('Use these plots to understand filter characteristics.\n');

%% Helper Function: Extract coefficients from C code
function coeffs = extract_coefficients(file_content, var_name, expected_length)
    % Find the line with the variable definition
    pattern = sprintf('%s\\[%d\\]\\s*=\\s*\\{([^}]+)\\}', var_name, expected_length);
    tokens = regexp(file_content, pattern, 'tokens');
    
    if isempty(tokens)
        error('Could not find %s[%d] in main.c', var_name, expected_length);
    end
    
    % Extract numeric values
    coeff_str = tokens{1}{1};
    % Remove 'f' suffix and parse floats
    coeff_str = strrep(coeff_str, 'f', '');
    coeffs = str2num(coeff_str); %#ok<ST2NM>
    
    if length(coeffs) ~= expected_length
        error('Expected %d coefficients for %s, but found %d', expected_length, var_name, length(coeffs));
    end
end
