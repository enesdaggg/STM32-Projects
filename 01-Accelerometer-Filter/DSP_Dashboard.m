classdef DSP_Dashboard < matlab.apps.AppBase
    % DSP_Dashboard: Real-Time Dual-Axis Digital Filter Performance Analyzer
    % Accelerometer Filter Project - Real-time Accelerometer Data Processing
    % Features:
    %   - Live time-domain plots (Raw vs Filtered) for Pitch & Roll
    %   - Real-time FFT spectrum analysis for both axes
    %   - 3D accelerometer visualization (realistic board model)
    %   - Filter performance metrics (SNR, RMS)
    %   - Data recording capability
    
    properties (Access = private)
        % UI Components
        UIFigure           matlab.ui.Figure
        GridLayout         matlab.ui.container.GridLayout
        
        % Control Panel
        ControlPanel       matlab.ui.container.Panel
        PortDropdown       matlab.ui.control.DropDown
        RefreshBtn         matlab.ui.control.Button
        ConnectBtn         matlab.ui.control.Button
        PauseBtn           matlab.ui.control.Button
        RecordBtn          matlab.ui.control.Button
        ClearBtn           matlab.ui.control.Button
        StatusLamp         matlab.ui.control.Lamp
        StatusLabel        matlab.ui.control.Label
        
        % Plot Axes
        TimePitchAxes      matlab.ui.control.UIAxes
        TimeRollAxes       matlab.ui.control.UIAxes
        FFTPitchAxes       matlab.ui.control.UIAxes
        FFTRollAxes        matlab.ui.control.UIAxes
        IMU3DAxes          matlab.ui.control.UIAxes
        
        % Metrics Display
        MetricsPanel       matlab.ui.container.Panel
        SNRLabel           matlab.ui.control.Label
        NRRLabel           matlab.ui.control.Label
        RMSRawLabel        matlab.ui.control.Label
        RMSFiltLabel       matlab.ui.control.Label
        SampleCountLabel   matlab.ui.control.Label
        PerfOrder2Label    matlab.ui.control.Label    % Performance for 2nd order
        PerfOrder4Label    matlab.ui.control.Label    % Performance for 4th order
        PerfOrder8Label    matlab.ui.control.Label    % Performance for 8th order
        
        % Serial Communication
        SerialObj          internal.Serialport
        IsStreaming        logical = false
        IsPaused           logical = false
        IsRecording        logical = false
        FilterOrder        double = 2            % Current filter order (2, 4, or 8)
        PerformanceData    struct               % Stores performance test results
        
        % Data Buffers
        BufferSize         double = 500      % 5 seconds at 100Hz
        TimeBuffer         double
        PitchRawBuffer     double
        PitchFiltBuffer    double
        RollRawBuffer      double
        RollFiltBuffer     double
        DataIndex          double = 0
        
        % 3D IMU Visualization Models
        IMUPatchRaw        matlab.graphics.primitive.Patch
        IMUTransformRaw    matlab.graphics.primitive.Transform
        IMUPatchFilt       matlab.graphics.primitive.Patch
        IMUTransformFilt   matlab.graphics.primitive.Transform
        
        % Recording
        RecordedData       struct
        RecordStartTime    datetime
    end
    
    methods (Access = public)
        function app = DSP_Dashboard
            initializeBuffers(app);
            createComponents(app);
            refreshPortList(app);
        end
        
        function delete(app)
            stopStreaming(app);
            if isvalid(app.UIFigure)
                delete(app.UIFigure);
            end
        end
    end
    
    methods (Access = private)
        
        function initializeBuffers(app)
            app.TimeBuffer = zeros(app.BufferSize, 1);
            app.PitchRawBuffer = zeros(app.BufferSize, 1);
            app.PitchFiltBuffer = zeros(app.BufferSize, 1);
            app.RollRawBuffer = zeros(app.BufferSize, 1);
            app.RollFiltBuffer = zeros(app.BufferSize, 1);
            app.DataIndex = 0;
            
            % Initialize performance data structure
            app.PerformanceData = struct();
            app.PerformanceData.Order2 = struct('Cycles', 0, 'TimeUs', 0);
            app.PerformanceData.Order4 = struct('Cycles', 0, 'TimeUs', 0);
            app.PerformanceData.Order8 = struct('Cycles', 0, 'TimeUs', 0);
        end
        
        function refreshPortList(app, ~, ~)
            ports = serialportlist("available");
            if isempty(ports)
                app.PortDropdown.Items = ["No Device Found"];
                app.PortDropdown.Value = "No Device Found";
                app.ConnectBtn.Enable = 'off';
            else
                app.PortDropdown.Items = ports;
                app.PortDropdown.Value = ports(1);
                app.ConnectBtn.Enable = 'on';
            end
        end
        
        function onToggleConnection(app, ~, ~)
            if app.IsStreaming
                stopStreaming(app);
            else
                startStreaming(app);
            end
        end
        
        function onTogglePause(app, ~, ~)
            app.IsPaused = ~app.IsPaused;
            if app.IsPaused
                app.PauseBtn.Text = '▶ Resume';
                app.PauseBtn.BackgroundColor = [1 0.8 0];
                app.StatusLabel.Text = 'Paused';
            else
                app.PauseBtn.Text = '⏸ Pause';
                app.PauseBtn.BackgroundColor = [0.5 0.5 0.5];
                app.StatusLabel.Text = 'Streaming';
            end
        end
        
        function onToggleRecord(app, ~, ~)
            app.IsRecording = ~app.IsRecording;
            if app.IsRecording
                app.RecordStartTime = datetime('now');
                app.RecordedData = struct('Time', [], 'PitchRaw', [], 'PitchFiltered', [], ...
                                         'RollRaw', [], 'RollFiltered', []);
                app.RecordBtn.Text = '⏹ Stop Recording';
                app.RecordBtn.BackgroundColor = [1 0 0];
            else
                app.RecordBtn.Text = '⏺ Record';
                app.RecordBtn.BackgroundColor = [0.7 0.7 0.7];
                saveRecordedData(app);
            end
        end
        
        function onClearPlots(app, ~, ~)
            initializeBuffers(app);
            cla(app.TimePitchAxes);
            cla(app.TimeRollAxes);
            cla(app.FFTPitchAxes);
            cla(app.FFTRollAxes);
            % Don't clear 3D model - just reset to neutral position
            if ~isempty(app.IMUTransformFilt) && isvalid(app.IMUTransformFilt)
                set(app.IMUTransformFilt, 'Matrix', eye(4));
            end
            if ~isempty(app.IMUTransformRaw) && isvalid(app.IMUTransformRaw)
                set(app.IMUTransformRaw, 'Matrix', eye(4));
            end
            setupPlots(app);
            updateMetrics(app);
        end
        
        function startStreaming(app)
            portName = app.PortDropdown.Value;
            
            if strcmp(portName, "No Device Found")
                uialert(app.UIFigure, 'No serial port available!', 'Error');
                return;
            end
            
            try
                app.SerialObj = serialport(portName, 115200);
                configureTerminator(app.SerialObj, "LF");
                flush(app.SerialObj);
                
                app.IsStreaming = true;
                app.IsPaused = false;
                app.ConnectBtn.Text = '🔌 Disconnect';
                app.ConnectBtn.BackgroundColor = [0.8 0.3 0.3];
                app.StatusLamp.Color = 'green';
                app.StatusLabel.Text = 'Connected & Streaming';
                app.PortDropdown.Enable = 'off';
                app.RefreshBtn.Enable = 'off';
                app.PauseBtn.Enable = 'on';
                app.RecordBtn.Enable = 'on';
                app.ClearBtn.Enable = 'on';
                
                setupPlots(app);
                readDataLoop(app);
                
            catch ME
                uialert(app.UIFigure, "Connection Failed: " + ME.message, 'Error');
                stopStreaming(app);
            end
        end
        
        function stopStreaming(app)
            app.IsStreaming = false;
            app.IsPaused = false;
            
            if ~isempty(app.SerialObj) && isvalid(app.SerialObj)
                delete(app.SerialObj);
            end
            
            if isvalid(app.UIFigure)
                app.ConnectBtn.Text = '🔌 Connect';
                app.ConnectBtn.BackgroundColor = [0.3 0.8 0.3];
                app.StatusLamp.Color = 'red';
                app.StatusLabel.Text = 'Disconnected';
                app.PortDropdown.Enable = 'on';
                app.RefreshBtn.Enable = 'on';
                app.PauseBtn.Enable = 'off';
                app.PauseBtn.Text = '⏸ Pause';
                app.RecordBtn.Enable = 'off';
                
                if app.IsRecording
                    app.IsRecording = false;
                    saveRecordedData(app);
                end
            end
        end
        
        function readDataLoop(app)
            % Flush any old data in buffer at start
            if app.SerialObj.NumBytesAvailable > 0
                flush(app.SerialObj);
            end
            
            while app.IsStreaming && isvalid(app.UIFigure)
                try
                    % Process ALL available data to prevent buffer buildup
                    while app.SerialObj.NumBytesAvailable > 0
                        line = readline(app.SerialObj);
                        
                        % Split by carriage return or newline characters
                        sublines = regexp(char(line), '[\r\n]+', 'split');
                        
                        for i = 1:length(sublines)
                            subline = strtrim(sublines{i});
                            if isempty(subline)
                                continue; % Skip empty lines
                            end
                            
                            parts = split(string(subline), ',');
                        
                        % Check for filter change notification: "FILTER_CHANGE,<new_order>"
                        if length(parts) >= 2 && parts(1) == "FILTER_CHANGE"
                            new_order = str2double(parts(2));
                            if ~isnan(new_order)
                                fprintf('Filter order changed to: %d - Clearing buffers\n', new_order);
                                app.FilterOrder = new_order;
                                % Clear all data buffers
                                app.TimeBuffer(:) = 0;
                                app.PitchRawBuffer(:) = 0;
                                app.PitchFiltBuffer(:) = 0;
                                app.RollRawBuffer(:) = 0;
                                app.RollFiltBuffer(:) = 0;
                                app.DataIndex = 0;
                            end
                            continue;
                        end
                        
                        % Check for performance test result: "PERF,<order>,<cycles>,<time_us>"
                        if length(parts) >= 4 && parts(1) == "PERF"
                            order = str2double(parts(2));
                            cycles = str2double(parts(3));
                            time_us = str2double(parts(4));
                            
                            if ~isnan(order) && ~isnan(cycles) && ~isnan(time_us)
                                fprintf('Performance Test - Order: %d, Cycles: %d, Time: %.2f µs\n', order, cycles, time_us);
                                app.PerformanceData.(sprintf('Order%d', order)).Cycles = cycles;
                                app.PerformanceData.(sprintf('Order%d', order)).TimeUs = time_us;
                                
                                % Update corresponding performance label
                                if order == 2
                                    app.PerfOrder2Label.Text = sprintf('2nd: %d cy / %.2f µs', cycles, time_us);
                                    app.PerfOrder2Label.FontColor = [0, 0.6, 0]; % Green
                                elseif order == 4
                                    app.PerfOrder4Label.Text = sprintf('4th: %d cy / %.2f µs', cycles, time_us);
                                    app.PerfOrder4Label.FontColor = [0, 0.6, 0]; % Green
                                elseif order == 8
                                    app.PerfOrder8Label.Text = sprintf('8th: %d cy / %.2f µs', cycles, time_us);
                                    app.PerfOrder8Label.FontColor = [0, 0.6, 0]; % Green
                                end
                                drawnow; % Force UI update
                            end
                            continue;
                        end
                        
                        % Protocol: "S,<order>,<pitch_raw>,<pitch_filt>,<roll_raw>,<roll_filt>,E"
                        if length(parts) >= 7 && parts(1) == "S"
                            filter_order = str2double(parts(2));
                            pitch_raw = str2double(parts(3));
                            pitch_filt = str2double(parts(4));
                            roll_raw = str2double(parts(5));
                            roll_filt = str2double(parts(6));
                            
                            % Update filter order if changed
                            if ~isnan(filter_order)
                                app.FilterOrder = filter_order;
                            end
                            
                            if ~app.IsPaused && ~isnan(pitch_raw) && ~isnan(pitch_filt) && ~isnan(roll_raw) && ~isnan(roll_filt)
                                app.DataIndex = app.DataIndex + 1;
                                idx = mod(app.DataIndex - 1, app.BufferSize) + 1;
                                
                                app.TimeBuffer(idx) = app.DataIndex / 100;
                                app.PitchRawBuffer(idx) = pitch_raw;
                                app.PitchFiltBuffer(idx) = pitch_filt;
                                app.RollRawBuffer(idx) = roll_raw;
                                app.RollFiltBuffer(idx) = roll_filt;
                                
                                if app.IsRecording
                                    app.RecordedData.Time(end+1) = app.TimeBuffer(idx);
                                    app.RecordedData.PitchRaw(end+1) = pitch_raw;
                                    app.RecordedData.PitchFiltered(end+1) = pitch_filt;
                                    app.RecordedData.RollRaw(end+1) = roll_raw;
                                    app.RecordedData.RollFiltered(end+1) = roll_filt;
                                end
                                
                                % Update 3D model at full rate (100Hz)
                                update3DModel(app, pitch_filt, roll_filt);
                            end
                        end
                        end % End of for loop over sublines
                    end % End of while NumBytesAvailable > 0
                    
                    % Update plots and metrics after processing all available data
                    % High-performance mode: update every 4 samples (25Hz for smooth visualization)
                    if mod(app.DataIndex, 4) == 0
                        updatePlots(app);
                    end
                    
                    % Update metrics less frequently (every 20 samples = 5Hz)
                    if mod(app.DataIndex, 20) == 0
                        updateMetrics(app);
                    end
                    
                    % No pause - let MATLAB handle timing
                    drawnow;  % Force immediate graphics update
                catch ME
                    stopStreaming(app);
                    uialert(app.UIFigure, "Connection Lost: " + ME.message, "Error");
                    return;
                end
            end
        end
        
        function setupPlots(app)
            % Pitch time domain plot
            cla(app.TimePitchAxes);
            hold(app.TimePitchAxes, 'on');
            plot(app.TimePitchAxes, nan, nan, 'r-', 'LineWidth', 1.2, 'Tag', 'PitchRawLine');
            plot(app.TimePitchAxes, nan, nan, 'g-', 'LineWidth', 2, 'Tag', 'PitchFiltLine');
            xlabel(app.TimePitchAxes, 'Time (s)');
            ylabel(app.TimePitchAxes, 'Pitch Angle (degrees)');
            title(app.TimePitchAxes, 'Pitch: Raw vs Filtered');
            legend(app.TimePitchAxes, 'Raw', 'Filtered', 'Location', 'northeast');
            grid(app.TimePitchAxes, 'on');
            hold(app.TimePitchAxes, 'off');
            
            % Roll time domain plot
            cla(app.TimeRollAxes);
            hold(app.TimeRollAxes, 'on');
            plot(app.TimeRollAxes, nan, nan, 'Color', [1 0.6 0], 'LineWidth', 1.2, 'Tag', 'RollRawLine');
            plot(app.TimeRollAxes, nan, nan, 'b-', 'LineWidth', 2, 'Tag', 'RollFiltLine');
            xlabel(app.TimeRollAxes, 'Time (s)');
            ylabel(app.TimeRollAxes, 'Roll Angle (degrees)');
            title(app.TimeRollAxes, 'Roll: Raw vs Filtered');
            legend(app.TimeRollAxes, 'Raw', 'Filtered', 'Location', 'northeast');
            grid(app.TimeRollAxes, 'on');
            hold(app.TimeRollAxes, 'off');
            
            % Pitch FFT plot
            cla(app.FFTPitchAxes);
            xlabel(app.FFTPitchAxes, 'Frequency (Hz)');
            ylabel(app.FFTPitchAxes, 'Magnitude (dB)');
            title(app.FFTPitchAxes, 'Pitch Spectrum');
            grid(app.FFTPitchAxes, 'on');
            xlim(app.FFTPitchAxes, [0 50]);
            
            % Roll FFT plot
            cla(app.FFTRollAxes);
            xlabel(app.FFTRollAxes, 'Frequency (Hz)');
            ylabel(app.FFTRollAxes, 'Magnitude (dB)');
            title(app.FFTRollAxes, 'Roll Spectrum');
            grid(app.FFTRollAxes, 'on');
            xlim(app.FFTRollAxes, [0 50]);
        end
        
        function updatePlots(app)
            validIdx = app.PitchRawBuffer ~= 0;
            if sum(validIdx) < 10, return; end
            
            % Get data in correct order (circular buffer) - optimized version
            if app.DataIndex <= app.BufferSize
                % Buffer not full yet - use direct slicing (faster)
                idx_end = app.DataIndex;
                timeData = app.TimeBuffer(1:idx_end);
                pitchRawData = app.PitchRawBuffer(1:idx_end);
                pitchFiltData = app.PitchFiltBuffer(1:idx_end);
                rollRawData = app.RollRawBuffer(1:idx_end);
                rollFiltData = app.RollFiltBuffer(1:idx_end);
            else
                % Buffer full - use circular indexing (avoid concatenation)
                idx = mod(app.DataIndex - 1, app.BufferSize) + 1;
                
                % Pre-allocate for speed
                timeData = zeros(app.BufferSize, 1);
                pitchRawData = zeros(app.BufferSize, 1);
                pitchFiltData = zeros(app.BufferSize, 1);
                rollRawData = zeros(app.BufferSize, 1);
                rollFiltData = zeros(app.BufferSize, 1);
                
                % Copy in two chunks (faster than concatenation)
                n1 = app.BufferSize - idx;
                timeData(1:n1) = app.TimeBuffer(idx+1:end);
                timeData(n1+1:end) = app.TimeBuffer(1:idx);
                
                pitchRawData(1:n1) = app.PitchRawBuffer(idx+1:end);
                pitchRawData(n1+1:end) = app.PitchRawBuffer(1:idx);
                
                pitchFiltData(1:n1) = app.PitchFiltBuffer(idx+1:end);
                pitchFiltData(n1+1:end) = app.PitchFiltBuffer(1:idx);
                
                rollRawData(1:n1) = app.RollRawBuffer(idx+1:end);
                rollRawData(n1+1:end) = app.RollRawBuffer(1:idx);
                
                rollFiltData(1:n1) = app.RollFiltBuffer(idx+1:end);
                rollFiltData(n1+1:end) = app.RollFiltBuffer(1:idx);
            end
            
            % Update time plots - cache line handles for speed
            persistent pitchRawLine pitchFiltLine rollRawLine rollFiltLine;
            if isempty(pitchRawLine) || ~isvalid(pitchRawLine)
                pitchRawLine = findobj(app.TimePitchAxes, 'Tag', 'PitchRawLine');
                pitchFiltLine = findobj(app.TimePitchAxes, 'Tag', 'PitchFiltLine');
                rollRawLine = findobj(app.TimeRollAxes, 'Tag', 'RollRawLine');
                rollFiltLine = findobj(app.TimeRollAxes, 'Tag', 'RollFiltLine');
            end
            
            % Batch updates (single set call is faster than multiple)
            set(pitchRawLine, 'XData', timeData, 'YData', pitchRawData);
            set(pitchFiltLine, 'XData', timeData, 'YData', pitchFiltData);
            set(rollRawLine, 'XData', timeData, 'YData', rollRawData);
            set(rollFiltLine, 'XData', timeData, 'YData', rollFiltData);
            
            % Update axis limits only if significantly changed (reduce redraws)
            current_xlim = xlim(app.TimePitchAxes);
            new_xlim = [max(0, timeData(end)-5), timeData(end)+0.5];
            if abs(current_xlim(2) - new_xlim(2)) > 0.1
                xlim(app.TimePitchAxes, new_xlim);
                xlim(app.TimeRollAxes, new_xlim);
            end
            
            % Update FFT every 5th plot update (5Hz) - balanced for performance
            persistent fftUpdateCounter;
            if isempty(fftUpdateCounter), fftUpdateCounter = 0; end
            fftUpdateCounter = fftUpdateCounter + 1;
            
            if fftUpdateCounter < 5
                return;  % Skip FFT update this time
            end
            fftUpdateCounter = 0;  % Reset counter
            
            % Update FFT (separate plots for Pitch and Roll)
            N = length(pitchRawData);
            if N > 128
                Fs = 100;
                
                % Optimized FFT size for high-performance systems
                NFFT = min(2^nextpow2(N), 4096);  % Increased cap for better resolution
                
                % Apply Hamming window to reduce spectral leakage (reuse window)
                window = hamming(N);
                
                % Vectorized mean removal and windowing (faster)
                pitchRawWindowed = (pitchRawData - mean(pitchRawData)) .* window;
                pitchFiltWindowed = (pitchFiltData - mean(pitchFiltData)) .* window;
                rollRawWindowed = (rollRawData - mean(rollRawData)) .* window;
                rollFiltWindowed = (rollFiltData - mean(rollFiltData)) .* window;
                
                % Compute all FFTs (MATLAB optimizes batch operations)
                pitchRawFFT = abs(fft(pitchRawWindowed, NFFT));
                pitchFiltFFT = abs(fft(pitchFiltWindowed, NFFT));
                rollRawFFT = abs(fft(rollRawWindowed, NFFT));
                rollFiltFFT = abs(fft(rollFiltWindowed, NFFT));
                
                % Frequency axis
                freqs = (0:NFFT-1)*(Fs/NFFT);
                halfIdx = 1:floor(NFFT/2);
                
                % Convert to dB with vectorized operations
                pitchRawFFT_dB = 20*log10(pitchRawFFT(halfIdx) / max(pitchRawFFT(halfIdx)) + eps);
                pitchFiltFFT_dB = 20*log10(pitchFiltFFT(halfIdx) / max(pitchFiltFFT(halfIdx)) + eps);
                rollRawFFT_dB = 20*log10(rollRawFFT(halfIdx) / max(rollRawFFT(halfIdx)) + eps);
                rollFiltFFT_dB = 20*log10(rollFiltFFT(halfIdx) / max(rollFiltFFT(halfIdx)) + eps);
                
                % Minimal smoothing for faster computation (2-point)
                pitchRawFFT_dB = movmean(pitchRawFFT_dB, 2);
                pitchFiltFFT_dB = movmean(pitchFiltFFT_dB, 2);
                rollRawFFT_dB = movmean(rollRawFFT_dB, 2);
                rollFiltFFT_dB = movmean(rollFiltFFT_dB, 2);
                
                % Update Pitch FFT - cache line handles and check size
                persistent pitchFFTRawLine pitchFFTFiltLine rollFFTRawLine rollFFTFiltLine lastFFTSize;
                
                % Check if FFT size changed (need to recreate plots)
                currentFFTSize = length(halfIdx);
                if isempty(lastFFTSize) || lastFFTSize ~= currentFFTSize
                    lastFFTSize = currentFFTSize;
                    % Force recreation of plots
                    pitchFFTRawLine = [];
                    pitchFFTFiltLine = [];
                    rollFFTRawLine = [];
                    rollFFTFiltLine = [];
                end
                
                if isempty(pitchFFTRawLine) || ~isvalid(pitchFFTRawLine)
                    % First time or size changed: create lines with optimized properties
                    cla(app.FFTPitchAxes);
                    hold(app.FFTPitchAxes, 'on');
                    pitchFFTRawLine = plot(app.FFTPitchAxes, freqs(halfIdx), pitchRawFFT_dB, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Raw', 'Tag', 'PitchFFTRaw');
                    pitchFFTFiltLine = plot(app.FFTPitchAxes, freqs(halfIdx), pitchFiltFFT_dB, 'g-', 'LineWidth', 2, 'DisplayName', 'Filtered', 'Tag', 'PitchFFTFilt');
                    xline(app.FFTPitchAxes, 5, '--k', 'Fc', 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left');
                    yline(app.FFTPitchAxes, -3, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1, 'Alpha', 0.5);
                    hold(app.FFTPitchAxes, 'off');
                    legend(app.FFTPitchAxes, 'Location', 'northeast');
                    xlim(app.FFTPitchAxes, [0 50]);
                    ylim(app.FFTPitchAxes, [-60 5]);
                    grid(app.FFTPitchAxes, 'on');
                else
                    % Fast update using cached handles (both X and Y for size safety)
                    set(pitchFFTRawLine, 'XData', freqs(halfIdx), 'YData', pitchRawFFT_dB);
                    set(pitchFFTFiltLine, 'XData', freqs(halfIdx), 'YData', pitchFiltFFT_dB);
                end
                
                % Update Roll FFT - use cached handles
                if isempty(rollFFTRawLine) || ~isvalid(rollFFTRawLine)
                    % First time or size changed: create lines with optimized properties
                    cla(app.FFTRollAxes);
                    hold(app.FFTRollAxes, 'on');
                    rollFFTRawLine = plot(app.FFTRollAxes, freqs(halfIdx), rollRawFFT_dB, 'Color', [1 0.6 0], 'LineWidth', 1.5, 'DisplayName', 'Raw', 'Tag', 'RollFFTRaw');
                    rollFFTFiltLine = plot(app.FFTRollAxes, freqs(halfIdx), rollFiltFFT_dB, 'b-', 'LineWidth', 2, 'DisplayName', 'Filtered', 'Tag', 'RollFFTFilt');
                    xline(app.FFTRollAxes, 5, '--k', 'Fc', 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left');
                    yline(app.FFTRollAxes, -3, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1, 'Alpha', 0.5);
                    hold(app.FFTRollAxes, 'off');
                    legend(app.FFTRollAxes, 'Location', 'northeast');
                    xlim(app.FFTRollAxes, [0 50]);
                    ylim(app.FFTRollAxes, [-60 5]);
                    grid(app.FFTRollAxes, 'on');
                else
                    % Fast update using cached handles (both X and Y for size safety)
                    set(rollFFTRawLine, 'XData', freqs(halfIdx), 'YData', rollRawFFT_dB);
                    set(rollFFTFiltLine, 'XData', freqs(halfIdx), 'YData', rollFiltFFT_dB);
                end
            end
        end
        
        function updateMetrics(app)
            validIdx = app.PitchRawBuffer ~= 0;
            if sum(validIdx) < 10, return; end
            
            pitchRawData = app.PitchRawBuffer(validIdx);
            pitchFiltData = app.PitchFiltBuffer(validIdx);
            rollRawData = app.RollRawBuffer(validIdx);
            rollFiltData = app.RollFiltBuffer(validIdx);
            
            % Calculate metrics for pitch
            rms_pitch_raw = rms(pitchRawData);
            rms_pitch_filt = rms(pitchFiltData);
            % SNR: Signal power vs noise power
            signal_power_pitch = mean(pitchRawData.^2);
            noise_power_pitch = mean((pitchRawData - pitchFiltData).^2);
            snr_pitch = 10*log10(signal_power_pitch / (noise_power_pitch + eps));
            % Noise Reduction Ratio (NRR): Compare raw vs filtered signal variance
            raw_variance_pitch = var(pitchRawData);
            filt_variance_pitch = var(pitchFiltData);
            noise_reduction_pitch = 10*log10(raw_variance_pitch / (filt_variance_pitch + eps));
            
            % Calculate metrics for roll
            rms_roll_raw = rms(rollRawData);
            rms_roll_filt = rms(rollFiltData);
            % SNR: Signal power vs noise power
            signal_power_roll = mean(rollRawData.^2);
            noise_power_roll = mean((rollRawData - rollFiltData).^2);
            snr_roll = 10*log10(signal_power_roll / (noise_power_roll + eps));
            % Noise Reduction Ratio (NRR): Compare raw vs filtered signal variance
            raw_variance_roll = var(rollRawData);
            filt_variance_roll = var(rollFiltData);
            noise_reduction_roll = 10*log10(raw_variance_roll / (filt_variance_roll + eps));
            
            app.RMSRawLabel.Text = sprintf('RMS Raw:  Pitch %.2f°  |  Roll %.2f°', rms_pitch_raw, rms_roll_raw);
            app.RMSFiltLabel.Text = sprintf('RMS Filtered:  Pitch %.2f°  |  Roll %.2f°', rms_pitch_filt, rms_roll_filt);
            app.SNRLabel.Text = sprintf('SNR:  Pitch %.1f dB  |  Roll %.1f dB', snr_pitch, snr_roll);
            app.NRRLabel.Text = sprintf('NRR:  Pitch %.1f dB  |  Roll %.1f dB', noise_reduction_pitch, noise_reduction_roll);
            
            % Update sample count with filter order inline
            orderSuffix = {'2nd', '4th', '8th'};
            orderIdx = find([2, 4, 8] == app.FilterOrder, 1);
            if ~isempty(orderIdx)
                app.SampleCountLabel.Text = sprintf('Filter: %s Order IIR Butterworth | Samples: %d (%.1f sec)', ...
                    orderSuffix{orderIdx}, app.DataIndex, app.DataIndex/100);
            end
            
            % Highlight currently active filter order
            if app.FilterOrder == 2
                app.PerfOrder2Label.FontWeight = 'bold';
                app.PerfOrder4Label.FontWeight = 'normal';
                app.PerfOrder8Label.FontWeight = 'normal';
            elseif app.FilterOrder == 4
                app.PerfOrder2Label.FontWeight = 'normal';
                app.PerfOrder4Label.FontWeight = 'bold';
                app.PerfOrder8Label.FontWeight = 'normal';
            elseif app.FilterOrder == 8
                app.PerfOrder2Label.FontWeight = 'normal';
                app.PerfOrder4Label.FontWeight = 'normal';
                app.PerfOrder8Label.FontWeight = 'bold';
            end
        end
        
        function update3DModel(app, pitch_angle, roll_angle)
            if isempty(app.IMUTransformRaw) || ~isvalid(app.IMUTransformRaw)
                return;
            end
            
            % Both models at same position (overlaid) for easy comparison
            % Order: Roll (X-axis) then Pitch (Y-axis) rotation
            
            % Filtered model (smooth, opaque green) - Apply filtered angles
            R_filt = makehgtform('xrotate', roll_angle * pi/180) * ...
                     makehgtform('yrotate', pitch_angle * pi/180);
            set(app.IMUTransformFilt, 'Matrix', R_filt);
            
            % Raw model (jittery, transparent red) - Apply raw angles from buffers
            % Get most recent raw values
            if app.DataIndex > 0
                idx = mod(app.DataIndex - 1, app.BufferSize) + 1;
                pitch_raw_current = app.PitchRawBuffer(idx);
                roll_raw_current = app.RollRawBuffer(idx);
                
                R_raw = makehgtform('xrotate', roll_raw_current * pi/180) * ...
                        makehgtform('yrotate', pitch_raw_current * pi/180);
                set(app.IMUTransformRaw, 'Matrix', R_raw);
            end
        end
        
        function saveRecordedData(app)
            if isempty(app.RecordedData) || isempty(app.RecordedData.Time)
                return;
            end
            
            filename = sprintf('Accelerometer_Recording_%s.mat', ...
                datestr(app.RecordStartTime, 'yyyy-mm-dd_HH-MM-SS'));
            
            data = app.RecordedData;
            save(filename, 'data');
            
            uialert(app.UIFigure, sprintf('Data saved to: %s', filename), ...
                'Recording Saved', 'Icon', 'success');
        end
        
        function onWindowResize(app, ~, ~)
            % Force refresh of all axes when window is resized
            drawnow;
            
            % Ensure normalized positions are maintained
            if isvalid(app.TimePitchAxes)
                app.TimePitchAxes.Position = [0.01 0.01 0.98 0.98];
            end
            if isvalid(app.TimeRollAxes)
                app.TimeRollAxes.Position = [0.01 0.01 0.98 0.98];
            end
            if isvalid(app.IMU3DAxes)
                app.IMU3DAxes.Position = [0.01 0.01 0.98 0.98];
            end
            if isvalid(app.FFTPitchAxes)
                app.FFTPitchAxes.Position = [0.01 0.01 0.98 0.98];
            end
            if isvalid(app.FFTRollAxes)
                app.FFTRollAxes.Position = [0.01 0.01 0.98 0.98];
            end
        end
        
        function createComponents(app)
            % Main Window
            app.UIFigure = uifigure('Name', 'Dual-Axis Digital Filter Analyzer - STM32 Accelerometer (Pitch & Roll)', ...
                'Position', [50 50 1400 900], 'Color', [0.94 0.94 0.94], ...
                'AutoResizeChildren', 'off', ...
                'SizeChangedFcn', createCallbackFcn(app, @onWindowResize, true));
            
            % Main Grid Layout
            app.GridLayout = uigridlayout(app.UIFigure, [3, 2]);
            app.GridLayout.RowHeight = {60, '3x', '2x'};
            app.GridLayout.ColumnWidth = {'2x', '2x'};
            app.GridLayout.Padding = [10 10 10 10];
            app.GridLayout.RowSpacing = 10;
            app.GridLayout.ColumnSpacing = 10;
            
            % === CONTROL PANEL (Top, spans both columns) ===
            app.ControlPanel = uipanel(app.GridLayout, ...
                'Title', '⚙ Control Panel', ...
                'FontWeight', 'bold', ...
                'BackgroundColor', 'white');
            app.ControlPanel.Layout.Row = 1;
            app.ControlPanel.Layout.Column = [1 2];
            
            controlGrid = uigridlayout(app.ControlPanel, [1, 8]);
            controlGrid.ColumnWidth = {80, 150, 100, 110, 90, 90, 70, '1x'};
            controlGrid.Padding = [10 5 10 5];
            
            % Port selection
            uilabel(controlGrid, 'Text', 'Serial Port:', 'FontWeight', 'bold');
            app.PortDropdown = uidropdown(controlGrid);
            
            app.RefreshBtn = uibutton(controlGrid, 'Text', '🔄 Refresh', ...
                'ButtonPushedFcn', createCallbackFcn(app, @refreshPortList, true));
            
            app.ConnectBtn = uibutton(controlGrid, 'Text', '🔌 Connect', ...
                'FontWeight', 'bold', 'BackgroundColor', [0.3 0.8 0.3], ...
                'ButtonPushedFcn', createCallbackFcn(app, @onToggleConnection, true));
            
            app.PauseBtn = uibutton(controlGrid, 'Text', '⏸ Pause', ...
                'Enable', 'off', 'BackgroundColor', [0.5 0.5 0.5], ...
                'ButtonPushedFcn', createCallbackFcn(app, @onTogglePause, true));
            
            app.RecordBtn = uibutton(controlGrid, 'Text', '⏺ Record', ...
                'Enable', 'off', 'BackgroundColor', [0.7 0.7 0.7], ...
                'ButtonPushedFcn', createCallbackFcn(app, @onToggleRecord, true));
            
            app.ClearBtn = uibutton(controlGrid, 'Text', '🗑 Clear', ...
                'Enable', 'off', 'BackgroundColor', [0.9 0.9 0.9], ...
                'ButtonPushedFcn', createCallbackFcn(app, @onClearPlots, true));
            
            statusGrid = uigridlayout(controlGrid, [1, 2]);
            statusGrid.ColumnWidth = {25, '1x'};
            statusGrid.Padding = [0 0 0 0];
            
            app.StatusLamp = uilamp(statusGrid, 'Color', 'red');
            app.StatusLabel = uilabel(statusGrid, 'Text', 'Disconnected', 'FontWeight', 'bold');
            
            % === TIME DOMAIN PLOTS (Left, Middle - split vertically) ===
            timePanel = uipanel(app.GridLayout, 'Title', '📈 Time Domain Analysis (Pitch & Roll)', ...
                'FontWeight', 'bold', 'BackgroundColor', 'white');
            timePanel.Layout.Row = 2;
            timePanel.Layout.Column = 1;
            
            % Create a subgrid to split into two rows
            timeGrid = uigridlayout(timePanel, [2, 1]);
            timeGrid.RowHeight = {'1x', '1x'};
            timeGrid.Padding = [5 5 5 5];
            timeGrid.RowSpacing = 10;
            
            % Pitch time plot (top half)
            pitchTimeSubPanel = uipanel(timeGrid, 'BorderType', 'none', 'BackgroundColor', 'white');
            pitchTimeSubPanel.Layout.Row = 1;
            pitchTimeSubPanel.Layout.Column = 1;
            app.TimePitchAxes = uiaxes(pitchTimeSubPanel);
            app.TimePitchAxes.Units = 'normalized';
            app.TimePitchAxes.Position = [0.01 0.01 0.98 0.98];  % Maximum coverage
            
            % Roll time plot (bottom half)
            rollTimeSubPanel = uipanel(timeGrid, 'BorderType', 'none', 'BackgroundColor', 'white');
            rollTimeSubPanel.Layout.Row = 2;
            rollTimeSubPanel.Layout.Column = 1;
            app.TimeRollAxes = uiaxes(rollTimeSubPanel);
            app.TimeRollAxes.Units = 'normalized';
            app.TimeRollAxes.Position = [0.01 0.01 0.98 0.98];  % Maximum coverage
            
            % === 3D ACCELEROMETER VISUALIZATION (Right, Middle) ===
            imuPanel = uipanel(app.GridLayout, 'Title', '🎯 3D Overlay Comparison (Raw vs Filtered)', ...
                'FontWeight', 'bold', 'BackgroundColor', 'white');
            imuPanel.Layout.Row = 2;
            imuPanel.Layout.Column = 2;
            
            % Center and maximize 3D axes within panel (use normalized units)
            app.IMU3DAxes = uiaxes(imuPanel);
            app.IMU3DAxes.Units = 'normalized';  % Set units FIRST
            app.IMU3DAxes.Position = [0.01 0.01 0.98 0.98];  % Maximize: 1% margin, 98% coverage
            
            % Setup 3D scene for overlaid comparison
            view(app.IMU3DAxes, [135 20]);  % Better viewing angle for overlay
            axis(app.IMU3DAxes, 'vis3d');
            grid(app.IMU3DAxes, 'on');
            xlabel(app.IMU3DAxes, 'X (Forward)');
            ylabel(app.IMU3DAxes, 'Y (Right)');
            zlabel(app.IMU3DAxes, 'Z (Up)');
            xlim(app.IMU3DAxes, [-4 4]);
            ylim(app.IMU3DAxes, [-4 4]);
            zlim(app.IMU3DAxes, [-2 3]);
            
            % Create FILTERED airplane model (green, solid) - FIRST (behind)
            app.IMUTransformFilt = hgtransform('Parent', app.IMU3DAxes);
            createAirplaneModel(app, app.IMUTransformFilt, [0.2 0.85 0.2], 0.85, 'g', 0);
            
            % Create RAW airplane model (red, transparent) - SECOND (in front for visibility)
            app.IMUTransformRaw = hgtransform('Parent', app.IMU3DAxes);
            createAirplaneModel(app, app.IMUTransformRaw, [0.95 0.2 0.2], 0.35, 'r', 0);
            
            % Store patch references for legend
            app.IMUPatchRaw = findobj(app.IMUTransformRaw, 'Type', 'patch', '-depth', 1);
            app.IMUPatchFilt = findobj(app.IMUTransformFilt, 'Type', 'patch', '-depth', 1);
            
            % Add coordinate frame axes
            line(app.IMU3DAxes, [0 2], [0 0], [0 0], 'Color', 'r', 'LineWidth', 3); % X
            line(app.IMU3DAxes, [0 0], [0 1.5], [0 0], 'Color', 'g', 'LineWidth', 3); % Y
            line(app.IMU3DAxes, [0 0], [0 1], 'Color', 'b', 'LineWidth', 3); % Z
            text(app.IMU3DAxes, 2.2, 0, 0, 'X', 'Color', 'r', 'FontSize', 12, 'FontWeight', 'bold');
            text(app.IMU3DAxes, 0, 1.7, 0, 'Y', 'Color', 'g', 'FontSize', 12, 'FontWeight', 'bold');
            text(app.IMU3DAxes, 0, 0, 1.2, 'Z', 'Color', 'b', 'FontSize', 12, 'FontWeight', 'bold');
            
            % Add legend
            legend(app.IMU3DAxes, [app.IMUPatchFilt(1), app.IMUPatchRaw(1)], ...
                'Filtered (Smooth)', 'Raw (Noisy)', 'Location', 'northeast', 'Color', [1 1 1 0.8]);
            
            lighting(app.IMU3DAxes, 'gouraud');
            light(app.IMU3DAxes, 'Position', [3 3 5]);
            
            % === FFT SPECTRUM PLOTS (Bottom-left, split into 2 columns) ===
            fftPanel = uipanel(app.GridLayout, 'Title', '📊 Pitch Frequency Spectrum', ...
                'FontWeight', 'bold', 'BackgroundColor', 'white');
            fftPanel.Layout.Row = 3;
            fftPanel.Layout.Column = 1;
            
            % Create a subgrid to split the panel into two halves
            fftGrid = uigridlayout(fftPanel, [1, 2]);
            fftGrid.ColumnWidth = {'1x', '1x'};
            fftGrid.Padding = [5 5 5 5];
            fftGrid.ColumnSpacing = 10;
            
            % Pitch FFT axes (left half)
            pitchFftSubPanel = uipanel(fftGrid, 'BorderType', 'none', 'BackgroundColor', 'white');
            pitchFftSubPanel.Layout.Row = 1;
            pitchFftSubPanel.Layout.Column = 1;
            app.FFTPitchAxes = uiaxes(pitchFftSubPanel);
            app.FFTPitchAxes.Units = 'normalized';
            app.FFTPitchAxes.Position = [0.01 0.01 0.98 0.98];
            
            % Roll FFT axes (right half)
            rollFftSubPanel = uipanel(fftGrid, 'BorderType', 'none', 'BackgroundColor', 'white');
            rollFftSubPanel.Layout.Row = 1;
            rollFftSubPanel.Layout.Column = 2;
            app.FFTRollAxes = uiaxes(rollFftSubPanel);
            app.FFTRollAxes.Units = 'normalized';
            app.FFTRollAxes.Position = [0.01 0.01 0.98 0.98];
            
            % === METRICS PANEL (Right, Bottom) ===
            app.MetricsPanel = uipanel(app.GridLayout, 'Title', '📐 Filter Performance Metrics', ...
                'FontWeight', 'bold', 'BackgroundColor', 'white');
            app.MetricsPanel.Layout.Row = 3;
            app.MetricsPanel.Layout.Column = 2;
            
            metricsGrid = uigridlayout(app.MetricsPanel, [7, 2]);
            metricsGrid.RowHeight = {'1x', '1x', '1x', '1x', '1x', '0.6x', '0.6x'};
            metricsGrid.ColumnWidth = {'2x', '1.2x'};
            metricsGrid.Padding = [20 20 20 20];
            
            % Left Column - Main Metrics
            app.SampleCountLabel = uilabel(metricsGrid, 'Text', 'Filter: 2nd Order IIR Butterworth | Samples: 0', ...
                'FontSize', 14, 'FontWeight', 'bold');
            app.SampleCountLabel.Layout.Row = 1;
            app.SampleCountLabel.Layout.Column = 1;
            
            app.RMSRawLabel = uilabel(metricsGrid, 'Text', 'RMS Raw:  Pitch 0.00°  |  Roll 0.00°', ...
                'FontSize', 14, 'FontColor', [0.8 0.2 0.2]);
            app.RMSRawLabel.Layout.Row = 2;
            app.RMSRawLabel.Layout.Column = 1;
            
            app.RMSFiltLabel = uilabel(metricsGrid, 'Text', 'RMS Filtered:  Pitch 0.00°  |  Roll 0.00°', ...
                'FontSize', 14, 'FontColor', [0.2 0.8 0.2]);
            app.RMSFiltLabel.Layout.Row = 3;
            app.RMSFiltLabel.Layout.Column = 1;
            
            app.SNRLabel = uilabel(metricsGrid, 'Text', 'SNR:  Pitch 0.0 dB  |  Roll 0.0 dB', ...
                'FontSize', 14, 'FontWeight', 'bold', 'FontColor', [0.8 0.4 0.0]);
            app.SNRLabel.Layout.Row = 4;
            app.SNRLabel.Layout.Column = 1;
            
            app.NRRLabel = uilabel(metricsGrid, 'Text', 'NRR:  Pitch 0.0 dB  |  Roll 0.0 dB', ...
                'FontSize', 14, 'FontWeight', 'bold', 'FontColor', [0.2 0.4 0.8]);
            app.NRRLabel.Layout.Row = 5;
            app.NRRLabel.Layout.Column = 1;
            
            % Right Column - Performance Test Results
            perfTitle = uilabel(metricsGrid, 'Text', '⚡ Performance', ...
                'FontSize', 14, 'FontWeight', 'bold', 'FontColor', [0.3 0.3 0.7]);
            perfTitle.Layout.Row = 1;
            perfTitle.Layout.Column = 2;
            
            app.PerfOrder2Label = uilabel(metricsGrid, ...
                'Text', '2nd: --- cy / --- µs', ...
                'FontSize', 13, 'FontColor', [0.3 0.3 0.3]);
            app.PerfOrder2Label.Layout.Row = 2;
            app.PerfOrder2Label.Layout.Column = 2;
            
            app.PerfOrder4Label = uilabel(metricsGrid, ...
                'Text', '4th: --- cy / --- µs', ...
                'FontSize', 13, 'FontColor', [0.3 0.3 0.3]);
            app.PerfOrder4Label.Layout.Row = 3;
            app.PerfOrder4Label.Layout.Column = 2;
            
            app.PerfOrder8Label = uilabel(metricsGrid, ...
                'Text', '8th: --- cy / --- µs', ...
                'FontSize', 13, 'FontColor', [0.3 0.3 0.3]);
            app.PerfOrder8Label.Layout.Row = 4;
            app.PerfOrder8Label.Layout.Column = 2;
            
            % SNR & NRR Interpretation Guide (spanning both columns at bottom)
            % Row 6 - Explanations
            snrExplainLabel = uilabel(metricsGrid, ...
                'Text', 'SNR: Signal quality (higher = cleaner)', ...
                'FontSize', 12, 'FontColor', [0.5 0.5 0.5], 'FontAngle', 'italic');
            snrExplainLabel.Layout.Row = 6;
            snrExplainLabel.Layout.Column = 1;
            
            nrrExplainLabel = uilabel(metricsGrid, ...
                'Text', 'NRR: Noise reduction effectiveness', ...
                'FontSize', 12, 'FontColor', [0.5 0.5 0.5], 'FontAngle', 'italic');
            nrrExplainLabel.Layout.Row = 6;
            nrrExplainLabel.Layout.Column = 2;
            
            % Row 7 - Value ranges
            snrRangeLabel = uilabel(metricsGrid, ...
                'Text', '<10dB Poor | 10-20dB Fair | >20dB Good', ...
                'FontSize', 11, 'FontColor', [0.6 0.6 0.6], 'FontAngle', 'italic');
            snrRangeLabel.Layout.Row = 7;
            snrRangeLabel.Layout.Column = 1;
            
            nrrRangeLabel = uilabel(metricsGrid, ...
                'Text', '<5dB Minimal | 5-15dB Moderate | >15dB Excellent', ...
                'FontSize', 11, 'FontColor', [0.6 0.6 0.6], 'FontAngle', 'italic');
            nrrRangeLabel.Layout.Row = 7;
            nrrRangeLabel.Layout.Column = 2;
        end
        
        % Helper function to create airplane model
        function createAirplaneModel(app, parent, faceColor, faceAlpha, edgeColor, xOffset)
            % Create a clean, realistic airplane model
            
            % FUSELAGE - Main body
            fuseX = [-2 -1.5 -0.5 0.5 1.2 1.8] + xOffset;
            fuseY = [0 0 0 0 0 0];
            fuseZ = [0 0.15 0.2 0.2 0.15 0.05];
            fuseWidth = [0.15 0.25 0.3 0.3 0.2 0.08];
            
            for i = 1:length(fuseX)-1
                % Create elliptical cross-sections
                theta = linspace(0, 2*pi, 20);
                for j = 1:2
                    idx = i + j - 1;
                    y = fuseWidth(idx) * cos(theta);
                    z = fuseZ(idx) + fuseWidth(idx) * 0.7 * sin(theta);
                    x = ones(size(theta)) * fuseX(idx);
                    
                    if j == 1
                        fuseY1 = y; fuseZ1 = z; fuseX1 = x;
                    else
                        fuseY2 = y; fuseZ2 = z; fuseX2 = x;
                    end
                end
                surf([fuseX1; fuseX2], [fuseY1; fuseY2], [fuseZ1; fuseZ2], ...
                    'FaceColor', faceColor, 'FaceAlpha', faceAlpha, ...
                    'EdgeColor', edgeColor, 'LineWidth', 0.3, 'Parent', parent);
            end
            
            % MAIN WINGS - Clean swept wings
            wingX = [-0.5 0.3 0.8 0] + xOffset;
            wingY_R = [0.25 2.5 2.3 0.4];
            wingY_L = -wingY_R;
            wingZ = [0.15 0.25 0.2 0.15];
            
            % Right wing
            patch('XData', wingX, 'YData', wingY_R, 'ZData', wingZ, ...
                'FaceColor', faceColor, 'FaceAlpha', faceAlpha, ...
                'EdgeColor', edgeColor, 'LineWidth', 1, 'Parent', parent);
            % Left wing
            patch('XData', wingX, 'YData', wingY_L, 'ZData', wingZ, ...
                'FaceColor', faceColor, 'FaceAlpha', faceAlpha, ...
                'EdgeColor', edgeColor, 'LineWidth', 1, 'Parent', parent);
            
            % VERTICAL STABILIZER - Tail fin
            tailX = [-1.8 -1.8 -2.3 -2.1] + xOffset;
            tailY = [0 0 0 0];
            tailZ = [0.1 0.9 1.0 0.15];
            
            patch('XData', tailX, 'YData', tailY, 'ZData', tailZ, ...
                'FaceColor', faceColor, 'FaceAlpha', faceAlpha, ...
                'EdgeColor', edgeColor, 'LineWidth', 1, 'Parent', parent);
            
            % HORIZONTAL STABILIZERS - Tail wings
            hstabX = [-1.9 -1.9 -2.3 -2.3] + xOffset;
            hstabY_R = [0 0.9 0.85 0];
            hstabY_L = -hstabY_R;
            hstabZ = [0.25 0.28 0.28 0.25];
            
            % Right stabilizer
            patch('XData', hstabX, 'YData', hstabY_R, 'ZData', hstabZ, ...
                'FaceColor', faceColor, 'FaceAlpha', faceAlpha, ...
                'EdgeColor', edgeColor, 'LineWidth', 1, 'Parent', parent);
            % Left stabilizer
            patch('XData', hstabX, 'YData', hstabY_L, 'ZData', hstabZ, ...
                'FaceColor', faceColor, 'FaceAlpha', faceAlpha, ...
                'EdgeColor', edgeColor, 'LineWidth', 1, 'Parent', parent);
            
            % NOSE CONE
            noseX = [1.8 2.2] + xOffset;
            noseY = [0 0];
            noseZ = [0.05 0];
            noseWidth = [0.08 0.01];
            
            theta = linspace(0, 2*pi, 15);
            for i = 1:2
                y = noseWidth(i) * cos(theta);
                z = noseZ(i) + noseWidth(i) * sin(theta);
                x = ones(size(theta)) * noseX(i);
                if i == 1
                    nY1 = y; nZ1 = z; nX1 = x;
                else
                    nY2 = y; nZ2 = z; nX2 = x;
                end
            end
            surf([nX1; nX2], [nY1; nY2], [nZ1; nZ2], ...
                'FaceColor', faceColor, 'FaceAlpha', faceAlpha, ...
                'EdgeColor', edgeColor, 'LineWidth', 0.5, 'Parent', parent);
        end  % createAirplaneModel
    end  % methods
end  % classdef
