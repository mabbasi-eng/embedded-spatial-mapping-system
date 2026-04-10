clear; %removes variables from MATLAB workspace
clc; %clear cmd window
close all; %closes any open figure windows

%% ---------------- Configuration ----------------
port = "COM7";
baudrate = 115200;
use_cm = true; %want to be displayed in cm instead of mm

% Compare each scan to the previous one and clip large outward jumps
useScanToScanClipping = true;
clipTolerance_cm = 400;     % allowed extra distance beyond previous scan

% Smooth sudden spikes within each scan
useWithinScanSmoothing = true;
spikeTolerance_cm = 80;     % lower = stronger smoothing


%% ---------------- Open Serial ----------------
device = serialport(port, baudrate); %opens serial connection to board
device.Timeout = 30; %matlab will wait up to 30 secs for serial read
configureTerminator(device, "LF"); %each incoming line ends with line feed character
flush(device); %clears any old serial data in buffer

fprintf("Opened %s at %d baud\n", port, baudrate);
fprintf("Now press the board button to start scanning...\n\n");

%% ---------------- Data Storage ----------------
scanVals  = []; % stores scan number for each data point
dispVals  = []; % stores x-displacement 0,10,20 cm
angleVals = []; % stores angle
distVals  = []; % stores radial distance

doneFlag = false;
abortFlag = false;

%% ---------------- Read UART ----------------
while ~doneFlag && ~abortFlag
    while device.NumBytesAvailable == 0 %if there is no serial datawaiting yet, matlab pauses and checks again
        pause(0.05);
    end

    raw = readline(device);%reads one whole line from serial port
    line = strtrim(string(raw)); %convert to matlab string & remove whitespances or newline

    if line == "" %if line empty, skip and go to next loop cycle
        continue;
    end

    fprintf("%s\n", line); %print every line in cmd window

    if startsWith(line, "DATA")
        parts = split(line, ","); %code assumes one measurement and splits it with commas

        if numel(parts) == 5 %checks data line has 5 comma seperated parts
            s = str2double(parts(2));       % extracts scan index
            d = str2double(parts(3));       % displacement (cm)
            a = str2double(parts(4)) / 100; % hundredths of degree -> degree
            r = str2double(parts(5));       % distance in mm

            if all(~isnan([s d a r])) %checks all converted values are valid numbers
                scanVals(end+1,1)  = s;
                dispVals(end+1,1)  = d;
                angleVals(end+1,1) = a;
                distVals(end+1,1)  = r;
            end
        end

    elseif startsWith(line, "ENDSCAN")
        % no action needed

    elseif line == "DONE"
        doneFlag = true;

    elseif line == "ABORTED"
        abortFlag = true;
    end
end

clear device;
fprintf("\nSerial closed.\n");

if isempty(distVals)
    error("No DATA lines were received.");
end

%% ---------------- Unit Conversion ----------------
if use_cm
    X = dispVals;          % already cm
    R = distVals / 10;     % mm -> cm
    unitText = 'cm';
else
    X = dispVals * 10;     % cm -> mm
    R = distVals;
    unitText = 'mm';
end

%% ---------------- Sort Data Scan-by-Scan ----------------
uniqueScans = unique(scanVals);
numScans = length(uniqueScans);

scanData = cell(numScans,1);

for k = 1:numScans
    idx = find(scanVals == uniqueScans(k));

    ang = angleVals(idx);
    xk  = X(idx);
    rk  = R(idx);

    [angSorted, order] = sort(ang);
    idxSorted = idx(order);

    scanData{k}.idx = idxSorted;
    scanData{k}.ang = angSorted;
    scanData{k}.x   = xk(order);
    scanData{k}.r   = rk(order);
end

%% ---------------- Scan-to-Scan Clipping ----------------
R_clip = R;

if useScanToScanClipping
    for k = 2:numScans
        prevR = scanData{k-1}.r;
        currR = scanData{k}.r;

        n = min(length(prevR), length(currR));

        for i = 1:n
            allowedMax = prevR(i) + clipTolerance_cm;

            if currR(i) > allowedMax
                currR(i) = prevR(i);
            end
        end

        R_clip(scanData{k}.idx(1:n)) = currR(1:n);
        scanData{k}.r = currR;
    end
end

%% ---------------- Within-Scan Spike Smoothing ----------------
if useWithinScanSmoothing
    uniqueScans = unique(scanVals);

    for k = 1:length(uniqueScans)
        idx = find(scanVals == uniqueScans(k));
        ang = angleVals(idx);
        r   = R_clip(idx);

        [~, order] = sort(ang);
        idx = idx(order);
        r   = r(order);

        if length(r) >= 3
            rNew = r;

            for i = 2:length(r)-1
                neighborAvg = (r(i-1) + r(i+1)) / 2;

                if abs(r(i) - neighborAvg) > spikeTolerance_cm
                    rNew(i) = neighborAvg;
                end
            end

            R_clip(idx) = rNew;
        end
    end
end

%% ---------------- Polar to Cartesian ----------------
Y = R_clip .* cosd(angleVals);
Z = R_clip .* sind(angleVals);

Y_plot = Y;
Z_plot = Z;

%% ---------------- Final Plot ----------------
figure;
PlotCurrentData(scanVals, X, angleVals, Y_plot, Z_plot, unitText);
title('Final 3D Reconstruction');

%% ---------------- Local Plot Function ----------------
function PlotCurrentData(scanVals, X, angleVals, Y_plot, Z_plot, unitText)
    hold on;
    grid on;
    axis equal;
    view(35,25);

    uniqueScans = unique(scanVals);
    numScans = length(uniqueScans);

    % Light-to-dark blue gradient
    lightBlue = [0.70 0.85 1.00];
    darkBlue  = [0.00 0.20 0.65];

    if numScans == 1
        blueColors = darkBlue;
    else
        t = linspace(0,1,numScans)';
        blueColors = (1-t).*lightBlue + t.*darkBlue;
    end

    for k = 1:numScans
        idx = find(scanVals == uniqueScans(k));

        ang = angleVals(idx);
        xk  = X(idx);
        yk  = Y_plot(idx);
        zk  = Z_plot(idx);

        [~, order] = sort(ang);
        xk = xk(order);
        yk = yk(order);
        zk = zk(order);

        thisColor = blueColors(k,:);

        if length(xk) >= 2
            plot3([xk; xk(1)], [yk; yk(1)], [zk; zk(1)], '-', ...
                'LineWidth', 2, ...
                'Color', thisColor);
        elseif length(xk) == 1
            plot3(xk, yk, zk, '.', ...
                'LineWidth', 2, ...
                'MarkerSize', 14, ...
                'Color', thisColor);
        end
    end

    for k = 1:numScans-1
        idxA = find(scanVals == uniqueScans(k));
        idxB = find(scanVals == uniqueScans(k+1));

        angA = angleVals(idxA);
        angB = angleVals(idxB);

        xA = X(idxA); yA = Y_plot(idxA); zA = Z_plot(idxA);
        xB = X(idxB); yB = Y_plot(idxB); zB = Z_plot(idxB);

        [~, orderA] = sort(angA);
        [~, orderB] = sort(angB);

        xA = xA(orderA); yA = yA(orderA); zA = zA(orderA);
        xB = xB(orderB); yB = yB(orderB); zB = zB(orderB);

        n = min(length(xA), length(xB));
        lineColor = blueColors(k,:);

        for m = 1:n
            plot3([xA(m) xB(m)], [yA(m) yB(m)], [zA(m) zB(m)], '--', ...
                'Color', lineColor);
        end
    end

    xlabel(['X displacement (' unitText ')']);
    ylabel(['Y (' unitText ')']);
    zlabel(['Z (' unitText ')']);
    hold off;
end
