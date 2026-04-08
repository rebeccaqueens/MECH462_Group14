clear all; 
close all; 
clc; 


colors = [0.9290 0.6940 0.1250; 0.5210 0.0860 0.8190];% yellow ; purple (RGB)

data = load("C:\Users\rrebe\Downloads\ExoSuitTrials 1.mat"); 
rawData = data.data; 

isi = 0.0005 ; % s 
time = (0:isi:(size(rawData, 1) - 1) * isi)'; % Create time vector based on isi

% --- Trial start/end times in seconds ---
%No Suit Walking Trials 0-12% 
trialTimes.noSuit_0  = [536, 910];
trialTimes.noSuit_8  = [1370, 1706];
trialTimes.noSuit_12 = [1736, 1850];
%Active Suit Walking Trials 0-12% 
trialTimes.suitActive_0  = [2572, 2916];
trialTimes.suitActive_8  = [2940, 3284];
trialTimes.suitActive_12 = [3300, 3414]; %3726];
%Passive Suit Walking Trials 0-12% 
trialTimes.suitPassive_0  = [3890, 4278];
trialTimes.suitPassive_8  = [4314, 4682];
trialTimes.suitPassive_12 = [4700, 5066];

trialNames = fieldnames(trialTimes); 

for i = 1:numel(trialNames)-3
    tStart = trialTimes.(trialNames{i})(1);
    tEnd   = trialTimes.(trialNames{i})(2);
    idx = time >= tStart & time <= tEnd;   % logical indexing
    trialData.(trialNames{i}) = rawData(idx, :); % chop the data
    trialTime.(trialNames{i}) = time(idx); % chop the timeend
end 

%get [slope, intercept]
linearRegress = @(t, dataMatrix) polyfit(t, mean(dataMatrix, 2), 1); 

for i = 1:numel(trialNames)-3
    coeffs.(trialNames{i}) = linearRegress(trialTime.(trialNames{i}), trialData.(trialNames{i}));
    % Compute fitted line
    trialFit.(trialNames{i}) = coeffs.(trialNames{i})(1) * trialTime.(trialNames{i})+ coeffs.(trialNames{i})(2);
end

% --- Plot function ---
figure()
hold on
count = 1 ; 
plotTrialFit = @(trialNames, titleStr)figure(); 
for i = 1:3:numel(trialNames)-3
    plot(trialTime.(trialNames{i})- trialTime.(trialNames{i})(1), trialFit.(trialNames{i}), 'LineWidth', 1.5, 'Color', colors(count,:)) ; 
    count = count + 1 ; 
end
plot(trialTime.noSuit_0 - trialTime.noSuit_0(1), trialData.noSuit_0)
plot(trialTime.suitActive_0 - trialTime.suitActive_0(1), trialData.suitActive_0)
title(" 0 degree incline ")
xlabel("Time (s)")
ylabel("mV Reading")
grid on
legend("None Regression","Active Regression","None","Active")
hold off;




% --- Plot function ---
figure()
hold on; 
count = 1; 
plotTrialFit2 = @(trialNames, titleStr)figure(2); 
for i = 2:3:numel(trialNames)-3
    plot(trialTime.(trialNames{i})- trialTime.(trialNames{i})(1), trialFit.(trialNames{i}), 'LineWidth', 1.5, 'Color', colors(count,:))
    count = count + 1; 
end
plot(trialTime.noSuit_8- trialTime.noSuit_8(1), trialData.noSuit_8)
plot(trialTime.suitActive_8- trialTime.suitActive_8(1), trialData.suitActive_8)
title(" 8 degree incline ")
legend("None Regression","Active Regression","None","Active")
xlabel("Time (s)")
ylabel("mV Reading")
hold off


 
% --- Plot function ---
%match the lengths of the recorded files 
figure()
hold on;
count = 1; 
plotTrialFit3 = @(trialNames, titleStr)figure(3); 
for i = 3:3:numel(trialNames)-3
    plot(trialTime.(trialNames{i})- trialTime.(trialNames{i})(1), trialFit.(trialNames{i}), 'LineWidth', 1.5, 'Color', colors(count,:))
    count = count +1 ; 
end 


plot(trialTime.noSuit_12 - trialTime.noSuit_12(1), trialData.noSuit_12);
plot(trialTime.suitActive_12 - trialTime.suitActive_12(1), trialData.suitActive_12);
title(" 12 degree incline ")
legend("None Regression","Active Regression","None","Active")
xlabel("Time (s)")
ylabel("mV Reading")
hold off



% ---------------------------------------analysis ttest-----------------------------------------------
%lowering sampling rate: 
fs_old = 2000;% original Hz
fs_new = 100;% new Hz
factor = fs_old / fs_new;
% Bin size for averaging
binSize = 200; 

inclines = [0, 8, 12];   
sig_noSuit = cell(1, numel(inclines));
sig_active = cell(1, numel(inclines));

%sig_noSuit_12 = mean(trialData.noSuit_12, 2);   % average across channels
%sig_active_12 = mean(trialData.suitActive_12, 2);

for i = 1:numel(inclines)
    switch inclines(i)
        case 0
            sig_noSuit{i} = mean(trialData.noSuit_0, 2);
            sig_active{i} = mean(trialData.suitActive_0, 2);
        case 8
            sig_noSuit{i} = mean(trialData.noSuit_8, 2);
            sig_active{i} = mean(trialData.suitActive_8, 2);
        case 12
            sig_noSuit{i} = mean(trialData.noSuit_12, 2);
            sig_active{i} = mean(trialData.suitActive_12, 2);
    end
end


% Downsample by taking every 'factor'-th point
sig_noSuit_ds = cell(1, numel(inclines));
sig_active_ds = cell(1, numel(inclines));
% Standard deviations
s1 = cell(1, numel(inclines));
s2 = cell(1, numel(inclines));

SE = cell(1, numel(inclines)); 

for i = 1:numel(inclines)
    sig_noSuit_ds{i} = sig_noSuit{i}(1:factor:end);
    sig_active_ds{i} = sig_active{i}(1:factor:end);
end


for i = 1:numel(inclines)
    % Binning the downsampled data
    n1 = length(sig_noSuit_ds{i});
    n2 = length(sig_active_ds{i});


    if (i == 1)
     x1_binned = mean(reshape(sig_noSuit_ds{i}(1:floor(n1/binSize)*binSize,:), binSize, []));
      x2_binned = mean(reshape(sig_active_ds{i}(1:floor(n2/binSize)*binSize,:), binSize, []));
       [h, p] = ttest2(x1_binned, x2_binned, 'Tail', 'left'); % right-tailed: suit reduces effort
        x = 100;
    end 
    if (i == 2)
      x1_binned = mean(reshape(sig_noSuit_ds{i}(1:floor(n1/binSize)*binSize,:), binSize, []));
       x2_binned = mean(reshape(sig_active_ds{i}(1:floor(n2/binSize)*binSize,:), binSize, []));
        [h, p] = ttest2(x1_binned, x2_binned, 'Tail', 'left'); % right-tailed: suit reduces effort
        x = 10;
    end
    if (i == 3)
     x1_binned = mean(reshape(trialData.(trialNames{i})(1:floor(n1/binSize)*binSize,:), binSize, []));
      x2_binned = mean(reshape(trialData.(trialNames{i+3})(1:floor(n2/binSize)*binSize,:), binSize, []));
       [h,p] = ttest2(x1_binned, x2_binned);
       x = 10;
    end
    
    %x1_binned = mean(reshape(sig_noSuit_ds{i}(1:floor(n1/binSize)*binSize), binSize, []));
    %x2_binned = mean(reshape(sig_active_ds{i}(1:floor(n2/binSize)*binSize), binSize, []));
    %[h,p] = ttest2(x1_binned, x2_binned);
    % Standard deviations
    s1 = std(x1_binned);
    s2 = std(x2_binned);
    
    %standard deviation 
    SE = sqrt(s1^2/length(x1_binned) + s2^2/length(x2_binned));
    
    % Two-sample t-test
    % Averages
    x_1 = mean(x1_binned);
    x_2 = mean(x2_binned);

    % t-statistic
    t_stat = (x_1 - x_2) / SE ;
    % Degrees of freedom (approximate using Welch-Satterthwaite equation)
    dof = (s1^2/length(x1_binned) + s2^2/length(x2_binned))^2 / ...
     ((s1^2/length(x1_binned))^2/(length(x1_binned)-1) + (s2^2/length(x2_binned))^2/(length(x2_binned)-1));

    % Display results
    fprintf('Incline %d°:\n', inclines(i));
    fprintf('No Suit average: %.4f\n', x_1);
    fprintf('Active Suit average: %.4f\n', x_2);
    fprintf('p-value: %.4f\n', p);
    if h == 1
        fprintf('  Null hypothesis rejected: Active suit reduces effort\n\n');
    else
        fprintf('  Null hypothesis NOT rejected\n\n');
    end
    t_vals = linspace(t_stat-x, t_stat+x, 1000);
    % t-distribution PDF
    y = tpdf(t_vals, dof);
    % Plot
    figure;
    plot(t_vals, y, 'LineWidth', 2);
    xline(t_stat, 'r--', 'LineWidth', 2); % red dashed vertical line
    grid on;
    xlabel('t');
    ylabel('Probability Density');
    title(['t-Distribution with ', num2str(inclines(i)), '°']);
end
