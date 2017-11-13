function genStats(daq)
%This will generate stats on the current simulations
%Updated: 23 Jan 2017 - Jeff Anderson - I added sim finished to the daq
%file in hopes of catching non convergent runs.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Re load from existing file or start a new one
if exist('stat.mat','file')
    load('stat.mat')
else
    stat.simStarted = now;
    stat.lastCheckIn= now;
    stat.elapsedTime = now-now;
    stat.simFinished = 0;
    stat.lapTime = 0;
    stat.currentDistance = 0;
    stat.conv = nan;
    stat.iterNumb = nan;
end


%% Deal with the daq file

if exist('daq','var')
    stat.switchingDaq = daq.header.switchingDaq;
    stat.iterNumb = daq.header.iterNumb;
    try
        stat.currentDistance = daq.rawData.distance.meas(end);
        if isfield(daq.header,'simFinished')
            stat.simFinished = daq.header.simFinished;
        end
        
        if stat.currentDistance > daq.header.timingDistanceFinish
            stat.lapTime = calculateManeuveringTime(daq);
            stat.simFinished = 1;
        end
        if isfield(daq.header,'conv')
            stat.conv = daq.header.conv;
        end
    catch
    end
end

%% Deal with timing:
stat.elapsedTime = now-stat.lastCheckIn+stat.elapsedTime;
stat.lastCheckIn = now;
stat.elapsedTimeString = datestr(stat.elapsedTime,'HH:MM:SS');
save('stat.mat','stat')
fprintf('Elapsed time: %s\n',stat.elapsedTimeString);




%% Setup a text file
statFile = 'stat.txt';
if exist(statFile,'file')
    system(sprintf('rm %s',statFile));
end

fileID = fopen(statFile,'w');
fprintf(fileID,'Iterate             = %i\n',   stat.iterNumb);
fprintf(fileID,'Sim Started         = %s\n',   datestr(stat.simStarted,'yyyy-mm-dd, HH:MM:SS'));
fprintf(fileID,'Last Check In       = %s\n',   datestr(stat.lastCheckIn,'yyyy-mm-dd, HH:MM:SS'));
fprintf(fileID,'Elapsed Time        = %s\n',   stat.elapsedTimeString);
fprintf(fileID,'Current Distance    = %5.1f\n',stat.currentDistance);
fprintf(fileID,'Simulation Finished = %1i\n',  stat.simFinished);
fprintf(fileID,'Lap Time            = %6.4f\n',  stat.lapTime);
fprintf(fileID,'Convergence         = %f\n',     stat.conv);
fclose(fileID);
    
    
