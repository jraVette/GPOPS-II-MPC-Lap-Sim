function [daq,lapTime,conv] = folderedRun(daq,newFolder,varargin)
%This function will 
defaults = {'renameFolderWithTime',true
            'deleteResultsAfterRun',false
            'horizonScheduling',false %used to schedule the horizons
            'runAfterSetup',true %On palmetto, I wanted to use this to setup the sims but not run them necessairly
            'saveSetup',true};
setDefaultsForVarargin(defaults,varargin)


filesToCopy = { 'fourwheelEndpoint.m'
                'fourwheelContinuous.m'
                'shortsegmentGPOPS.m'
                'gpopsMPC_withSwitchingAndAD.m'   
                'simplifiedPacejka.m'
                'guessDaqFile.mat'
                'genStats.m'
                'fourwheelContinuousADiGatorGrd.m'
                'fourwheelContinuousADiGatorGrd.mat'
                'fourwheelContinuousADiGatorHes.m'
                'fourwheelContinuousADiGatorHes.mat'
                'fourwheelEndpointADiGatorGrd.m'
                'fourwheelEndpointADiGatorGrd.mat'
                'fourwheelEndpointADiGatorHes.m'
                'fourwheelEndpointADiGatorHes.mat'};
%                 'matlab.pbs'
%                 'runJob.m'};

%Create and go into new folder            
currentDirectory = pwd;
mkdir(newFolder)
cd(newFolder)
for iFile = 1:length(filesToCopy)
    copyfile(fullfile(currentDirectory,filesToCopy{iFile}),pwd);
end




%Set up daq
daq.header.path = pwd;
conv = 0; %Need now so it wont' try to rename if we haven't run sim

% %Set up matlab script to cd to the correct folder
% system(sprintf('sed -e "4s+.*+ simDirectory = ''%s''; +" <runJob.m>tempText.m',pwd));
% system('mv tempText.m runJob.m');

%Save the initial setup
if saveSetup
    save(fullfile(daq.header.path,daq.header.filename),'daq');
end

%Gen adigator files
% shortsegmentGPOPS('generateAdigatorFiles','manual');
% pause(1)


%Run the sim
if runAfterSetup
    lapTime = nan;
    if ~horizonScheduling
        [daq,lapTime,conv] = gpopsMPC_withSwitchingAndAD(daq);
    else
        [daq,lapTime,conv] = gpopsMPC_withSwitchingAndADAndHorizonScheduling(daq);
    end
    save(fullfile(daq.header.path,daq.header.filename),'daq');
    
end

%Delete results
if deleteResultsAfterRun
    system(sprintf('rm -r %s',newFolder));
end

%Rename the file
if conv && renameFolderWithTime
    newNameFolder = sprintf('%s_ManueveringTime%5.3f',newFolder,lapTime);
    try
        system(sprintf('mv %s %s',newFolder,newNameFolder));
    catch err
        warning('file could not be renamed with maneuvering time')
    end
end

cd(currentDirectory);

%Save the iterate number in folder
if isfield(daq.header,'iterNumb')
    fileID = fopen('iterate.txt','w');
    fprintf(fileID,'Iterate             = %i\n',   daq.header.iterNumb);
    fclose(fileID);
end

end




