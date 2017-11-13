function [daq,lapTime,conv] = gpopsMPC_withSwitchingAndAD(daq,varargin)
%MPC with GPOPS with switching
%INPUTS: 
%    switching - c(s) saved as a daq file
%    daq - optional to start from
%OUTPUS:
%    daq - solution daq
%    lapTime - double of the laptime calculated
%    conv - status of the convergence
%
%Updated: 6 Oct 2015 - Jeff Anderson - for switching
%Updated: 30 Dec 2015 - Jeff Anderson - removed switching from input arg
%and put it in daq.header.switchingDaq
%Updated: 28 Jan 2015 - Jeff Anderson - save each horizon in their own
%file.  The files become really large and I/O becomes the largest
%computational cost.  I have this as a changeable option in the default
%settings.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Defaults
defaults = {'makePlots',false                   %Update the progress of simulation at each setp (won't work in command line)
            'statusGui',false                   %Make a figure that updates user on progress (won't work in command line)
            'saveHorizonInDaq',false            %Saving all horizion info in the daq makes a really big file...put each horizon in own daq for "false", otherwise it will save it in daq.  Ok since we're using the folder separated runs
            'replayExistingSolution',false};    %Used for debugging
        
setDefaultsForVarargin(defaults,varargin)        

    
%% Initialization

%MPC Parameters
track                   = daq.header.track;
horizon                 = daq.header.horizon;
controlHorizon          = daq.header.controlHorizon;
interpolationAccuracy   = daq.header.interpolationAccuracy; %m
horizonDecrement        = daq.header.horizonDecrement;      %[m] used to shorten horizon incase of convergence error
minimumHorizon          = daq.header.minimumHorizon;        %[m] minimum acceptable horizon
initialDistance         = daq.header.initialDistance;
pathX0                  = daq.header.pathX0;
pathY0                  = daq.header.pathY0;
pathPsi0                = daq.header.pathPsi0;
finishDistance          = daq.header.finishDistance;        %used to stop the sim
% timingDistanceStart     = daq.header.timingDistanceStart;
% timingDistanceFinish    = daq.header.timingDistanceFinish; %used to calculate maneuvering time
vehicle                 = daq.vehicle;
bounds                  = daq.header.bounds;
gpopsOptions            = daq.header.gpopsOptions;


if isfield(daq.header,'lastSeg') %For mid sim restart
    x0              = daq.header.lastSeg.x0;
    timeGuess       = daq.header.lastSeg.timeGuess;
    stateGuess      = daq.header.lastSeg.stateGuess;
    controlGuess    = daq.header.lastSeg.controlGuess;
    segPathX0       = daq.header.lastSeg.segPathX0;
    segPathY0       = daq.header.lastSeg.segPathY0;
    segPathPsi0     = daq.header.lastSeg.segPathPsi0;
    currentDistance = daq.header.lastSeg.currentDistance;
    iSeg            = daq.header.lastSeg.iSeg;
    lastTime        = daq.rawData.time.meas(end);
else
    x0              = daq.header.x0;
    timeGuess       = daq.header.timeGuess;
    stateGuess      = daq.header.stateGuess;
    controlGuess    = daq.header.controlGuess;
    segPathX0       = daq.header.segPathX0;
    segPathY0       = daq.header.segPathY0;
    segPathPsi0     = daq.header.segPathPsi0;
    currentDistance = daq.header.initialDistance;
    iSeg            = 1;
end
preferredHorizon = horizon;

%% GUI Update
if statusGui
    hFig = figure('Name','Status updates',...
                  'MenuBar','none',...
                  'ToolBar','figure',...
                  'NumberTitle','off');
    set(hFig,'HandleVisibility','off');

    vSpace          = uiextras.VBox('Parent',hFig);
    hTitle          = uicontrol('Parent',vSpace,'Style','edit','String',sprintf('File:              %s',daq.header.filename));
    hDistance       = uicontrol('Parent',vSpace,'Style','edit','String',sprintf('Current Distance:  %5.1f',currentDistance));
    hCurrentSeg     = uicontrol('Parent',vSpace,'Style','edit','String',sprintf('Current Segment:   %i',iSeg));
    hIterHorizon    = uicontrol('Parent',vSpace,'Style','edit','String',sprintf('Iteration Horizon: %i',0));
    hCurrentHorizon = uicontrol('Parent',vSpace,'Style','edit','String',sprintf('Current Horizon: %%5.0f',horizon));
end


%% Master Loop
genStats(); %Start a stat file
finishFlag = false;
while ~finishFlag  
    clear mex
   
    %Inner while loop per horizon, Horizon refinement if necessary:
    convergenceFlag = false;
    iterInnerLoop = 1;
    horizon = preferredHorizon;
    cpuTime = 0;
    
    if ~replayExistingSolution                                         %Normally this option, actually soliving
        while ~convergenceFlag                  
            %Update status gui
            if statusGui
                set(hDistance,'String',sprintf('Current Distance:  %5.1f',currentDistance));
                set(hCurrentSeg,'String',sprintf('Current Segment:   %i',iSeg))  
                set(hIterHorizon,'String',sprintf('Iteration Horizon: %i',iterInnerLoop));
                set(hCurrentHorizon,'String', sprintf('Current Horizon: %5.0f',horizon));
                drawnow
            end
            hLocal = tic;

            [segDaq,convergenceFlag] = shortsegmentGPOPS(...
                                                   currentDistance,...
                                                   horizon,...
                                                   x0,...
                                                   timeGuess,...
                                                   stateGuess,...
                                                   controlGuess,...
                                                   daq.header.switchingDaq,...
                                                   track,...
                                                   vehicle,...
                                                   bounds,...
                                                   gpopsOptions);
            localCpuTime = toc(hLocal);
            fprintf('Current Distance = %6.2f, Current Segment %i, Horizon Iter = %i, Horizon = %4.0f\n',currentDistance,iSeg,iterInnerLoop,horizon);
            cpuTime = cpuTime + localCpuTime;
            if ~convergenceFlag 
                horizon = horizon - horizonDecrement;
                iterInnerLoop = iterInnerLoop+1;
                if horizon < minimumHorizon
                    conv = 0;
                    lapTime = nan;
                    daq.header.conv = conv;
                    daq.header.simFinished = 1;
                    warning('Minimum horizon - exited program')
                    return
                end
            end
            seqDaq.cpuTime = cpuTime;
            seqDaq.nHorizonRementLoops = iterInnerLoop;
        end
    else                                                                    %If debugging, just load the solution
        segDaq = loadHorizonInfoForDaqAtHorizon(daq,iSeg);
    end
    
        
    
    %Interp the track and get the frenet calcs and save horizon
    trackRawData = trackParameterInterpolation(track,segDaq.rawData.distance.meas);
    segDaq.rawData = catstruct(segDaq.rawData,trackRawData);
    vars = {'s','distance'
            'k','curvature'
            'ey','ey'
            'ePsi','ePsi'
            'trackWidth','trackWidth'};
    getChannelDataFromDaqFile(segDaq,vars);
    frenetRawData = frenetToXYFrameTrajectory(k,s,ey,ePsi,segPathX0,segPathY0,segPathPsi0,trackWidth(1),'suppressPlot',true);
    segDaq.rawData = catstruct(segDaq.rawData,frenetRawData);
    
    %Make sure no nans exist, if they do then error out if not, then save
    %and move on
    output = writeDaqChannelsToMatrix(segDaq);
    ind = find(isnan(output));
    if ~isempty(ind)
        error('Nans found in the output data, find source');
    end   
    
    %Save the segment either in the daq or a new file 
    if saveHorizonInDaq
        daq.movingHorizon(iSeg) = segDaq;  
    else
        tempDaq.daq = segDaq;        
        segFilename = sprintf('Horizon_%03i_daqForFile_%s',iSeg,daq.header.filename);
        tempDaq.daq.header.filename = segFilename;
        tempDaq.daq.header.path = daq.header.path;
        save([segFilename '.mat'], '-struct', 'tempDaq') 
    end
    

    %If this is the first time through loop, need to create space for the
    %channels 
    channels = daqChannels(segDaq);
    if iSeg == 1;
        daq.rawData = segDaq.rawData;
        for iCh = 1:length(channels)
            daq.rawData.(channels{iCh}) = createDaqChannelData([],'',channels{iCh});
        end
        daq.rawData.iControlHorizon = createDaqChannelData([],'','Control Horizon');
    end
    
    %
     close all
       
    if statusGui
        %Plot the solution space
        s = initialDistance:0.5:currentDistance+horizon+200;
        localTrack = trackParameterInterpolation(track,s);
        k = localTrack.curvature.meas;
        frenetToXYFrameTrajectory(k',s',zeros(size(s')),zeros(size(s')),pathX0,pathY0,pathPsi0,10)
        hOverallFig = gcf;
        legend off
        delete(findall(gcf,'Type','Line','-and','Color','b'))
        
        %Plot the MPC orizion
        hold on
        plot(segDaq.rawData.xCar.meas,segDaq.rawData.yCar.meas,'r','linewidth',1.5)

        
        if iSeg > 1 %Can't do it on the first one no data
            plot(daq.rawData.xCar.meas,daq.rawData.yCar.meas,'k','linewidth',1.5)
        end
    end
    
              
    %Interpolate solutions over control horizon
    interpolationDistance = currentDistance:interpolationAccuracy:currentDistance+controlHorizon;
    for iCh = 1:length(channels)
        
            tempCh = rowVector(interp1(segDaq.rawData.distance.meas,...
                                       segDaq.rawData.(channels{iCh}).meas,...
                                       interpolationDistance,'spline','extrap'));
            switch channels{iCh}
                case 'time'
                    if isempty(daq.rawData.time.meas)
                        tempCh0 = 0;
                    else
                        tempCh0 = lastTime; %defined before we removed the last index of the raw data
                    end
                    tempCh = tempCh+tempCh0;
            end
            daq.rawData.(channels{iCh}).meas = [daq.rawData.(channels{iCh}).meas; tempCh];
    end
    daq.rawData.iControlHorizon.meas = [daq.rawData.iControlHorizon.meas; rowVector(iSeg*ones(size(interpolationDistance)))];
    
    %Initial guess for next loop
    x0 = [daq.rawData.ePsi.meas(end)
          daq.rawData.ey.meas(end)
          daq.rawData.vx.meas(end)
          daq.rawData.vy.meas(end)
          daq.rawData.r.meas(end)
          0 %I just did the segments w/ 0 time so state constraints could be reasonable %daq.rawData.time.meas(end)]';
          daq.rawData.omega_L1.meas(end)
          daq.rawData.omega_R1.meas(end)
          daq.rawData.omega_L2.meas(end)
          daq.rawData.omega_R2.meas(end)
          daq.rawData.delta.meas(end)
          daq.rawData.torqueDemand.meas(end)]';
      
    switch daq.header.optimizationGuessType
        case 'lastMpcHorizon'
            timeGuess       = segDaq.gpopsOutput.result.solution.phase.time;
            stateGuess      = segDaq.gpopsOutput.result.solution.phase.state;
            controlGuess    = segDaq.gpopsOutput.result.solution.phase.control;
        case 'global'
            timeGuess = daq.header.timeGuess;
            stateGuess = daq.header.stateGuess;
            controlGuess = daq.header.controlGuess;
        otherwise
            error('Invalid option for guess type')
    end
            
    
    segPathX0       = daq.rawData.xPath.meas(end);
    segPathY0       = daq.rawData.yPath.meas(end);
    segPathPsi0     = daq.rawData.pathHeading.meas(end);
    
    %Save them in the daq file
    daq.header.lastSeg.x0           = x0;
    daq.header.lastSeg.timeGuess    = timeGuess;
    daq.header.lastSeg.stateGuess   = stateGuess;
    daq.header.lastSeg.controlGuess = controlGuess;
    daq.header.lastSeg.segPathX0    = segPathX0;
    daq.header.lastSeg.segPathY0    = segPathY0;
    daq.header.lastSeg.segPathPsi0  = segPathPsi0;
              
    %Remove last point
    %Next time around, the last point in the global solution will be the
    %first point in local solution...to aviod dupliate points, remove the
    %last point in the global solution
    channels = daqChannels(daq);
    for iCh = 1:length(channels)
        if strcmp(channels{iCh},'time')
            lastTime = daq.rawData.(channels{iCh}).meas(end);
        end
        daq.rawData.(channels{iCh}).meas(end) = [];
    end
    
    %Update loop
    currentDistance = currentDistance+controlHorizon;
    if daq.rawData.distance.meas(end) >= finishDistance
        finishFlag = ~finishFlag;
    end
    iSeg = iSeg + 1;
    
    %Save the loops vars
    daq.header.lastSeg.currentDistance = currentDistance;
    daq.header.lastSeg.iSeg = iSeg;
    
    %Save files - wrap in a cacth try because of palmetto issuse w/
    %resource problem?
    try
        saveFiles(daq);
    catch err
        warning('saving problem: error msg: %s',err.message)
    end
    
    %Update stats
    genStats(daq);
    
end%while finishFlag

%% Double check monotic time
time = daq.rawData.time.meas;
if ~ismonotonic(time)
    conv = 0;
    genStats(daq);
    return
end



conv = 1;
daq.header.conv = conv;
saveFiles(daq)
global DAQ
DAQ{1} = daq;

calculateManeuveringTime
lapTime = displayLapTimes([],'suppressOutput',true);
putAllHorizonsIntoDaq
% fixHorizonTime
% mapSwitchingCostToRawData
genStats(daq);
saveFiles

daq = DAQ{1};


