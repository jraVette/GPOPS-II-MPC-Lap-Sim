function daq = generateInitialDaq(varargin)
%This funciton will generate an initial daq file for running the GPOPS MPC
%It will be a time optimal switched solution
%
%Creation: 28 Dec 2015 - Jeff Anderson
%Updated:  06 Jan 2017 - Jeff Anderson updated hockenheim initial
%    conditions in hopes of making a cyclic lap possible.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

defaults = {'loadInitialGuess',true         %I needed to skip this for open loop solving only 
            'loadBounds',true               %I needed to skip this for open loop solving only 
            'guessFilename','guessDaqFile.mat'};             
setDefaultsForVarargin(defaults,varargin)

%% Load Vehicle 
car = 'F1'; %Choose 'F1' or 'C7R'
vehicleDirectory = fullfile(jatecPath,'Resources/In house code/Vehicle Parameters/');
switch car
    case 'F1'
        carFileName = 'LimebeerF1Car.mat';
        fullVehicleFile = fullfile(vehicleDirectory,'Optimal Control Research',carFileName);
    case 'C7R'
        carFilename = '2015_Corvette_C7R.mat';
        fullVehicleFile = fullfile(vehicleDirectory,'Corvette',carFilename);
end
load(fullVehicleFile);
vehicle.parameter.enginePower.meas = 500*1000*myConstants.w2hp;
vehicle.parameter.differentialFrictionCoeff.meas = 1e2;

%track - must be first, needed for switching
%trackFilename = 'PathInfoChicane2LoopsAndFrontStraight.mat';
% trackFilename = 'PathInfoChicaneStraightsBeforeAndAfter.mat';
trackFilename = 'HockenheimLoopedBeforeAndAfter.mat';
%trackFilename = 'Nurburgring.mat';
%trackFilename = 'PathInfoChicane.mat'

track = load(trackFilename);
track = track.track;

%appropiate switching
s                       = [-100 1*10000]';                                 %just big numbers so it spans the track
c                       = [0 0]';                                          %time optimal switching
switchingDaq.rawData.distance = createDaqChannelData(s,'m','Distance');
switchingDaq.rawData.switching = createDaqChannelData(c,'','Switching');

clear s c %don't need them
 
%MPC Parameters
horizon                 = 200;                                             %[m] Look ahead %150m for chicane, updated based on course DOE
controlHorizon          = 10;                                               %[m] MPC update %5m for chicane, updated based on course DOE
interpolationAccuracy   = 0.25;                                            %[m] ds
horizonDecrement        = 10;                                              %[m] used to shorten horizon incase of convergence error
minimumHorizon          = 50;                                              %[m] minimum acceptable horizon

%Initalizaiton
initialDistance         = -200;                                            %[m] s0 %I want to start well before start/finish line                                         %Where the car gets on the track
timingDistanceStart     = 0;                                               %Where timing starts
timingDistanceFinish    = track.finishDistance;
finishDistance          = timingDistanceFinish+10;
horizonRefinement       = true;
   
clear trackRawData


%% Switching
s = initialDistance:controlHorizon:finishDistance;
     %'vx';'time'},...
c0  = [ 0     1  ];
lb0 = [ 0     0  ];
ub0 = [ 1     1  ];
c  = repmat(c0,length(s),[]);
lb = repmat(lb0,length(s),[]);
ub = repmat(ub0,length(s),[]);

switchingDaq.rawData.distance = createDaqChannelData(s,'m','Distance');
switchingDaq.rawData.switching = createDaqChannelData(c,'','Switching');
switchingDaq.rawData.lb = createDaqChannelData(lb,'','Switching Lower Bounds');
switchingDaq.rawData.ub = createDaqChannelData(ub,'','Switching Upper Bounds');
clear s c c0 lb ub lb0 ub0%don't need them

%% Initial Guess
if loadInitialGuess
    optimizationGuessType = 'lastMpcHorizon'; %Use either 'lastMpcHorizon' or 'global'.  Use global to load an exisiting solution or lastMpc to use solver
    delta = []; %I'm overwriting this builtin matlab function
    distance = []; %I want to index this and it will throw and error otherwise
    guessDaq = load(guessFilename);                                                    %Load a daq file from open loop sims
    guessDaq = guessDaq.daq; %move one level up
    guessDaq = getChannelDataFromDaqFile(guessDaq,'distance');
    
 
    [~,rawDataStruct]       = getChannelDataFromDaqFile(guessDaq,[],...         %Get the right channel data 200m before the end of the lap till the end
                                 'atIndepVariableTime',{distance(end)+initialDistance 'end'},...
                                 'indepVariableChannel','distance');                     
    timeGuess               = [distance-distance(1)+initialDistance];   
    stateGuess              = [ePsi  ey       vx       vy       r     time-time(1) omega_L1 omega_R1 omega_L2 omega_R2 delta torqueDemand];
    x0                      = stateGuess(1,:);
    controlGuess            = [u1 u2];
    trackRawData            = trackParameterInterpolation(track,initialDistance); %Get track data
    pathX0                  = xPath(1);                                 %We want the track to start at X0 = s0
    pathY0                  = yPath(1);                                               %We want the track to start at Y0 = 0
    pathPsi0                = pathHeading(1);                        %However, we want the trak oriented correclty
    segPathX0               = xPath(1);                                          %This is updated per segment, start now
    segPathY0               = yPath(1);                                          %This is updated per segment, start now
    segPathPsi0             = pathHeading(1);                                        %This is updated per segment, start now  
    
end


%Deal with bounds
if loadBounds
    ePsiMax     = 25*myConstants.deg2rad;                                  %Pevious solutions said this was bounded by [-25, 25]
    eyMax       = 5;                                                       %Road width constraint
    vxLb        = 5;                                                       %Original bound
    vxUb        = 90;%69.9240505593388;                                        %Original Bound
    vyMax       = 10;                                                      %Orignal bounds
    rMax        = 55*myConstants.deg2rad;                                  %Orignal bound 45 deg/s
    tLb         = 0;
    tUb         = 4.609395789295020;                                       %Oringal bounds
    omegaLb     = -vxUb/vehicle.tire_front.reff.meas;
    omegaUb     = -vxLb/vehicle.tire_front.reff.meas;
    deltaMax    = 20*myConstants.deg2rad;                                  %Was at 20deg but, sim showed only using 12, so shrink
    TMax        = 5000;
    deltaRate   = 100*myConstants.deg2rad;                                 % rad/s
    TRate       = 10*1000;                                                 % N*m/s
    
    bounds.lbX              = [-ePsiMax -eyMax 	 vxLb    -vyMax      -rMax   tLb     omegaLb omegaLb omegaLb omegaLb -deltaMax -TMax]; 
    bounds.ubX              = [ ePsiMax  eyMax   vxUb     vyMax       rMax   tUb     omegaUb omegaUb omegaUb omegaUb  deltaMax  TMax];
    bounds.ubU              = [ deltaRate  1];
    bounds.lbU              = [-deltaRate -1];
    bounds.pathLower        = -1e6;
    bounds.pathUpper        = 0;
%    bounds.integralLower    = 0;
%    bounds.integralUpper    = tUb;]
                               %   'vx'; 'time'
	costScaling         = [   vxUb   tUb    ];
    switchingDaq.rawData.costScaling.meas = costScaling;
    clear vxLb vxUb tUb omegaLb omegaUb DAQ
    clear costScaling
end

%Clear out the vars we don't need
if exist('rawDataStruct','var')
    cellfun(@clear,fieldnames(rawDataStruct))                                  %Have the daq, don't need channels any more
end

%% Gpops options
%GPOPS OUTPUT FLAGS:
% -1 : not solved!
%  0 : EXIT: Optimal Solution Found.
%  1 : EXIT: Solved To Acceptable Level.
%  2 : EXIT: Converged to a point of local infeasibility. Problem may be infeasible.
% NOT SURE THE NUMBER EXIT: Maximum Number of Iterations Exceeded.
gpopsOptions.acceptableNlpOutpus = [0 1 2] ; 
gpopsOptions.specifyMeshIterationSolution = 'auto'; %Choose 'auto' for the default or an integer for the mesh number

gpopsOptions.mesh.method       = 'hp-PattersonRao';
gpopsOptions.mesh.tolerance    = 1e-5;
gpopsOptions.mesh.maxiterations = 1;
gpopsOptions.mesh.colpointsmin = 7;
gpopsOptions.mesh.colpointsmax = 20;

gpopsOptions.setup.name                        = 'quadCar';
% gpopsOptions.setup.functions.continuous        = @fourwheelContinuous;
% gpopsOptions.setup.functions.endpoint          = @fourwheelEndpoint;
gpopsOptions.setup.nlp.solver                  = 'ipopt';
gpopsOptions.setup.derivatives.supplier        = 'adigator';%'adigator';%'sparseFD'; %'adigator';%
gpopsOptions.setup.derivatives.derivativelevel = 'second';
gpopsOptions.setup.scales.method               = 'automatic-hybridUpdate';
gpopsOptions.setup.method                      = 'RPM-Differentiation';
gpopsOptions.setup.displaylevel                = 2;

%Ga options
gaFilename = 'gaInformation.mat';
populationSize = 100;
nonConvergentCost = 200;
wallTime = 240; %After this time, nonConvergentCost applied to iterate.  [min]


%Set up daq
filename = sprintf('%s_GPOPS_MPC_Solution-tOpt,deltaS=%i,horizon=%i',datestr(now,'yyyy-mm-dd_HH_MM_SS'),controlHorizon,horizon);
shortFilename = 'tOpt';
daq.header = saveVariablesAssignedToPointInStructure('exclude',{'varargin';'vehicle'});
daq.vehicle = vehicle;
daq.header.iterNumb = 1;
daq.header.path = pwd;
daq.header.notes = 'Created using an MPC approach with GPOPS-II';
