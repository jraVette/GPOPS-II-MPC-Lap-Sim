function [daq,convergence] = shortsegmentGPOPS(s0,horizon,x0,timeGuess,stateGuess,controlGuess,switchingDaq,track,vehicle,bounds,gpopsOptions,varargin)
%Particle motion MTM
%
%Creation: 2015 dec 8  - Jeff Anderson
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


if nargin == 0
    daq = generateInitialDaq;
    extractVariablesFromSructure(daq.header);
    
    clearallbut s0 horizon x0 timeGuess stateGuess controlGuess switchingDaq track vehicle bounds daq
    s0 = -50;
%     horizon = 150; %any higher and the compuattion seems to have problems w/ non convergence
    %NOTE: Omegas in SAE coords are negative!!! Also, note that
    %omega = -Vx*(kappa+1)/R to easily be able to back out
    %guess
              
end

if nargin == 2 %This is the case where we are just setting up adigator files
    if isa(s0,'char')
        varargin{1} = s0;
        varargin{2} = horizon;
%         loadFiles
%         global DAQ
%         daq = DAQ{1};
        daq = generateInitialDaq;
        s0 = daq.header.initialDistance;
        horizon = daq.header.horizon;
        track = daq.header.track;
        vehicle = daq.vehicle;
        x0 = daq.header.x0;
        timeGuess = daq.header.timeGuess;
        stateGuess = daq.header.stateGuess;
        controlGuess = daq.header.controlGuess;
        bounds = daq.header.bounds;
        switchingDaq = daq.header.switchingDaq;
        gpopsOptions = daq.header.gpopsOptions;
    end
end
        
        
defaults = {'generateAdigatorFiles','automatic' %This will run the script automatically like normal set to 'manual' to just generate files
            'linkAdigatorFiles','manual'};      %Choose manual or automatic, auto will look at date codes etc and try to do it itself, issues when compiling
setDefaultsForVarargin(defaults,varargin);        



%% Set up switching
%Determine switching cost - needs to be above aux data 
ind = find(switchingDaq.rawData.distance.meas == s0,1);
cost = switchingDaq.rawData.switching.meas(ind,:);
  

%% Form problem structure
%Bounds - need to be first, sf used later
sf    =  s0+horizon;

%set up Aux data
auxdata.track = track ;
auxdata.vehicle = vehicle;
auxdata.cost = cost;
auxdata.costScaling = switchingDaq.rawData.costScaling.meas;




%% Formulate GPOPSII
%-------------------------------------------------------------------------%
%----------------------- Setup for Problem Bounds ------------------------%
%-------------------------------------------------------------------------%
bounds.phase.initialtime.lower  = s0; 
bounds.phase.initialtime.upper  = s0;
bounds.phase.finaltime.lower    = sf; 
bounds.phase.finaltime.upper    = sf;
bounds.phase.initialstate.lower = x0;
bounds.phase.initialstate.upper = x0;
bounds.phase.state.lower        = bounds.lbX; 
bounds.phase.state.upper        = bounds.ubX; 
bounds.phase.finalstate.lower   = bounds.lbX; 
bounds.phase.finalstate.upper   = bounds.ubX; 
bounds.phase.control.lower      = bounds.lbU; 
bounds.phase.control.upper      = bounds.ubU; 
bounds.phase.path.lower         = bounds.pathLower;
bounds.phase.path.upper         = bounds.pathUpper;
% bounds.phase.integral.lower     = bounds.integralLower;
% bounds.phase.integral.upper     = bounds.integralUpper;

%-------------------------------------------------------------------------%
%---------------------- Provide Guess of Solution ------------------------%
%-------------------------------------------------------------------------%
guess.phase.time    = timeGuess; 
guess.phase.state   = stateGuess;
guess.phase.control = controlGuess;
% guess.phase.integral = integralGuess;

%-------------------------------------------------------------------------%
%----------Provide Mesh Refinement Method and Initial Mesh ---------------%
%-------------------------------------------------------------------------%
mesh.method        = gpopsOptions.mesh.method;
mesh.tolerance     = gpopsOptions.mesh.tolerance;
mesh.maxiterations = gpopsOptions.mesh.maxiterations;
mesh.colpointsmin  = gpopsOptions.mesh.colpointsmin;
mesh.colpointsmax  = gpopsOptions.mesh.colpointsmax;

% mesh.method       = 'hp-PattersonRao';
% mesh.tolerance    = 1e-5;
% mesh.maxiterations = 1;
% mesh.colpointsmin = 7;
% mesh.colpointsmax = 20;

%-------------------------------------------------------------------------%
%------------- Assemble Information into Problem Structure ---------------%        
%-------------------------------------------------------------------------%
setup.name                        = gpopsOptions.setup.name;
%setup.functions.continuous        = gpopsOptions.setup.functions.continuous;
%setup.functions.endpoint          = gpopsOptions.setup.functions.endpoint;
setup.nlp.solver                  = gpopsOptions.setup.nlp.solver;
setup.derivatives.supplier        = gpopsOptions.setup.derivatives.supplier;%'adigator';%'sparseFD'; %'adigator';%
setup.derivatives.derivativelevel = gpopsOptions.setup.derivatives.derivativelevel;
setup.scales.method               = gpopsOptions.setup.scales.method;
setup.method                      = gpopsOptions.setup.method;
setup.displaylevel                = gpopsOptions.setup.displaylevel;

% setup.name                        = 'quadCar';
setup.functions.continuous        = @fourwheelContinuous;
setup.functions.endpoint          = @fourwheelEndpoint;
% setup.nlp.solver                  = 'ipopt';
% setup.derivatives.supplier        = 'adigator';%'adigator';%'sparseFD'; %'adigator';%
% setup.derivatives.derivativelevel = 'second';
% setup.scales.method               = 'automatic-hybridUpdate';
% setup.method                      = 'RPM-Differentiation';
% setup.displaylevel                = 2;



setup.auxdata                     = auxdata;
setup.bounds                      = bounds;
setup.guess                       = guess;
setup.mesh                        = mesh; 



%-------------------------------------------------------------------------%
%------------------------- Solve Problem Using GPOP2 ---------------------%
%-------------------------------------------------------------------------%
%% Adigator links:
switch linkAdigatorFiles
    case 'manual'
        setup.adigatorgrd.continuous    = @fourwheelContinuousADiGatorGrd;
        setup.adigatorgrd.endpoint      = @fourwheelEndpointADiGatorGrd;
        setup.adigatorhes.continuous    = @fourwheelContinuousADiGatorHes;
        setup.adigatorhes.endpoint      = @fourwheelEndpointADiGatorHes;
end


switch generateAdigatorFiles
    case 'manual' 
        adigatorfilenames = adigatorGenFiles4gpops2(setup);
        postProcessFlag = false;
    case 'automatic'
        tic
        output   = gpops2(setup);
        toc
        postProcessFlag = true;
end



%% Deal with output
if postProcessFlag
    %Error handling if problems
    if ismember(output.result.nlpinfo,gpopsOptions.acceptableNlpOutpus)% == 0 || output.result.nlpinfo == 1 % ~= 0 % == -1 %Did not converge
        convergence = true;
        if ~isa(gpopsOptions.specifyMeshIterationSolution,'char')
			output.result = output.meshhistory(gpopsOptions.specifyMeshIterationSolution).result;
		end
    else
        daq = [];
        convergence = false; 
        return
    end

    %Daq the solution
                                %   ePsi   ey   vx   vy   r  t
    daq = parseGpops2toDaq(output,gpopsOptions.stateNames,...
                                  gpopsOptions.controlNames,...
                                  gpopsOptions.indepVarName,...
                                  gpopsOptions.units,...
                                  gpopsOptions.names);
    daq.gpopsSetup = setup;
                                  
end %post prcess flag
end %short segment gpops









