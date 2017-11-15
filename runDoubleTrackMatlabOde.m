function daq = runDoubleTrackMatlabOde(s,x0,u,auxdata)
clc

if nargin == 0
    carFileName = 'LimebeerF1Car.mat';
    vehicleDirectory = fullfile(jatecPath,'Resources/In house code/Vehicle Parameters/');
    fullVehicleFile = fullfile(vehicleDirectory,'Optimal Control Research',carFileName);
    load(fullVehicleFile);
    s0          = 0;                                             %[m] s
    sf          = 10;
    s = s0:1:sf;
    u = 1000*ones(size(s));
    vx0 = 10;
    omega_front0 = -vx0./vehicle.tire_front.reff.meas;
    x0 = [vx0 omega_front0];
    auxdata.vehicle = vehicle;
end

%% Run simulation
sSpace = s;
sSpan = [s(1) s(end)]; %#ok s will be an array



options = odeset('MaxStep',1);
[sSol,xSol] = ode23tb(@(s,x)fourwheelMatlabODE(s,x,u,sSpace,auxdata), sSpan, x0,options);

stateNames = {'ePsi';'ey';'vx';'vy';'r';'time';'omega_L1';'omega_R1';'omega_L2';'omega_R2';'delta';'torqueDemand'}';
controlNames = {'u1';'u2'}';
units = {'rad';'m';'m/s';'m/s';'rad/s';'s';'rad/s';'rad/s';'rad/s';'rad/s';'rad';'N*m';'-';'-';'m'};
names = {'Heading Deviation','Lateral Deviation','Longitudinal Speed','Lateral Speed','Yaw Rate','time',...
       'Wheel Speed Left Front','Wheel Speed Right Front','Wheel Speed Left Rear','Wheel Speed Right Rear','Steering Wheel Angle','Torque Demand','Distance','Steering Rate','Torque Demand Rate'};
     
%Resample to requested input
SSOL = s;
for i = 1:size(xSol,2)
    XSOL(:,i) = interp1(sSol,xSol(:,i),SSOL);
end


daq = parseMatlabOdeOutput(SSOL,XSOL,u,'distance',stateNames,controlNames,units,names);
gpopsOutput.result.solution.phase.time = SSOL;
gpopsOutput.result.solution.phase.state = XSOL;
gpopsOutput.result.solution.phase.control = u;
gpopsOutput = addNotesToDaqFile(gpopsOutput,'Just added this to mimic gpops output for calculating algebraic states');
daq.gpopsOutput = gpopsOutput; 
daq.vehicle = auxdata.vehicle;
if nargin == 0
    daq.vehicle = auxdata.vehicle;
%     daq = mathChannelCalculateAlgebraicStates(daq);

     quickPlotTable(daq)

end
