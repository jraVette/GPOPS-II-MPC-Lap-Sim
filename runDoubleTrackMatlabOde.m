close all
clc

%Inport parameters
daq = generateInitialDaq('loadInitialGuess',false,'loadBounds',false);
auxdata.vehicle = daq.header.vehicle;
auxdata.track   = daq.header.track;
clear daq

%Define parameters for sim
s = rowVector(0:0.25:150);
u1 = rowVector(0*myConstants.deg2rad*ones(size(s)));
% u1(end-20:end) = 0*myConstants.deg2rad*ones(size(u1(end-20:end)));
u2 = 1000*ones(size(s)) %(2700*ones(size(s))+10*s)/4000;
u= [u1 u2];

%% Set up plots
plotOpt = 0
if plotOpt
    m           = auxdata.vehicle.m;      %Total vehicle mass                             [kg] 
    a           = auxdata.vehicle.a ;      %Length of front axle to CG                     [m]
    b           = auxdata.vehicle.b;      %Length of rear axle to CG                      [m]
    fzf = b*m*9.81/(a+b)*0.5;
    fzr = a*m*9.81/(a+b)*0.5;
    

    slipRatio = -1:0.01:1;
    hFig = figure('Name','Tire Fx','Position',[28   536    560   420]);
    tags = {'L1','R1','L2','R2'};
    for i = 1:4
        if i == 1 || i ==2
            fz = fzf;
        else
            fz = fzr;
        end
        [fx,fy] = simplifiedPacejka(fzf,0,slipRatio,auxdata.vehicle.tire.coeff);
        subplot(2,2,i)
        plot(slipRatio,fx,'linewidth',1.5,'Tag','tireTrace'); hold all
        plot(slipRatio,fx,'--k','linewidth',1.5); hold all
        plot(0,0,'k+','linewidth',1.5,'markersize',10,'Tag','cursor')
        grid on
        set(gca,'Tag',tags{i})
        xlim([-1 1])
        ylim([-10000 10000])        
        title(sprintf('SA = %4.2fdeg, SR = %4.2f, FZ = %5.0fN',0,0,fz));
    end
    
    
    slipAngle = (-15:0.1:15)*myConstants.deg2rad;
    pos = getFigurePositionRelativeToFig(hFig,1,1,1,0);
    hFig2 = figure('Name','Tire Fy','Position',pos);
    tags = {'L1','R1','L2','R2'};
    for i = 1:4
        if i == 1 || i ==2
            fz = fzf;
        else
            fz = fzr;
        end
        [fx,fy] = simplifiedPacejka(fzf,slipAngle,0,auxdata.vehicle.tire.coeff);
        subplot(2,2,i)
        plot(slipAngle*myConstants.rad2deg,fy,'linewidth',1.5,'Tag','tireTrace'); hold all
        plot(slipAngle*myConstants.rad2deg,fy,'--k','linewidth',1.5); hold all
        plot(0,0,'k+','linewidth',1.5,'markersize',10,'Tag','cursor')
        grid on
        set(gca,'Tag',tags{i})
        ylim([-10000 10000])
        title(sprintf('SA = %4.2fdeg, SR = %4.2f, FZ = %5.0fN',0,0,fz));
    end
    
%     pos = getFigurePositionRelativeToFig(hFig2,1,1,1,0);    
%     figure('Name','Trajectory')
%     dist = auxdata.track.distance.meas;
%     k = auxdata.track.curvature.meas;
%     trackData.rawData = frenetToXYFrameTrajectory(k,dist,zeros(size(dist)),zeros(size(dist)),0,0,0,10,'suppressPlot',true);
%     getChannelDataFromDaqFile(trackData);
%     plot(xPath,yPath,'--','Color',[240/255 255/255 40/255],'linewidth',1.5); hold on
%     plot(xRoadUp,yRoadUp,'k-','linewidth',1.5)
%     plot(xRoadLow,yRoadLow,'k-','linewidth',1.5)
%     grid on
%     xlabel('X [m]')
%     ylabel('Y [m]')
%     line(s(1),0,'marker','+','markersize',12,'linewidth',1.5,'Tag','cursor');
    



end %if plotOpt



%% Run simulation
sSpace = s;
sSpan = [s(1) s(end)]; %#ok s will be an array

%Initial conditiosn
vx_0 = 20;
omega_0 = -vx_0/0.33;
     % epsi  ey  vx    vy  r   time    omega_L1 omega_R1 omega_L2 omega_R2   dekta0  T0
x0 = [ 0     0   vx_0  0   0   0       omega_0  omega_0  omega_0  omega_0    0       0];
options = odeset('MaxStep',1);
[sSol,xSol] = ode15s(@(s,x)doubleTrackMatlabODE(s,x,u,sSpace,auxdata), sSpan, x0,options);
uSol = interp1(s,u,sSol);
stateNames = {'ePsi'     'ey'      'vx'      'vy'   'r'     'time'  'omega_L1' 'omega_R1' 'omega_L2' 'omega_R2','delta','torqueDemand'};
controlNames = {'u1' 'u2'};
units =      {'m' 'rad' 'm'  'm/s' 'm/s'  'rad/s' 's' 'rad/s' 'rad/s' 'rad/s' 'rad/s' 'rad' 'N*m'  'rad/s' 'N*m/s'};
names = {'Distance','Heading Deviation','Lateral Deviation','Longitudinal Speed','Lateral Speed','Yaw Rate','Time',...
         'Wheel Speed Left Front','Wheel Speed Right Front','Wheel Speed Left Rear','Wheel Speed Right Rear',...
         'Steering Wheel Angle','Torque','Lateral Control','Longitudinal Control'};
daq = parseMatlabOdeOutput(sSol,xSol,uSol,'distance',stateNames,controlNames,units,names);

 quickPlotTable(daq)


