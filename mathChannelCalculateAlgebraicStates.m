function daq = mathChannelCalculateAlgebraicStates(daq)
%This function will add math channels for all the algebric states sovled in
%the optimal control porblem
%
%INPUTS:
%    daq - (optional) local instance of a daq file
%
%OUTPUTS
%    daq - if local instance is used...the updated daq
%
%Creation: 12 Apr 2016 - Jeff Anderson
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


if ~exist('daq','var'); daq = []; end;
DAQ = useLocalOrGlobalDaqFiles(daq);



for iFile = 1:length(DAQ)
    daq = DAQ{iFile};
    fprintf('Adding channels to %s\n',displayDaqFiles(daq,'suppressOutput',true));
    
    %Get vehicle parameters
    extractVariablesFromSructure(daq.header.vehicle);
    coeff = tire.coeff;
    
    
    %% Slip angles and ratios
    vars = {'deltaFront'    'delta' 
            'omega_L1'      'omega_L1'
            'omega_R1'      'omega_R1'
            'omega_L2'      'omega_L2'
            'omega_R2'      'omega_R2'
            'r'             'r'
            'vx'            'vx'
            'vy'            'vy'   
            'time'          'time'
            'u2'            'u2'};
   
    daq = getChannelDataFromDaqFile(daq,vars);
    
    
    %Differentiate vx and vy to get accles
    ax = [0; diff(vx)./diff(time)];
    ay = [0; diff(vy)./diff(time)];
    
    %Aero loads      
    Faz =  0.5*CL*rho*frontalArea*vx.^2;
    Fax = -0.5*CD*rho*frontalArea*vx.^2;

    FxSS = m*ax;
    FySS = m*ay;

    %Loads (based on teh work in Tremlett)
    Fz_L1       = (1/2)*b*(-g*m-Faz)/(b+a)-(1/2)*D*h*FySS/(D*wf-D*wr+wr)-(1/2)*(-h*FxSS-(a_a-a)*Faz)/(b+a);
    Fz_R1       = (1/2)*b*(-g*m-Faz)/(b+a)+(1/2)*D*h*FySS/(D*wf-D*wr+wr)-(1/2)*(-h*FxSS-(a_a-a)*Faz)/(b+a);
    Fz_L2       = (1/2)*a*(-g*m-Faz)/(b+a)+(1/2)*(D-1)*h*FySS/(D*wf-D*wr+wr)+(1/2)*(-h*FxSS-(a_a-a)*Faz)/(b+a);
    Fz_R2       = (1/2)*a*(-g*m-Faz)/(b+a)-(1/2)*(D-1)*h*FySS/(D*wf-D*wr+wr)+(1/2)*(-h*FxSS-(a_a-a)*Faz)/(b+a);
    
    %Slip Ratios 
    kappa_L1    = -(1 + reff_f*omega_L1./(cos(deltaFront).*(vx + r*wf) + sin(deltaFront).*(r*a + vy))); 
    kappa_R1    = -(1 + reff_f*omega_R1./(cos(deltaFront).*(vx - r*wf) + sin(deltaFront).*(r*a + vy)));
    kappa_L2    = -(1 + reff_r*omega_L2./(vx + r*wr)); 
    kappa_R2    = -(1 + reff_r*omega_R2./(vx - r*wr));                                     

    %Slip angles:
    alpha_L1    = atan((-sin(deltaFront).*(vx + r*wf) + cos(deltaFront).*(vy + r*a))./(cos(deltaFront).*(vx + r*wf) + sin(deltaFront).*(vy + r*a)));
    alpha_R1    = atan(( sin(deltaFront).*(r*wf - vx) + cos(deltaFront).*(vy + r*a))./(cos(deltaFront).*(vx - r*wf) + sin(deltaFront).*(vy + r*a)));
    alpha_L2    = atan((vy - r*b)./(vx + r*wr));
    alpha_R2    = atan((vy - r*b)./(vx - r*wr));

    
    %Tire forces
    [fx_L1, fy_L1] = simplifiedPacejka(-Fz_L1,-alpha_L1,kappa_L1,coeff); %Fix because Pacejka is in ISO-W
    [fx_R1, fy_R1] = simplifiedPacejka(-Fz_R1,-alpha_R1,kappa_R1,coeff);
    [fx_L2, fy_L2] = simplifiedPacejka(-Fz_L2,-alpha_L2,kappa_L2,coeff);
    [fx_R2, fy_R2] = simplifiedPacejka(-Fz_R2,-alpha_R2,kappa_R2,coeff);
    
    %Torques
    u2Plus   = 0.5+0.5*sin(atan(100*u2));
    u2Minus  = 0.5-0.5*sin(atan(100*u2));
    kt = u2Plus*torqueDrivingRear + u2Minus*torqueBrakingRear;

    T_drive_L1 = (1-kt).*(u2*u2Gain)/(2);
    T_drive_R1 = (1-kt).*(u2*u2Gain)/(2);
    T_drive_L2 = (kt).*(u2*u2Gain)/(2) + kd*(omega_L2 - omega_R2);
    T_drive_R2 = (kt).*(u2*u2Gain)/(2) - kd*(omega_L2 - omega_R2);
    


    
    %Engine demand
    rearWheelSpeed = (abs(omega_L1)+abs(omega_L1))/2;
    powerUsage = ((u2*u2Gain).*rearWheelSpeed )/maxEnginePower;

    
    %Calculate %max and efficienty according to kelly p.203
    posSuffix = {'_L1','_R1','_L2','_R2'};
    for iPos = 1:length(posSuffix)
        fz = eval(['Fz' posSuffix{iPos}]);
        muX_max  = interp1([coeff.Fz1 coeff.Fz2],[coeff.muX1   coeff.muX2],  fz,'linear','extrap');
        muY_max  = interp1([coeff.Fz1 coeff.Fz2],[coeff.muY1   coeff.muY2],  fz,'linear','extrap');
        fxMax = muX_max.*fz;
        fyMax = muY_max.*fz;
        fx = eval(['fx' posSuffix{iPos}]);
        fy = eval(['fy' posSuffix{iPos}]);
        fxNorm = fx./fxMax;
        fyNorm = fy./fyMax;
        saturation = sqrt(fxNorm.^2 + fyNorm.^2);
        varName = ['tireSaturation' posSuffix{iPos}];
        eval([varName '= saturation']);
    end
    
    
    
    %Add the channels back
    notes = sprintf('This channel was created by mathChannelCalcualteAlgebraicStates.m on %s',datestr(now,'YYYY-mm-dd'));
    channelsToAdd = {'sa_L1'        alpha_L1    notes
                     'sa_R1'        alpha_R1    notes
                     'sa_L2'        alpha_L2    notes
                     'sa_R2'        alpha_R2    notes
                     'slipRatio_L1' kappa_L1    notes
                     'slipRatio_R1' kappa_R1    notes
                     'slipRatio_L2' kappa_L2    notes
                     'slipRatio_R2' kappa_R2    notes
                     'fx_L1'        fx_L1       notes
                     'fx_R1'        fx_R1       notes
                     'fx_L2'        fx_L2       notes
                     'fx_R2'        fx_R2       notes
                     'fy_L1'        fy_L1       notes
                     'fy_R1'        fy_R1       notes
                     'fy_L2'        fy_L2       notes
                     'fy_R2'        fy_R2       notes
                     'fz_L1'        Fz_L1       notes
                     'fz_R1'        Fz_R1       notes
                     'fz_L2'        Fz_L2       notes
                     'fz_R2'        Fz_R2       notes
                     'totalDownforce'   Faz     notes      
                     'dragForce'        Fax     notes
                     'tireSaturation_L1' tireSaturation_L1 notes
                     'tireSaturation_R1' tireSaturation_R1 notes
                     'tireSaturation_L2' tireSaturation_L2 notes
                     'tireSaturation_R2' tireSaturation_R2 notes
                     'wheelTorque_L1'  T_drive_L1 notes
                     'wheelTorque_R1'  T_drive_R1 notes
                     'wheelTorque_L2'  T_drive_L2 notes
                     'wheelTorque_R2'  T_drive_R2 notes
                     'powerUsage'	powerUsage  notes
                     'ax'         ax   notes
                     'ay'         ay   notes};
    for iCh = 1:length(channelsToAdd)
        daq.rawData = addMathChannelsThatAreStandardChannels(daq.rawData,channelsToAdd{iCh,1},channelsToAdd{iCh,2},channelsToAdd{iCh,3});
    end
    
    fprintf('Complete on file: %s\n',displayDaqFiles(daq,'suppressOutput',true));
    disp(' ')
                     
                     
                 


    
    DAQ{iFile} = daq;
end

