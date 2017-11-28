function dx = fourwheelMatlabODE(s,x,u,sSim,auxdata,varargin)

defaults = {'suppressOutput',false};
setDefaultsForVarargin(defaults,varargin);


u = interp1(sSim,u,s);

input.phase.time = s;
input.phase.state = x';
input.phase.control = u;
input.auxdata = auxdata;
phaseout = fourwheelContinuous(input);
dx = phaseout.dynamics';


if ~suppressOutput
    persistent count
    if isempty(count); count = 1; end
    
    %What vars do you want to print
    vars = {'s', 'x(1)'};
    headerText = [];
    dataText = [];
    for i = 1:length(vars)
        headerText = [headerText sprintf('%-5s ',vars{i})];
        dataText   = [dataText   sprintf('%5.1f ',eval(vars{i}))];
    end

    if count == 1
        fprintf('%s\n',headerText)
    end
    count = count+1;
    if count == 21
        count = 1;
    end
    fprintf('%s\n',dataText)
end
