%-------------------------------------------%
% BEGIN: function particleMotionEndpoint.m %
%-------------------------------------------%
function output = fourwheelEndpoint(input)
cost = input.auxdata.cost;
costScaling = input.auxdata.costScaling;

xTerminal = [input.phase(1).finalstate(3) input.phase(1).finalstate(6)];
X         = (xTerminal./costScaling).^2;
X(1)      = -X(1); %Vx term should be a maximizer


J = cost*X';

output.objective = J;

end
%-------------------------------------------%
% END: function brachistochroneEndpoint.m   %
%-------------------------------------------%
