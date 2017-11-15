function daq = calculateAlgebraicStates(daq)
%This funciton will throw the inputs and states back through the ODE and
%calculate the algebraic states in the fourwheelContinuous model from
%Gpops2. 
%INPTUS:
%    daq (optional) - local instance of the daq file
%OUTPUTS:
%    daq (optional) - updated local instnace of the daq file
%
%Creation: 12 Nov 2017 - Jeff Anderson
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


output = daq.gpopsOutput;

input.phase.time = output.result.solution.phase.time;
input.phase.state = output.result.solution.phase.state;
input.phase.control = output.result.solution.phase.control;

input.auxdata.vehicle = daq.vehicle;
input.auxdata.track = daq.header.track;
phaseout = fourwheelContinuousForMatlab(input);

algebraicStates = fieldnames(phaseout.algebraicStates);
for i = 1:length(algebraicStates)
    ch = algebraicStates{i};
    daq.rawData = addMathChannelsThatAreStandardChannels(daq.rawData,ch,phaseout.algebraicStates.(ch).meas);
end