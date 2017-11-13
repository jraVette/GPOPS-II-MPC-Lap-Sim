function setDefaultsForVarargin(defaults,varargin)
%INPUTS:
%    defaults - is a nx2 cell array where the first col is the parameters
%               and the 2nd col is the value for the defautls to be set
%                                                            class cell nx2
%    varargin - this is the key value varargin cell array to be parsed to
%               override the defaults                        class cell nx1
%
% I started using a lot of key, value pairs to override default behavoirs,
% so I wrote this function to help code be more concise
% 
% Example: w/in the parent function:
%   function out=myExmaple(varargin)
%       defaults = {'suppressPlots',true};
%       setDefaultsForVarargin(defaults,varargin)
%   >> myExample('suppressPlots',false)  
%   - This will overrite the behavior in myExample to not plot
%
% Creation: 13 March 2015 - Jeff Anderson   
% Updated:   6 Oct   2015 - Jeff Anderson - bug fixes
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Argument checking - defaults must be nx2 cell
badInput = false;
if isa(defaults,'cell')
    [~,nCols] = size(defaults);
    if nCols ~= 2
        badInput = true;
        error('Default must be an nx2 cell arrary\n')
    end
else
    badInput = true;
    error('Default must be cell arrary\n')
end
if badInput; return; end

%Because this double nests varargin, we must fix that
varargin = varargin{1};

%Assign defaults in the calling function
keys = defaults(:,1);                                                      %
values = defaults(:,2);

for i = 1:length(keys)
    key = keys{i};
    value = values{i};
    assignin('caller',key,value)
end

%Loop through the varargin and overwrite defaults if necessary
if isempty(varargin); return; end
count = 1;                      
while count <= length(varargin)
    if ~isempty(varargin{count})
        if ismember(varargin{count},keys)
            key = varargin{count};
            value = varargin{count+1};
            assignin('caller',key,value)
        else
            warning('Key not recognized: %s',varargin{count});
        end
    end
    count = count+2; %key value pairs
end


end%setDefaultsForVarargin

