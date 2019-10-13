function [ setIdx, trialIdx] = clearClasses()
current_breakpoints = dbstatus('-completenames');
try
    setIdx   = 0;
    trialIdx = 0;
    if(evalin('base','exist(''setIdx'')') )
        setIdx = evalin('base', 'setIdx');
    end
    if(evalin('base','exist(''trialIdx'')') )
        trialIdx = evalin('base', 'trialIdx');
    end
    %   manager = evalin('base', 'manager');
catch me
    %   manager = 0;
end
evalin('base', 'clear classes');
%evalin('base', ['manager = ',num2str(manager)]);
% get around bug where red icons are not displayed
% by pausing for a short time
pause(0.1)
dbstop(current_breakpoints);
end