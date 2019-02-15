function [EI] = bridge2beam2(freq, mass, span_length, spans)
%% bridge2beam
% takes bridge attributes and returns beam stiffness parameters
% 
% Input: 
%       freq - frequency (Hz) of first (bending) mode of vibration
%       mass - mass of bridge (force_time^2/dist; e.g. lb-s^2/in)
%       span_length - length of a single span (in)
%       spans - number of spans
%
% author: John Braley
% create date: 2018-11-06 03:28:31.678

switch spans
    case 1
        % single span
        EI = (4*freq^2/pi^2)*mass*span_length^3;
    case 2    
        % 2-span continuous
        EI = mass*(2*freq/pi)^2*span_length^3;
end
    
	
	
	
	
end
