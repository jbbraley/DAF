function [mass, EI] = bridge2beam(freq, K_mid, span_length, spans)
%% bridge2beam
% takes bridge attributes and returns beam mass and stiffness parameters
% 
% Input: 
%       freq - frequency (Hz) of first (bending) mode of vibration
%       K_mid - stiffness due to concentrated load at midspan (input_force/deflection)
%       spans - number of spans
%
% author: John Braley
% create date: 2018-11-06 03:28:31.678

switch spans
    case 1
        % single span
        EI = K_mid*span_length^3/48;
        mass = K_mid*pi^2/(48*4*freq^2);
    case 2    
        % 2-span continuous
        EI = K_mid*23*span_length^3/1536;
        mass = EI*pi^2/((2*freq)^2*span_length^3);
end
    
	
	
	
	
end
