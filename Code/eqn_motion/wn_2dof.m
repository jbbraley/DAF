function freqs = wn_2dof(m1, m2, k1, k2, max_mass)
%% wn_2dof
% 
% 
% 
% author: John Braley
% create date: 16-Jul-2018 16:21:11
	
% Undamped 2-DOF system
% k1 - spring stiffness of top sprung mass
% k2 - spring stiffness for bottom sprung mass
% m1 - mass of top sprung mass
% m2 - mass of bottom sprung mass
% max_mass = 1 if only the frequencies of the mode with the highest mass
% participation is to be returned

if nargin<5; max_mass = 0; end


a = m1*m2;
b = -k1*m2-k1*m1-k2*m1;
c = k1*k2;

w2(:,1) = (-b+sqrt(b^2-4*a*c))/(2*a);
w2(:,2) = (-b-sqrt(b^2-4*a*c))/(2*a);

w = sqrt(w2);
freqs = w/(2*pi);	
	
if max_mass
    if sqrt(k2/m2)<sqrt(k1/m1)*1.1
        freqs = freqs(2);
    else
        freqs = freqs(1);
    end
end
	
end
