function y = simulate(self)
%% simulate
% 
% returns (y) the state space time history
% 
% author: John Braley
% create date: 12-Oct-2017 12:39:52

% run simulation

y = lsim(self.ssmodel,self.profile,self.time,self.x0);	
	
end
