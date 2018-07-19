function y = simulate(self)
%% simulate
% 
% returns (y) the state space time history
% 
% author: John Braley
% create date: 12-Oct-2017 12:39:52

% run simulation
 dt = diff(self.time(1:2));
y = lsim(self.ssmodel,[[0; diff(self.profile)/dt] self.profile],self.time,self.x0);	
	
end
