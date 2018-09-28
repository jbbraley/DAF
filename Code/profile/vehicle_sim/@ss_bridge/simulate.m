function y = simulate(self)
%% simulate
% 
% returns (y) the state space time history
% 
% author: John Braley
% create date: 12-Oct-2017 12:39:52

% run simulation
y = lsim(self.ssmodel, self.p.*sin(pi*self.vel*self.time'/self.L), self.time,self.x0);	

% evaluate at location x
% evaluate at location x
if ~isempty(self.x) && self.x~=self.L/2
y(:,1:2) = y(:,1:2).*(sin(pi*self.x/self.L)*ones(1,2)); % according to shape function	
end
end
