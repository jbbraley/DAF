function y = simulate(self)
%% simulate
% 
% 
% 
% author: John Braley
% create date: 2018-07-26 06:46:49.510
	
% input
u = [[0; diff(self.profile)/self.dt] self.profile]; % profile velocity and displacement
% preallocate state space
z = zeros(length(self.time),length(self.x0));
z = [self.x0'; z];
% preallocate results
y = zeros(length(self.time),size(self.ssout{1}(1),1));

% Compute each state
for ii = 1:length(self.time)
   time = self.time(ii);
   A = self.ssmodel{1}(time);
   B = self.ssmodel{2}(time);
   F = self.ssmodel{3};
   % next state
   z(ii+1,:) = self.ssfun(z(ii,:)',u(ii,:)',A,B,F)*self.dt+z(ii,:)';
   
   % Output
   C = self.ssout{1}(time);
   y(ii,:) = self.ssout_fun(z(ii,:)',u(ii,:)',C,self.ssout{2});
    
end


% evaluate at location x
if ~isempty(self.x) && self.x~=self.L/2
y(:,1:2) = y(:,1:2).*(sin(pi*self.x/self.L)*ones(1,2)); % according to shape function	
end
	
end
