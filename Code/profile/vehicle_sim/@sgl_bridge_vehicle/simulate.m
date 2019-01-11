function y = simulate(self)
%% simulate
% 
% 
% 
% author: John Braley
% create date: 2018-07-26 06:46:49.510
	
% input
u = [[0; diff(self.profile)/self.dt] self.profile]; % profile velocity and displacement

if isempty(self.x0)
    self.x0 = [0 0 0 -self.mt/self.kt]';% initial conditions of states (start with vehicle displaced by deadload)
end
% preallocate state space
z = zeros(length(self.time),length(self.x0));
z = [self.x0'; z];
% preallocate results
y = zeros(length(self.time),size(self.ssout{1}(1),1));

% get indices of vehicle on and off the bridge
vehicle_on = find(self.time>=0,1,'first');
vehicle_off = find(self.time>(self.L/self.vel),1,'first');
vehicle_mid = ceil((vehicle_off-vehicle_on)/2)+vehicle_on-1;
if rem(diff([vehicle_on vehicle_off]),2)==0; extra_ind = 1; else extra_ind = 0; end
% Compute each state
% Run before vehicle enters bridge
 % state space model
lssmodel = ss(self.ssmodel_off{1},[self.ssmodel_off{2} [0 0 -self.gravity 0]'],self.ssout{2},[self.ssout{3} [0 0 0 0 0]']);
if vehicle_on>1
    y(1:(vehicle_on-1),:) = lsim(lssmodel,[u(1:(vehicle_on-1),:) ones((vehicle_on-1),1)],self.time(1:(vehicle_on-1))-self.time(1),self.x0);
    z(2:(vehicle_on),:) = (y(1:(vehicle_on-1),1:4)'-self.ssout{3}(1:4,:)*u(1:(vehicle_on-1),:)')'/(self.ssout{2}(1:4,:));
end
% for ii = 1:(vehicle_on-1)
%    
%     % next state
%    z(ii+1,:) = self.ssfun(z(ii,:)',u(ii,:)',self.ssmodel_off{1},self.ssmodel_off{2},self.ssmodel_off{3})*self.dt+z(ii,:)';
%    
%    % Output   
%    y(ii,:) = self.ssout_fun(z(ii,:)',u(ii,:)',self.ssout{2},self.ssout{3}, self.ssout{4});
% end
for ii = vehicle_on:(vehicle_off-1)
   time = self.time(ii);      
   A = self.ssmodel{1}(time);
   B = self.ssmodel{2}(time);
   F = self.ssmodel{3};

   % next state
   z(ii+1,:) = self.ssfun(z(ii,:)',u(ii,:)',A,B,F)*self.dt+z(ii,:)';
   
   % Output   
   y(ii,:) = self.ssout_fun(z(ii,:)',u(ii,:)',self.ssout{1}(time),self.ssout{3});
end 

if vehicle_off<length(self.time)
    y(vehicle_off:length(self.time),:) = lsim(lssmodel,[u(vehicle_off:length(self.time),:) ones(length(self.time)-vehicle_off+1,1)],self.time(vehicle_off:length(self.time))-self.time(vehicle_off),z(vehicle_off,:)');
    z((vehicle_off+1):(length(self.time)+1),:) = (y(vehicle_off:length(self.time),1:4)'-self.ssout{3}(1:4,:)*u(vehicle_off:length(self.time),:)')'/(self.ssout{2}(1:4,:));
end
% for ii = vehicle_off:length(self.time)
%     % next state
%    z(ii+1,:) = self.ssfun(z(ii,:)',u(ii,:)',self.ssmodel_off{1},self.ssmodel_off{2},self.ssmodel_off{3})*self.dt+z(ii,:)';
%    
%    % Output   
%    y(ii,:) = self.ssout_fun(z(ii,:)',u(ii,:)',self.ssout{2},self.ssout{3}, self.ssout{4});
% end

% add static deflection and amplification to output
z_st = zeros(length(self.time),1);
z_theor = zeros(length(self.time),1);

z_st(vehicle_on:(vehicle_off-1)) = -2*self.mt*self.L^3/(pi^4*self.EI)*sin(pi*self.vel*self.time(vehicle_on:(vehicle_off-1))/self.L);
z_theor(vehicle_on:vehicle_mid) = (self.vel*self.time(vehicle_on:vehicle_mid))*-self.mt/(12*self.EI).*(3*self.L^2/4-(self.vel*self.time(vehicle_on:vehicle_mid)).^2);
z_theor((vehicle_off-1):-1:(vehicle_mid+extra_ind)) = (self.vel*self.time(vehicle_on:vehicle_mid))*-self.mt/(12*self.EI).*(3*self.L^2/4-(self.vel*self.time(vehicle_on:vehicle_mid)).^2);
z_amp = (y(:,1)-z_st)./min(z_st);
y = [y z_st z_amp z_theor];

% evaluate at location x
if ~isempty(self.x) && self.x~=self.L/2
y(:,[1:2 6]) = y(:,[1:2 6])*(sin(pi*self.x/self.L)); % according to shape function	
end
	
end
