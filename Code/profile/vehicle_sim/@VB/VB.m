classdef VB < handle & classio
%% classdef qcar
% https://www.omicsonline.org/open-access/analysis-of-automotive-passive-suspension-system-with-matlab-program-generation-0976-4860-4-115-119.pdf
% 
% author: John Braley
% create date: 12-Oct-2017 12:27:24
% classy version: 0.1.2

%% object properties
	properties        
        profile         % elevation profile over which the model traverses (in);
        dist            % vector specifying location of each profile point (in)
        vel             % vehicle speed (in/sec)
        kt               % suspension stiffness (lb/in)
        ct               % damping coefficient (lb-s/in)
        kb              % bridge stiffness (lb/in)
        cb              % bridge damping coefficient (lb-s/in)
        mt              % mass of truck (lb)
        mb              % mass of bridge (lb)
        x0 = [0 0 0 0]';% initial conditions of states
        gravity = 386.09; % in/sec^2       
	end

%% dependent properties
	properties (Dependent)
       time            % time vector for each step of simulation
       ssmodel         % state space model
	end

%% private properties
	properties (Access = private)
	end

%% constructor
	methods
		function self = qcar2()
		end
	end

%% ordinary methods
	methods 
	end % /ordinary

%% dependent methods
	methods 
        function time = get.time(self)
        %% time - get time based on distance and velocity
            if isempty(self.dist) || isempty(self.vel)
                time = [];
            else
                if any(round(diff(self.dist),5)~=round(diff(self.dist(1:2)),5)) || self.dist(1)~=0
                    fprintf('distance must start from zero and be evenly spaced, constructing new distance vector based on average distance step\n')
                    dx = round(mean(diff(self.dist)),5);
                    self.dist = 0:dx:(length(self.dist)-1)*dx;
                end
                time = self.dist/self.vel;
            end
        end
        
        function ssmodel = get.ssmodel(self)
            %% Construct linear state space model 
            Aqcar = [-self.ct/(self.mt/self.gravity) -self.kt/(self.mt/self.gravity) self.ct/(self.mt/self.gravity) self.kt/(self.mt/self.gravity);...
                    1 0 0 0;... 
                    self.ct/(self.mb/self.gravity) self.kt/(self.mb/self.gravity) -(self.ct+self.cb)/(self.mb/self.gravity) -(self.kt+self.kb)/(self.mb/self.gravity);...
                    0 0 1 0]; 
            Bqcar = [self.ct/(self.mt/self.gravity) self.kt/(self.mt/self.gravity);...
                    0 0;...
                    -self.ct/(self.mb/self.gravity) -self.kt/(self.mb/self.gravity);...
                    0 0]; 
            %% Output Matrix
            Cqcar = [0 1 0 0;... % sprung mass displacement
                    1 0 0 0;...     % vehicle velocity
                    0 0 0 1;...  % bridge displacement
                    0 0 1 0];    % bridge velocity
            Dqcar = [0];
%                        
            ssmodel = ss(Aqcar,Bqcar,Cqcar,Dqcar);
        end
	end % /dependent

%% static methods
	methods (Static)
	end % /static

%% protected methods
	methods (Access = protected)
	end % /protected

end
