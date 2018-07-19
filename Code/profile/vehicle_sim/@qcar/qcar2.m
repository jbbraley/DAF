classdef qcar < handle & classio
%% classdef qcar
% 
% 
% 
% author: John Braley
% create date: 12-Oct-2017 12:27:24
% classy version: 0.1.2

%% object properties
	properties        
        profile         % velocity profile over which the model traverses [0; diff(elevation_profile)/dt];
        dist            % vector specifying location of each profile point (m)
        vel               % vehicle speed (m/sec)
        k               % suspension stiffness (N/m)
        c               % damping coefficient (N.s/m)
        ms              % mass of main sprung mass (kg)
        mus             % mass of tire (or first sprung mass) (kg)
        kus             % stiffness of tire (or first sprung mass)
        x0 = [0 0 0 0]';% initial conditions of states
        gravity = 9.81; % m/sec        
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
		function self = qcar()
		end
	end

%% ordinary methods
	methods 
	end % /ordinary

%% dependent methods
	methods 
        function time = get.time(self)
        %% time - get time based on distance and velocity
            if any(round(diff(self.dist),5)~=round(diff(self.dist(1:2)),5)) || self.dist(1)~=0
                fprintf('distance must start from zero and be evenly spaced, constructing new distance vector based on average distance step\n')
                dx = round(mean(diff(self.dist)),5);
                self.dist = 0:dx:(length(self.dist)-1)*dx;
            end
            time = self.dist/self.vel;
        end
        
        function ssmodel = get.ssmodel(self)
        %% Construct linear state space model 
            Aqcar = [0 0 0 1; 0 0 1 -1; 0 -self.k/self.ms -self.c/self.ms self.c/self.ms; -self.kus/self.mus self.k/self.mus self.c/self.mus -self.c/self.mus];
            Bqcar = [0 0 0 self.kus/self.mus]'; 
            % output matrix
            Cqcar = [1 1 0 0;... % sprung mass displacement
                    0 0 1 0;... % sprung mass velocity
                    1 0 0 0;... % unsprung mass displacement
                    0 0 0 1;... % unsprung mass velocity
                    0 1 0 0];   % suspension travel
            Dqcar = 0;
            
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
