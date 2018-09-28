classdef SDOF_P < handle
%% classdef SDOF_P
% 
% 
% 
% author: John Braley
% create date: 2018-07-25 03:59:13.127
% classy version: 0.1.2

%% object properties
	properties
        k               % stiffness (lb/in)
        c               % damping coefficient (lb-s/in)
        ms              % mass of sprung mass (lb)
        p               % applied force (lb)
        dt              % time step for each element of force vector
        x0 = [0 0]';% initial conditions of states
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
		function self = SDOF_P()
		end
	end

%% ordinary methods
	methods 
	end % /ordinary

%% dependent methods
	methods 
        function time = get.time(self)
            time = 0:self.dt:(length(self.p)-1)*self.dt;
        end
        
        function ssmodel = get.ssmodel(self)
            %% Construct linear state space model 
            Aqcar = [-self.c/(self.ms/self.gravity) -self.k/(self.ms/self.gravity); 1 0]; 
            Bqcar = [1/(self.ms/self.gravity); 0]; 
            %% Output Matrix
            Cqcar = [0 1;... % sprung mass displacement
                    1 0];    % sprung mass velocity
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
