classdef ss_bridge < handle
%% classdef ss_bridge
% 
% models a simply supported beam subjected to a time varying point load
% moving over the bridge with a constant velocity
% 
% author: John Braley
% create date: 2018-07-26 02:59:25.641
% classy version: 0.1.2

%% object properties
	properties
        EI               % flexural rigidity (lb-in^2)
        L               % length of bridge (in)
        ms              % mass of sprung mass (lb)
        p               % applied force (lb)
        dt              % time step for each element of force vector
        vel             % velocity of load
        x = [];              % location on bridge to compute response
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
		function self = ss_bridge()
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
            A = [0 -pi^4*self.EI/(self.L^3*self.ms/self.gravity);...
                1 0]; 
            B = [2/(self.ms/self.gravity); 0]; 
            %% Output Matrices (y=Cx+Du)
            C = [0 1; ... % bridge displacement (at midspan)
                    1 0];    % bridge velocity (at midspan)
            D = [0];
%                        
            ssmodel = ss(A,B,C,D);
        end
	end % /dependent

%% static methods
	methods (Static)
	end % /static

%% protected methods
	methods (Access = protected)
	end % /protected

end
