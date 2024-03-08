%---------------------------------------------------------------------------------------------------
% For Paper
% "Distributed Flocking Control with Ellipsoidal Level Sets"
% by P. Hastedt, A. Datar, K. Kocev, and H. Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Philipp Hastedt
%---------------------------------------------------------------------------------------------------

classdef EllipticTAgent < DoubleIntegratorAgent
    
    % member variables
    properties(GetAccess = private, SetAccess = private)
        d;          % desired distance
        r;          % interaction range
        h;          % bump function parameter
        tau;        % bump function parameter
        s;          % sigma norm parameter
        c_alpha;    % alpha flocking tuning parameter
        c_gamma;    % gamma flocking tuning parameter

        T_normalized;           % normalized transformation matrix 
        normalizationFactor;    % normalization factor

        d_sigma;    % sigma norm of d
        r_sigma;    % sigma norm of r

        m;              % space dimension
        numberAgents;   % number of agents
        reference;      % gamma agent
    end
    
    % data to be transmitted in addition to position, velocity, and u
    properties
        num_N;      % number of neighbors
        neighbors;  % ids of neighbors        
    end
    
    % methods
    methods
        [y, gradient] = sigma_T_norm(obj, z, T)
    end
    
    methods
        function obj = EllipticTAgent(id, param, initialPos, initialVel, cfg)
            % call parent constructor
            obj@DoubleIntegratorAgent(id, param.dT, initialPos, initialVel);
            
            % calculate normalized transformation matrix and normalization factor
            [~,obj.T_normalized,obj.normalizationFactor] = calculateEllipsoidalTransformation(cfg.ellipseAxes, cfg.ellipseRotation);
                       
            % load parameters
            obj.d = cfg.d*obj.normalizationFactor;
            obj.r = param.range*obj.normalizationFactor;
            obj.h = cfg.h;
            obj.s = cfg.s;  
            obj.c_alpha = cfg.c_alpha;
            obj.c_gamma = cfg.c_gamma;
            obj.m = cfg.m;
            obj.numberAgents = param.agentCount;

            % calculate sigma values
            obj.d_sigma = sigma_norm(obj.d, obj.s);
            obj.r_sigma = sigma_norm(obj.r, obj.s);
            
            % initialize member variables
            obj.num_N = 0;
            obj.neighbors = zeros(obj.numberAgents,1);
            
            % set reference
            if numel(param.reference) == 0
                obj.reference = [];
            elseif numel(param.reference) == 2*obj.m
                obj.reference = param.reference;
            else
                obj.reference = param.reference(:,id);
            end
        end
        
        function step(obj)
            u = zeros(obj.m, 1);
            obj.neighbors = zeros(obj.numberAgents,1);
            % Receive messages from the network
            messages = obj.receive();
            obj.num_N = length(messages);
            
            % alpha flocking component
            if obj.num_N ~= 0
                for message = messages
                    obj.neighbors(message.data.id) = 1;
                    % compute sigma distance and gradient
                    qij = message.data.position - obj.position;
                    pij = message.data.velocity - obj.velocity;
                    [qij_sigma, grad_qij_sigma] = obj.sigma_T_norm(qij,obj.T_normalized);
                    
                    % calculate u_alpha_q
                    u_alpha_q = phi_alpha(qij_sigma, obj.r_sigma, obj.d_sigma, obj.h, obj.c_alpha, obj.c_alpha) * grad_qij_sigma;
                    % calculate u_alpha_p
                    aij = rho_h(qij_sigma / obj.r_sigma, obj.h);
                    u_alpha_p = aij * pij;
                    
                    % add to overall input
                    u = u + u_alpha_q+ u_alpha_p;
                end
            end
            
            % gamma flocking component
            if ~isempty(obj.reference)
                qir = obj.reference(1:obj.m) - obj.position;
                pir = obj.reference(obj.m+1:end) - obj.velocity;
                [~, grad_qir_sigma] = sigma_norm(qir, 1);
                
                % calculate u_gamma_q
                u_gamma_q = obj.c_gamma * grad_qir_sigma;
                
                % calculate u_gamma_p
                u_gamma_p = obj.c_gamma * pir;
                
                % add to overall input
                u = u + u_gamma_q + u_gamma_p;
            end
            
            % Evaluate double integrator dynamics
            obj.move(u);
            
            % Send message to network, include position and velocity
            data = struct;
            data.position = obj.position;
            data.velocity = obj.velocity;
            data.u = u;
            data.num_N = obj.num_N;
            data.id = obj.id;
            data.neighbors = obj.neighbors;
            obj.send(data)
        end
    end
end

