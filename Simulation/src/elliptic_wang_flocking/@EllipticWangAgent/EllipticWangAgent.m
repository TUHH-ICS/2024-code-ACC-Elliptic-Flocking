%---------------------------------------------------------------------------------------------------
% For Paper
% "Distributed Flocking Control with Ellipsoidal Level Sets"
% by P. Hastedt, A. Datar, K. Kocev, and H. Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Philipp Hastedt
%---------------------------------------------------------------------------------------------------

% This class is a implementation of the flocking algorithm derived by
% Wang et al. in the paper "Adaptive Spacing Policy Design of 
% Flocking Control for Multi-agent Vehicular Systems"
% (https://doi.org/10.1016/j.ifacol.2022.11.236)

classdef EllipticWangAgent < DoubleIntegratorAgent
    
    % member variables
    properties(GetAccess = private, SetAccess = private)
        d;              % desired distance
        r;              % interaction range
        h;              % bump function parameter
        tau;            % bump function parameter
        epsilon;        % sigma norm parameter
        c_alpha;        % alpha flocking tuning parameter
        c_alpha_phi;    % alpha flocking tuning parameter
        c_gamma;        % gamma flocking tuning parameter
        m;              % space dimension
        numberAgents;   % number of agents
        reference;      % gamma agent

        % elliptic parameters
        d_major;    % major semi-axis length
        d_minor;    % major semi-axis length
        f1;         % 1st focal point
        f2;         % 2nd focal point
    end
    
    % data to be transmitted in addition to position, velocity, and u
    properties
        num_N;      % number of neighbors
        neighbors;  % ids of neighbors
    end
    
    % methods
    methods
        y = s_h(obj, z);
        alpha = phi_wang(obj, qij_sigma, eij);
        V = psi_wang(obj, z);
    end
    
    methods
        function obj = EllipticWangAgent(id, param, initialPos, initialVel, cfg)
            % call parent constructor
            obj@DoubleIntegratorAgent(id, param.dT, initialPos, initialVel);
            
            % load parameters
            obj.d = cfg.d;
            obj.r = param.range; 
            obj.h = cfg.h;
            obj.epsilon = cfg.epsilon;
            obj.tau = cfg.tau;
            obj.c_alpha = cfg.c_alpha;
            obj.c_alpha_phi = cfg.c_alpha_phi;
            obj.c_gamma = cfg.c_gamma;
            obj.d_major = max(cfg.ellipseAxes)*obj.d;
            obj.d_minor = min(cfg.ellipseAxes)*obj.d;
            obj.m = cfg.m;
            obj.numberAgents = param.agentCount;
      
            % calculate sigma values
            obj.d = sigma_norm(obj.d, obj.epsilon);
            obj.r = sigma_norm(obj.r, obj.epsilon);
            
            % calculate ellipse focal points
            e = sqrt(obj.d_major^2-obj.d_minor^2);
            R = [cos(cfg.ellipseRotation) -sin(cfg.ellipseRotation);
                sin(cfg.ellipseRotation)  cos(cfg.ellipseRotation)];
            obj.f1 = R*[e;0];
            obj.f2 = R*[-e;0];
            
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
            qi = obj.position;
            pi = obj.velocity;
            % alpha flocking component
            if obj.num_N ~= 0
                for message = messages
                    obj.neighbors(message.data.id) = 1;
                    % extract neighbor data from message
                    qj = message.data.position;
                    pj = message.data.velocity;
                    
                    % calculate corresponding ellipse
                    eij = ( norm(qj - (qi + obj.f1)) + norm(qj - (qi - obj.f1)) ) - 2*obj.d_major;
                    
                    [qij_sigma, ~] = sigma_norm(qj-qi, obj.epsilon);
                    
                    % calculate u_alpha_q
                    u_alpha_q = obj.phi_wang(qij_sigma, eij) * (qj-qi)/sqrt(1+norm(qj-qi));
                    
                    % calculate u_alpha_p
                    aij = obj.s_h(qij_sigma / obj.r);
                    u_alpha_p = aij * (pj - pi);
                    
                    % add to overall input
                    u = u + u_alpha_q + u_alpha_p;
                end
            end
            
            % gamma flocking component
            if ~isempty(obj.reference)
                qir = obj.reference(1:obj.m) - obj.position;
                pir = obj.reference(obj.m+1:end) - obj.velocity;
                [~, grad_qir_sigma] = sigma_norm(qir, 1);
                u = u + obj.c_gamma*(grad_qir_sigma + pir);  
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

