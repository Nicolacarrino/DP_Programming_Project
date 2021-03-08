function [ J_opt, u_opt_ind ] = PolicyIteration( P, G )
%POLICYITERATION Policy iteration
%   Solve a stochastic shortest path problem by Policy Iteration.
%
%   [J_opt, u_opt_ind] = PolicyIteration(P, G) computes the optimal cost and
%   the optimal control input for each state of the state space.
%
%   Input arguments:
%
%       P:
%           A (k x k x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%       G:
%           A (k x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%
%       J_opt:
%       	A (k x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (k x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space. Mapping of the
%       	terminal state is arbitrary (for example: HOVER).

global K HOVER

%% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration?
global TERMINAL_STATE_INDEX
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)

% Defining a proper policy to initialize PI algorithm
U = HOVER*ones(K-1,1);

% Create a copy of P
P_copy = P;
% Eliminate all rows and colums related with TERMINAL_STATE_INDEX
P_copy(TERMINAL_STATE_INDEX,:,:) = [];
P_copy(:,TERMINAL_STATE_INDEX,:) = [];

% Create a copy of G
G_copy = G;
% Eliminate all rows related with TERMINAL_STATE_INDEX
G_copy(TERMINAL_STATE_INDEX,:) = [];

% Initializing optimal cost for mu_c and mu_n
J_mu_c = zeros(K-1,1);
J_mu_n = ones(K-1,1);

P_tmp = zeros(K-1,K-1);
G_tmp = zeros(K-1,1);

% Looping until J_mu_c and J_mu_n is numerically the same
while (norm(J_mu_n - J_mu_c) > eps )
    
    % Assigning J_mu_n to J_mu_c
    J_mu_c = J_mu_n;
    
    
    % Policy Evaluation
    for i = 1:K-1
        
        % Extracting the transition probability and stage cost for a
        % given policy
        P_tmp(i,:) = P_copy(i,:,U(i));
        G_tmp(i) = G_copy(i,U(i));
        
    end
    
    % Solving J = G + PJ for J.    J = (I - P)^(-1)*G
    J_mu_n = (eye(K-1)-P_tmp)\G_tmp;
    
    
    % Policy Improvement
    for i = 1:K-1
        
        % Utilising PI algorithm for policy imprvement
        [~, U(i)] = min( G_copy(i,:) + sum(squeeze(P_copy(i,:,:)).*J_mu_n(:,:)));
    end
    
end

% Reconstruct optimal cost with terminal state cost to be 0
J_opt = [J_mu_n(1:TERMINAL_STATE_INDEX-1); 0; J_mu_n(TERMINAL_STATE_INDEX:end)];

% Calculating one strategy for terminal state
[~, u_term] = min( G(TERMINAL_STATE_INDEX,:) + sum(squeeze(P(TERMINAL_STATE_INDEX,:,:)).*J_opt(:,:)));

% Reconstruct optimal strategy with terminal state strategy u_term
u_opt_ind = [U(1:TERMINAL_STATE_INDEX-1); u_term; U(TERMINAL_STATE_INDEX:end)];

end
