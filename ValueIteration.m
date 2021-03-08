function [ J_opt, u_opt_ind ] = ValueIteration(P, G)
%VALUEITERATION Value iteration
%   Solve a stochastic shortest path problem by Value Iteration.
%
%   [J_opt, u_opt_ind] = ValueIteration(P, G) computes the optimal cost and
%   the optimal control input for each state of the state space.
%
%   Input arguments:
%
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%
%       J_opt:
%       	A (K x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (K x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space. Mapping of the
%       	terminal state is arbitrary (for example: HOVER).
global K HOVER

%% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration ?
global TERMINAL_STATE_INDEX
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)

% Create a copy of P
P_copy = P;
% Eliminating all rows and colums related with TERMINAL_STATE_INDEX
P_copy(TERMINAL_STATE_INDEX,:,:) = [];
P_copy(:,TERMINAL_STATE_INDEX,:) = [];

% Create a copy of G
G_copy = G;
% Eliminate all rows related with TERMINAL_STATE_INDEX
G_copy(TERMINAL_STATE_INDEX,:) = [];

% V_c stands for current V and V_n stands for next V
V_c = ones(K-1,1);
V_n = zeros(K-1,1);

% Initializing optimal strategy
U = zeros(K-1,1);

% Looping until difference between V_n-V_c is less than 10^-5
while (norm(V_n-V_c) > 1e-5)
    
    % Assigning V_n to V_c
    V_c = V_n;
    
    % Calculate the V and U of next steps for all states without terminal
    % state
    for i = 1:K-1
        % Utilising VI algorithm
        [V_n(i), U(i)] = min( G_copy(i,:) + sum(squeeze(P_copy(i,:,:)).*V_c(:,:)) );
    end
end

% Reconstruct optimal cost with terminal state cost to be 0
J_opt = [V_c(1:TERMINAL_STATE_INDEX-1); 0; V_c(TERMINAL_STATE_INDEX:end)];

% Calculating one strategy for terminal state
[~, u_term] = min( G(TERMINAL_STATE_INDEX,:) + sum(squeeze(P(TERMINAL_STATE_INDEX,:,:)).*J_opt(:,:)) );

% Reconstruct optimal strategy with terminal state strategy u_term
u_opt_ind = [U(1:TERMINAL_STATE_INDEX-1); u_term; U(TERMINAL_STATE_INDEX:end)];


end