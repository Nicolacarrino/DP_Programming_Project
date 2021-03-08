function [ J_opt, u_opt_ind ] = LinearProgramming(P, G)
%LINEARPROGRAMMING Linear Programming
%   Solve a stochastic shortest path problem by Linear Programming.
%
%   [J_opt, u_opt_ind] = LinearProgramming(P, G) computes the optimal cost
%   and the optimal control input for each state of the state space.
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

% A and b are used to construct coefficient matrix and constraint vector
A=[];
b=[];

% Create a copy of P
P_copy = P;

% Eliminate all rows and colums related with TERMINAL_STATE_INDEX
P_copy(TERMINAL_STATE_INDEX,:,:) = [];
P_copy(:,TERMINAL_STATE_INDEX,:) = [];

% Create a copy of G
G_copy = G;
% Eliminate all rows related with TERMINAL_STATE_INDEX
G_copy(TERMINAL_STATE_INDEX,:) = [];

for u = 1:5
    
    % Extracting the transition probability and stage cost for a
    % given policy
    P_tmp = P_copy(:,:,u);
    G_tmp = G_copy(:,u);
    
    % Construct the coefficient matrices for A*x <= b
    A = [A; (eye(K-1) - P_tmp)];
    b = [b; G_tmp];
end

% Eliminating the constraints with Inf value because linprog cannot run
% with infinite objective or constraint matrix coefficients
% However Inf here does not impose any actual constraint on V
to_del = b == inf;
b(to_del) = [];
A(to_del,:) = [];

% Construct the objective vector to maximize V since linprog finds the
% minimum of a problem we use -1 to convert it into a maximization problem
f = -1*ones(K-1,1);

% Utilising linprog function with option of not preprocessing
options = optimoptions('linprog','Preprocess','none');
J_opt = linprog(f,A,b,[],[],[],[],options);

% Reconstruct optimal cost with terminal state cost to be 0
J_opt = [J_opt(1:TERMINAL_STATE_INDEX-1); 0; J_opt(TERMINAL_STATE_INDEX:end)];

u_opt_ind = zeros(K,1);

% Calculating optimal strategy for every state
for i = 1:K
    [~,u_opt_ind(i)] = min( G(i,:) + sum(squeeze(P(i,:,:)).*J_opt(:,:)) );
end

end

