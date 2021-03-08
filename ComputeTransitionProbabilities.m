function P = ComputeTransitionProbabilities( stateSpace, map)
%COMPUTETRANSITIONPROBABILITIES Compute transition probabilities.
% 	Compute the transition probabilities between all states in the state
%   space for all control inputs.
%
%   P = ComputeTransitionProbabilities(stateSpace, map)
%   computes the transition probabilities between all states in the state
%   space for all control inputs.
%
%   Input arguments:
%
%       stateSpace:
%           A (K x 3)-matrix, where the i-th row represents the i-th
%           element of the state space.
%
%       map:
%           A (M x N)-matrix describing the world. With
%           values: FREE TREE SHOOTER PICK_UP DROP_OFF BASE
%
%   Output arguments:
%
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.

global GAMMA R P_WIND
global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
global NORTH SOUTH EAST WEST HOVER
global K TERMINAL_STATE_INDEX

% Extracting size of map
[M,N] = size(map);

%Base station, calculating m,n coordinates and index
[base_m, base_n] = find(map == BASE);
base_index = find(stateSpace(:,1) == base_m & stateSpace(:,2) == base_n & stateSpace(:,3) == 0);

%Shooter, calculating m,n coordinates and number
[shooter_m, shooter_n] = find(map == SHOOTER);
num_shooter = size(shooter_m,1);

%Pick-up station, calculating m,n coordinates
[pick_m, pick_n] = find(map == PICK_UP);

% Initialization of P
P = zeros(K,K,5);

% Looping for all states
for i = 1 : K
    % Extracting coordinates of state i in m,n,q coordinates,
    % _c stands for current
    m_c = stateSpace(i,1);
    n_c = stateSpace(i,2);
    q_c = stateSpace(i,3);
    
    if i == TERMINAL_STATE_INDEX
        % Probability of remaining into the terminal state after we have
        % reach it
        P(i,i,:) = 1;
    else
        for u = [NORTH SOUTH EAST WEST HOVER]
            % m_n,n_n,q_n are used to account for the input movement and
            % package; _n stands for next
            m_n = m_c;
            n_n = n_c;
            q_n = q_c;
            
            % Updating next coordinates after applying control input
            switch u
                case NORTH
                    if n_c + 1 <= N && map(m_c,n_c + 1) ~= TREE
                        n_n = n_c + 1;
                    else
                        continue
                    end
                case SOUTH
                    if n_c - 1 > 0 && map(m_c,n_c - 1) ~= TREE
                        n_n = n_c - 1;
                    else
                        continue
                    end
                case EAST
                    if m_c + 1 <= M && map(m_c + 1,n_c) ~= TREE
                        m_n = m_c + 1;
                    else
                        continue
                    end
                case WEST
                    if m_c - 1 > 0 && map(m_c - 1,n_c) ~= TREE
                        m_n = m_c - 1;
                    else
                        continue
                    end
                case HOVER
            end
            
            % Initializing variable product, which is the probability
            % of not being shot by the shooters
            product = 1;
            
            % North Wind with probability P_WIND/4
            % Checking if after wind disturbance we are in bound and
            % not in a tree
            if n_n + 1 <= N && map(m_n,n_n + 1) ~= TREE
                
                % Calculating probability of not being shot
                for tmp = 1:num_shooter
                    
                    % Calculating distance from drone position and shooter
                    d = abs(n_n+1-shooter_n(tmp)) + abs(m_n-shooter_m(tmp));
                    if d <= R
                        % Using De Morgan law not(union(Ai)) == intersection(not(Ai))
                        % product is calculating intersection(not(Ai))
                        % where Ai is probability of being shot by i
                        product = product*(1-GAMMA/(d+1));
                    end
                end
                
                % 1-product is probability of being shot by at least one shooter
                % We update probability of going to the base station
                P(i,base_index,u) = P(i,base_index,u) + (1-product)*P_WIND/4;
                
                % Checking if we have reached PICK_UP due to wind
                % disturbances
                if m_n == pick_m && n_n + 1 == pick_n && q_n == 0
                    
                    % If the drone has not crashed by this point and is
                    % at a pick-up station carrying no package, it collects one
                    j = find(stateSpace(:,1) == m_n & stateSpace(:,2) == n_n + 1 & stateSpace(:,3) == 1);
                    
                    % Updating transition probability between i,j,u
                    P(i,j,u) = P(i,j,u) + product*P_WIND/4;
                else
                    % Finding state where drone is after input and wind
                    j = find(stateSpace(:,1) == m_n & stateSpace(:,2) == n_n + 1 & stateSpace(:,3) == q_n);
                    % Updating transition probability between i,j,u
                    P(i,j,u) = P(i,j,u) + product*P_WIND/4;
                end
                
            else
                % Probability of crashing out of bound or in a tree due
                % to wind disturbance
                P(i,base_index,u) = P(i,base_index,u) + P_WIND/4;
            end
            
            % Reset product to 1 and repeat for all wind disturbances
            product = 1;
            
            %South Wind
            if n_n - 1 > 0 && map(m_n,n_n - 1) ~= TREE
                for tmp = 1:num_shooter
                    d = abs(n_n - 1 - shooter_n(tmp)) + abs(m_n - shooter_m(tmp));
                    if d <= R
                        product = product*(1-GAMMA/(d+1));
                    end
                end
                P(i,base_index,u) = P(i,base_index,u) + (1-product)*P_WIND/4;
                
                if m_n == pick_m && n_n - 1 == pick_n && q_n == 0
                    j = find(stateSpace(:,1) == m_n & stateSpace(:,2) == n_n - 1 & stateSpace(:,3) == 1);
                    P(i,j,u) = P(i,j,u) + product*P_WIND/4;
                else
                    j = find(stateSpace(:,1) == m_n & stateSpace(:,2) == n_n - 1 & stateSpace(:,3) == q_n);
                    P(i,j,u) = P(i,j,u) + product*P_WIND/4;
                end
                
            else
                P(i,base_index,u) = P(i,base_index,u) + P_WIND/4;
            end
            
            product = 1;
            
            %East Wind
            if m_n + 1 <= M && map(m_n + 1,n_n) ~= TREE
                for tmp = 1:num_shooter
                    d = abs(n_n - shooter_n(tmp)) + abs(m_n + 1 - shooter_m(tmp));
                    if d <= R
                        product = product*(1-GAMMA/(d+1));
                    end
                end
                P(i,base_index,u) = P(i,base_index,u) + (1-product)*P_WIND/4;
                
                if m_n + 1 == pick_m && n_n == pick_n && q_n == 0
                    j = find(stateSpace(:,1) == m_n + 1 & stateSpace(:,2) == n_n & stateSpace(:,3) == 1);
                    P(i,j,u) = P(i,j,u) + product*P_WIND/4;
                else
                    j = find(stateSpace(:,1) == m_n + 1 & stateSpace(:,2) == n_n & stateSpace(:,3) == q_n);
                    P(i,j,u) = P(i,j,u) + product*P_WIND/4;
                end
                
            else
                P(i,base_index,u) = P(i,base_index,u) + P_WIND/4;
            end
            
            product = 1;
            
            %West Wind
            if m_n - 1 > 0 && map(m_n - 1,n_n) ~= TREE
                for tmp = 1:num_shooter
                    d = abs(n_n - shooter_n(tmp)) + abs(m_n - 1 - shooter_m(tmp));
                    if d <= R
                        product = product*(1-GAMMA/(d+1));
                    end
                end
                P(i,base_index,u) = P(i,base_index,u) + (1-product)*P_WIND/4;
                
                if m_n - 1 == pick_m && n_n == pick_n && q_n == 0
                    j = find(stateSpace(:,1) == m_n - 1 & stateSpace(:,2) == n_n & stateSpace(:,3) == 1);
                    P(i,j,u) = P(i,j,u) + product*P_WIND/4;
                else
                    j = find(stateSpace(:,1) == m_n - 1 & stateSpace(:,2) == n_n & stateSpace(:,3) == q_n);
                    P(i,j,u) = P(i,j,u) + product*P_WIND/4;
                end
                
            else
                P(i,base_index,u) = P(i,base_index,u) + P_WIND/4;
            end
            
            product = 1;
            
            %No Wind (Hover) with probability 1-P_WIND
            for tmp = 1:num_shooter
                d = abs(n_n - shooter_n(tmp)) + abs(m_n - shooter_m(tmp));
                if d <= R
                    product = product*(1-GAMMA/(d+1));
                end
            end
            P(i,base_index,u) = P(i,base_index,u) + (1-product)*(1-P_WIND);
            
            if m_n == pick_m && n_n == pick_n && q_n == 0
                j = find(stateSpace(:,1) == m_n & stateSpace(:,2) == n_n & stateSpace(:,3) == 1);
                P(i,j,u) = P(i,j,u) + product*(1-P_WIND);
            else
                j = find(stateSpace(:,1) == m_n & stateSpace(:,2) == n_n & stateSpace(:,3) == q_n);
                P(i,j,u) = P(i,j,u) + product*(1-P_WIND);
            end
        end
    end
end

end

