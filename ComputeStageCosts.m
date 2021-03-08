function G = ComputeStageCosts( stateSpace, map )
%COMPUTESTAGECOSTS Compute stage costs.
% 	Compute the stage costs for all states in the state space for all
%   control inputs.
%
%   G = ComputeStageCosts(stateSpace, map)
%   computes the stage costs for all states in the state space for all
%   control inputs.
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
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the expected stage cost if we are in state i and
%           apply control input l.

global GAMMA R P_WIND Nc
global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
global NORTH SOUTH EAST WEST HOVER
global K
global TERMINAL_STATE_INDEX

% Extracting size of map
[M,N] = size(map);

% Defining m and n coordinates for SHOOTER and count number of Shooters
[shooter_m, shooter_n] = find(map == SHOOTER);
num_shooter = size(shooter_m,1);

% Initializing matrix G
G = zeros(K,5);

% Looping over all states
for i = 1 : K
    % Extracting coordinates of state i in m and n coordinates,
    % _c stands for current
    m_c = stateSpace(i,1);
    n_c = stateSpace(i,2);
    
    
    if i == TERMINAL_STATE_INDEX
        % Defining terminal state cost
        G(i,:) = 0;
    else
        for u = [NORTH SOUTH EAST WEST HOVER]
            % m_n and n_n are used to account for the input movement
            % _n stands for next
            m_n = m_c;
            n_n = n_c;
            
            % Defining a flag in order to account for allowable inputs
            flag = true;
            
            % Checking if the input is allowed
            % if it is not allowed we put flag to false and cost to inf
            % else if is allowed we apply the control input
            switch u
                case NORTH
                    if n_c + 1 > N || map(m_c,n_c + 1) == TREE
                        G(i,NORTH) = inf;
                        flag = false;
                    else
                        n_n = n_c + 1;
                    end
                    
                case SOUTH
                    if n_c - 1 <= 0 || map(m_c,n_c - 1) == TREE
                        G(i,SOUTH) = inf;
                        flag = false;
                    else
                        n_n = n_c - 1;
                    end
                    
                case EAST
                    if m_c + 1 > M || map(m_c + 1,n_c) == TREE
                        G(i,EAST) = inf;
                        flag = false;
                    else
                        m_n = m_c + 1;
                    end
                    
                case WEST
                    if m_c - 1 <= 0 || map(m_c - 1,n_c) == TREE
                        G(i,WEST) = inf;
                        flag = false;
                    else
                        m_n = m_c - 1;
                    end
                    
                case HOVER
                    
            end
            
            % If we have had an allowable input
            if  flag
                
                % We calculate the probability of crashing
                P_crash = 0;
                
                % Initializing variable product, which is the probability
                % of not being shot by the shooters
                product = 1;
                
                
                % North Wind with probability P_WIND/4
                % Checking if after wind disturbance we are in bound and
                % not in a tree
                if n_n + 1 <= N && map(m_n,n_n + 1) ~= TREE
                    
                    % Calculating probability of not being shot
                    for tmp = 1:num_shooter
                        
                        % Calculating distance from drone position and
                        % shooter
                        d = abs(n_n + 1-shooter_n(tmp)) + abs(m_n - shooter_m(tmp));
                        if d <= R
                            
                            % Using De Morgan law not(union(Ai)) == intersection(not(Ai))
                            % product is calculating intersection(not(Ai))
                            % where Ai is probability of being shot by i
                            product = product*(1-GAMMA/(d+1));
                        end
                    end
                    
                    % 1-product is probability of being shot by at least
                    % one shooter
                    P_crash = P_crash + (1-product)*P_WIND/4;
                else
                    % Probability of crashing out of bound or in a tree due
                    % to wind disturbance
                    P_crash = P_crash + P_WIND/4;
                end
                
                % Reset product to 1 and repeat for all wind disturbances
                product = 1;
                
                %South Wind
                if n_n - 1 > 0 && map(m_n,n_n - 1)~= TREE
                    for tmp = 1:num_shooter
                        d = abs(n_n - 1-shooter_n(tmp)) + abs(m_n - shooter_m(tmp));
                        if d <= R
                            product = product*(1-GAMMA/(d+1));
                        end
                    end
                    
                    P_crash = P_crash + (1-product)*P_WIND/4;
                else
                    P_crash = P_crash + P_WIND/4;
                end
                
                product = 1;
                
                %East Wind
                if m_n + 1 <= M && map(m_n + 1,n_n)~= TREE
                    for tmp = 1:num_shooter
                        d = abs(n_n - shooter_n(tmp)) + abs(m_n+1 - shooter_m(tmp));
                        if d <= R
                            product = product*(1-GAMMA/(d+1));
                        end
                    end
                    
                    P_crash = P_crash + (1-product)*P_WIND/4;
                else
                    P_crash = P_crash + P_WIND/4;
                end
                
                product = 1;
                
                %West Wind
                if m_n - 1 > 0 && map(m_n - 1,n_n)~= TREE
                    for tmp = 1:num_shooter
                        d = abs(n_n - shooter_n(tmp)) + abs(m_n - 1-shooter_m(tmp));
                        if d <= R
                            product = product*(1-GAMMA/(d+1));
                        end
                    end
                    
                    P_crash = P_crash + (1-product)*P_WIND/4;
                else
                    P_crash = P_crash + P_WIND/4;
                end
                
                product = 1;
                
                %No Wind (Hover) with probability 1-P_WIND
                for tmp = 1:num_shooter
                    d = abs(n_n - shooter_n(tmp)) + abs(m_n - shooter_m(tmp));
                    if d <= R
                        product = product*(1-GAMMA/(d+1));
                    end
                end
                
                P_crash = P_crash + (1-product)*(1-P_WIND);
                
                % Setting cost to be expected value of the steps
                G(i,u) = P_crash*Nc + (1-P_crash)*1;
            end
        end
    end
end

end

