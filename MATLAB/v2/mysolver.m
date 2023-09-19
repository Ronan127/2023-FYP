function [solution, remainingEquations] = mysolver(inputs, useSolve, solution)
arguments 
    inputs = cell;
    useSolve = 1;
    solution = struct;
end

%UNTITLED Solves a set of equations through a series of substitutions
%   Detailed explanation goes here
f= fieldnames(solution);
for i = 1:length(f)
   inputs= subs(inputs, sym(f{i}), solution.(f{i}));
end

equations = [];
inequalities = [];
for i = 1:length(inputs)
    if isSymType(inputs(i), "eq")
        equations = [equations; inputs(i)];
    elseif isSymType(inputs(i), "lt")||isSymType(inputs(i), "le")
        inequalities = [inequalities; inputs(i)];
    end
end

timeout = 0;
while isempty(equations) == 0
    [solution, equations, inequalities, timeout] = solveEasy(solution, equations, inequalities, timeout);

    if timeout == 1
        [solution, equations, inequalities, timeout] = solveEqual(solution, equations, inequalities, timeout);
    end
    if timeout == 2
        [solution, equations, inequalities, timeout] = solveSets(solution, equations, inequalities, timeout);
    end
    if timeout == 3
        if (length(symvar(equations)) <= length(equations))&&(useSolve == 1)
            sol2 = solveWithInequalities(equations, inequalities);
            f=fieldnames(sol2);
            if size(sol2.(f{1})) == 1
                for i = 1:length(f)
                    solution.(f{i}) = sol2.(f{i});
                end
                equations = [];
                timeout = 0;
            end
        end
    end

                            % tempequation = sym(zeros(length(tempeqs),1));
                            % tempvareqs = getEquationsContainingVariables(tempeqs, combinationvars); %list equations that contain each variable
                            % for k = 1:length(tempeqs)
                            %     % tempvareqs = getEquationsContainingVariables(tempeqs, combinationvars);
                            %     [~, idx] = min(cellfun('size',tempvareqs,2)); %choose variable with least equations describing it
                            %     for l = 1:length(tempvareqs{idx(1)}(1)) %cycle through each equation describing the chosen variable
                            %         try
                            %             tempequation(k) = isolate(tempeqs(tempvareqs{idx(1)}(l)),combinationvars(idx(1))); %isolate the chosen variable 
                            %             break
                            %         catch % if isolation fails, try the next equation
                            %             test = 1;
                            %         end
                            %     end
                            %     equationIndex = tempvareqs{idx(1)}(l); %index of the removed equation
                            %     tempeqs(equationIndex) = []; %remove eliminated equation
                            %     combinationvars(idx(1)) = []; %remove eliminated variable
                            % 
                            % 
                            %     %update tempvareqs
                            %     newvareqs =  tempvareqs{idx(1)}(1:end ~= l);
                            %     for m = 1:length(tempvareqs)
                            %         for n = 1:length(tempvareqs{m})
                            %             if tempvareqs{m}(n) == equationIndex
                            %                 %remove references to removed equations
                            %                 tempvareqs{m}(n) = [];
                            %                 %add refrences to substituted equations
                            %                 tempvareqs{m} = unique([tempvareqs{m}, newvareqs]);
                            %                 break
                            %             end
                            %         end
                            %         for n = 1:length(tempvareqs{m})
                            %             %offset indexes of equations
                            %             if tempvareqs{m}(n) > equationIndex
                            %                 tempvareqs{m}(n) = tempvareqs{m}(n)-1;
                            %             end
                            %         end
                            %     end
                            %     %remove eleminated variable from list
                            %     tempvareqs(idx(1)) = [];
                            % end
        %                     equations(combination) = tempequation;
        %                     timeout = 0;
        %                     break
        %                 elseif length(combination) >= length(varlist)
        %                     break
        %                 end
        %             end
        %         end
        %     end
        % end
 %       for i = 1:length(varlist)
  %          sizes(i) = length(varlist{i});
  %      end
  %      minimum = min(sizes);
  %      for i = 1:length(varlist)
  %          if sizes(i) == minimum
  %              sets
  %          end
  %      end
  %      for i in 
  %      while (1)
  %          for i = 1:length(varlist)
  %              newvar = sets
   %             sets{i,1} 
    %        end
%
%        end
    if timeout == 5
        break
    end
    timeout = timeout +1;
end
solution=orderfields(solution);
remainingEquations = equations;
end

function [solution, equations, inequalities, timeout] = solveEasy(solution, equations, inequalities, timeout)
    deletionIndex = [];
    for i = 1:length(inequalities)
        vars = symvar(inequalities(i));
        if isempty(vars) %if inequality has no variables, remove it
            deletionIndex = [deletionIndex, i];
        end
    end
    if ~isempty(deletionIndex)
        inequalities(deletionIndex) = [];
    end
    deletionIndex = [];
    %Check for single variable equations to solve and substitute
    for i = 1:length(equations)
        vars = symvar(equations(i));
        if isempty(vars) %if equation has no variables, remove it
            deletionIndex = [deletionIndex, i];
            % equations(i) = [];
            % timeout = 0;
            % break
        elseif length(vars) == 1
            %Check if equation is symetrical to avoid setting values to 0
            %with isolate
            if lhs(equations(i)) == rhs(equations(i))
                % equations(i) = [];
                deletionIndex = [deletionIndex, i];
            else
                % tempeq = isolate(equations(i),vars);
                % solution.(string(vars)) = vpa(rhs(tempeq));
                solution.(string(vars)) = vpa(solveWithInequalities(equations(i),inequalities));
                deletionIndex = [deletionIndex, i];
                equations = subs(equations, vars, solution.(string(vars)));
                if ~isempty(inequalities)
                    inequalities = subs(inequalities, vars, solution.(string(vars)));
                end
            end
        end
    end
    if ~isempty(deletionIndex)
        equations(deletionIndex) = [];
        timeout = 0;
    end
end

function [solution, equations, inequalities, timeout] = solveEqual(solution, equations, inequalities, timeout)
    varlist = {};
    matches = {};
    %List variables in each eq
    for i = 1:length(equations)
        vars = symvar(equations(i));
        varlist(i) = {vars};
    end
    %Find eqs with matching variables
    for i = 1:length(varlist)
        matches{i} = [];
        for j = 1:length(varlist)
            if (length(varlist{i}) == length(varlist{j}))
                %Check for sets of equations that use the same variables
                if (varlist{i} == varlist{j})
                    matches{i} = [matches{i}, j];
                end
            end
        end
        if length(matches{i}) == length(varlist{i})
            tempeqs = equations(matches{i});
            %Check if equations are identical
            eqsEqual=0;
            for j = 1:(length(tempeqs))
                for k = ((1:(length(tempeqs)))~=j)
                    if tempeqs(j) == tempeqs(k)
                        eqsEqual = j;
                    end
                    if lhs(tempeqs(j)) == 0
                        if lhs(tempeqs(k)) == 0
                            if rhs(tempeqs(j)) == -rhs(tempeqs(k))
                                eqsEqual = j;
                            end
                        end
                        if rhs(tempeqs(k)) == 0
                            if rhs(tempeqs(j)) == -lhs(tempeqs(k))
                                eqsEqual = j;
                            end
                        end
                    end
                    if rhs(tempeqs(j)) == 0
                        if lhs(tempeqs(k)) == 0
                            if lhs(tempeqs(j)) == -rhs(tempeqs(k))
                                eqsEqual = j;
                            end
                        end
                        if rhs(tempeqs(k)) == 0
                            if lhs(tempeqs(j)) == -lhs(tempeqs(k))
                                eqsEqual = j;
                            end
                        end
                    end
                end
            end
            if eqsEqual ~= 0
                equations(matches{i}(eqsEqual)) = [];
                timeout = 0;
                break
            end
            %Solve equations through substitution
            for j = 1:(length(tempeqs)-1)
                tempeqs(j) = isolate(tempeqs(j), varlist{i}(j));
                tempeqs((j+1):end) = subs(tempeqs((j+1):end), lhs(tempeqs(j)), rhs(tempeqs(j)));
            end
            equations(matches{i}) = tempeqs; 
            %for j = length(tempeqs):2
            %    tempsolution = simplify(rhs(tempeqs(j)));
            %    if j ~= 1
            %        tempeqs(1:(j-1)) = subs(tempeqs,lhs(tempeqs(end)), tempsolution);
            %    end
            %    solution.(string(lhs(tempeqs(j)))) = tempsolution;
            %    equations = subs(equations,lhs(tempeqs(j)), tempsolution);
            %end
            timeout = 0;
            break
        end 
    end
end

function [solution, equations, inequalities, timeout] = solveSets(solution, equations, inequalities, timeout)
    %Find fully defined sets
    varlist = {};
    %List variables in each eq
    for i = 1:length(equations)
        varlist(i) = {symvar(equations(i))};
    end
    vars = symvar(equations);
    %List equations that contain each variable
    vareqs = getEquationsContainingVariables(equations, vars);
    tempeqs = [];
    % Find equations with unique variables
    minimum = min(cellfun('size',vareqs,2));
    for i = 1:length(vareqs)
        if length(vareqs{i}) == minimum
            combination = vareqs{i};
            for j = 1:10 %Test adding up to 10 equations before giving up
                [combination, combinationvars, numvars] = addequations(combination, varlist);
                if  numvars <= length(combination)
                    %Solve equations through substitution
                    tempeqs = equations(combination);
                    break
                elseif length(combination) >= length(varlist)
                    break
                end
            end
            if ~isempty(tempeqs)
                break
            end
        end
    end
    if ~isempty(tempeqs)
        sol2 = solveWithInequalities(tempeqs, inequalities);
        f=fieldnames(sol2);
        if size(sol2.(f{1})) == 1
            for i = 1:length(f)
                solution.(f{i}) = sol2.(f{i});
                equations = subs(equations, sym(f{i}), solution.(f{i}));
                inequalities = subs(inequalities, sym(f{i}), solution.(f{i}));
            end
            equations(combination) = [];
            timeout = 0;
        end
    end
end

function solution = solveWithInequalities(equations, inequalities)
%Solves equations, adding only the relevant inequalities
    vars = symvar(equations);
    inequalitiesIndex = [];
    for i = 1:length(inequalities)
        if all(ismember(symvar(inequalities(i)), vars))
            inequalitiesIndex = [inequalitiesIndex, i];
        end
    end
    solution = solve([equations; inequalities(inequalitiesIndex)]);
end


function [combinations, combinationvars, numvars] = addequations(combination, varlist)
    combinationvars = [];
    for i = 1:length(combination)
        combinationvars = unique([combinationvars, varlist{combination}]);
    end
    possible_combinations = cell(length(varlist),1);
    minimum = 1000;
    remainingEqs=1:length(varlist);
    remainingEqs(combination) = [];
    for j = remainingEqs
        possible_combinations{j} = unique([combinationvars, varlist{j}]);
        if length(possible_combinations{j}) < minimum
            minimum = length(possible_combinations{j});
        end
    end
    combinationcounter = 1;
    for j = 1:length(possible_combinations)
        if length(possible_combinations{j}) == minimum
            combinations = [combination, j];
            combinationvars = possible_combinations{j};
            numvars = length(possible_combinations{j});
            combinationcounter = combinationcounter + 1;
            break
        end
    end
end

function [vareqs] = getEquationsContainingVariables(equations, vars)
    vareqs = cell(1,length(vars));
    eqvars = cell(1,length(equations));
    for i = 1:length(equations)
        eqvars{i} = symvar(equations(i));
    end
    for i = 1:length(vars)
        vareqs{i} = [];
        for j = 1:length(equations)
            if ismember(vars(i), eqvars{j})
                vareqs{i} = [vareqs{i}, j];
            end
        end
    end
end
