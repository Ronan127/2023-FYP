function [solution, remainingEquations] = mysolver(equations, useSolve, solution)
arguments 
    equations = cell;
    useSolve = 1;
    solution = struct;
end

%UNTITLED Solves a set of equations through a series of substitutions
%   Detailed explanation goes here
f= fieldnames(solution);
for i = 1:length(f)
   equations= subs(equations, sym(f{i}), solution.(f{i}));
end
timeout = 0;
while isempty(equations) == 0
    %Check for single variable equations to solve and substitute
    for i = 1:length(equations)
        vars = symvar(equations(i));
        if isempty(vars)
            equations(i) = [];
            timeout = 0;
            break
        elseif length(vars) == 1
            %Check if equation is symetrical to avoid setting values to 0
            %with isolate
            if lhs(equations(i)) == rhs(equations(i))
                equations(i) = [];
            else
                tempeq = isolate(equations(i),vars);
                % tempeq = simplify(rhs(tempeq));
                solution.(string(vars)) = vpa(rhs(tempeq));
                equations(i) = [];
                equations = subs(equations, vars, solution.(string(vars)));
            end
            timeout = 0;
            break
        end
    end
   
    if timeout == 1
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
                %Solve equations through substitution
                tempeqs = equations(matches{i});
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
    if timeout == 2
        if (length(symvar(equations)) <= length(equations))&&(useSolve == 1)
            sol2 = solve(equations);
            f=fieldnames(sol2);
            if size(sol2.(f{1})) == 1
                for i = 1:length(f)
                    solution.(f{i}) = sol2.(f{i});
                end
                equations = [];
                timeout = 0;
            end
        else   
            %Find fully defined sets
            vars = symvar(equations);
            vareqs = getEquationsContainingVariables(equations, vars);
    
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
                            tempequation = sym(zeros(length(tempeqs),1));
                            tempvareqs = getEquationsContainingVariables(tempeqs, combinationvars);
                            for k = 1:length(tempeqs)
                                % tempvareqs = getEquationsContainingVariables(tempeqs, combinationvars);
                                [~, idx] = min(cellfun('size',tempvareqs,2));
                                for l = 1:tempvareqs{idx(1)}(1)
                                    try
                                        tempequation(k) = isolate(tempeqs(tempvareqs{idx(1)}(l)),combinationvars(idx(1)));
                                        break
                                    catch
                                    end
                                end
        
                                tempeqs(tempvareqs{idx(1)}(l)) = [];
                                combinationvars(idx(1)) = [];
                                equationIndex = tempvareqs{idx(1)}(l);
                                % tempvareqs_old = tempvareqs;
        
                                %update tempvareqs
                                newvareqs =  tempvareqs{idx(1)}(1:end ~= l);
                                for m = 1:length(tempvareqs)
                                    for n = 1:length(tempvareqs{m})
                                        if tempvareqs{m}(n) == equationIndex
                                            %remove references to removed equations
                                            tempvareqs{m}(n) = [];
                                            %add refrences to substituted equations
                                            tempvareqs{m} = unique([tempvareqs{m}, newvareqs]);
                                            break
                                        end
                                    end
                                    for n = 1:length(tempvareqs{m})
                                        %offset indexes of equations
                                        if tempvareqs{m}(n) > equationIndex
                                            tempvareqs{m}(n) = tempvareqs{m}(n)-1;
                                        end
                                    end
                                end
                                %remove eleminated variable from list
                                tempvareqs(idx(1)) = [];
                                % tempvareqs2 = tempvareqs;
        
        
                                tempeqs = subs(tempeqs, lhs(tempequation(k)), rhs(tempequation(k)));
        
                                % tempvareqs = getEquationsContainingVariables(tempeqs, combinationvars);
        
                                % for m = 1:length(tempvareqs)
                                %     if length(tempvareqs{m}) == length(tempvareqs2{m})
                                %         for n = 1:length(tempvareqs{m})
                                %             if tempvareqs{m}(n) ~= tempvareqs2{m}(n)
                                %                 test = 0;
                                %             end
                                %         end
                                %     else
                                %         test = 0;
                                %     end
                                % end
                            end
                            equations(combination) = tempequation;
                            timeout = 0;
                            break
                        elseif length(combination) >= length(varlist)
                            break
                        end
                    end
                end
            end
        end
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
    

    end
    if timeout == 5
        break
    end
    timeout = timeout +1;
end
solution=orderfields(solution);
remainingEquations = equations;
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
