function [] = SesIOReformat()

% Directory containing tables
ioDir = [pwd, '/../../IOfiles\SessionData'];

%% =========================== LOAD TABLES ================================
[SS_In_All, SS_Out_ICR] = LoadTables(ioDir);

%% ========================== WHAT TO CHANGE ==============================

% ------------------------------- ADD VAR ---------------------------------
% Specify new variable to add
% reward_zones =  ...
%     {'20', '15', '10', '5', '0', '-5', '-10', '-15', '-20'};
% Zones_Rewarded = ...
%     categorical({'<undefined>'}, reward_zones);
Zones_Rewarded = {[]};

% Specify variable to put var after
var_ahead = 'Bulldozings';

% Add new variable
SS_Out_ICR = AddNewVar(SS_Out_ICR, Zones_Rewarded, var_ahead);

% --------------------------- CHANGE VAR ENTRY ----------------------------
% Specify inputs
var_change = 'Rotation_Positions';
new_val = {[]};
preserve_val = true;

% Make changes
SS_Out_ICR = ChangeVarEntries(SS_Out_ICR, var_change, new_val, preserve_val);

%% =========================== SAVE TABLES ================================

save(fullfile(ioDir,'SS_In_All'), 'SS_In_All');
save(fullfile(ioDir,'SS_Out_ICR'), 'SS_Out_ICR');

%% ============================ FUNCTIONS =================================


% ADD NEW VAR TO TABLE
    function [T2] = AddNewVar(T, new_var, var_ahead)
        % INPUT:
        %   T(table): Table to be modified
        %   new_var(any class): Var to be added
        %   var_ahead(string): Variable that new var should be after
        
        % Get input names as string
        table_name = inputname(1);
        var_name = inputname(2);
        
        % Set default for optional inputs
        if nargin < 3
            var_ahead = '';
        end
        
        
        % Determine what table is being changed
        if strcmp(table_name, 'SS_Out_ICR')
            
            % Add for each field (rat) entry
            T2 = structfun(@(x) AddV(x, new_var, var_name, var_ahead), T, 'uni', false);
            
        elseif strcmp(table_name, 'SS_In_All')
            
            % Add single instance
            T2 = AddV(T, new_var, var_name, var_ahead);
            
        end
        
        % Add var function
        function [t] = AddV(t, nv, vn, va)
            
            % Get other info
            tab_vars = t.Properties.VariableNames;
            
            % Check for optional inputs
            if ~strcmp(va, '') % var to put new var after
                col_ind = find(ismember(tab_vars, va));
            else
                % Put at end
                col_ind = width(t);
            end
            
            % Repmat var to table height and make into table
            t_new = table(repmat(nv, height(t), 1), 'VariableNames', {vn});
            
            % Fold into table
            if col_ind ~= width(t)
                t2 = [t(:,1:col_ind), t_new, t(:,col_ind+1:end)];
            else
                t2 = [t, t_new];
            end
            
            % Update table
            t = t2;
            
        end
        
    end

% CHANGE OLD VAR IN TABLE
    function [T2] = ChangeVarEntries(T, var_change, new_val, preserve_val)
        % INPUT:
        %   T(table): Table to be modified
        %   var_change(string): Var to change
        %   new_val(any): new default value
        %   preserve_val(bool)[optional]: specify if filled entries should
        %       be preserved
        
        % Get input names as string
        table_name = inputname(1);
        
        % Determine what table is being changed
        if strcmp(table_name, 'SS_Out_ICR')
            
            % Add for each field (rat) entry
            T2 = structfun(@(x) ChngV(x, var_change, new_val, preserve_val), T, 'uni', false);
            
        elseif strcmp(table_name, 'SS_In_All')
            
            % Add single instance
            T2 = ChngV(T, var_change, new_val, preserve_val);
            
        end
        
        % Change var function
        function [t] = ChngV(t, vc, nv, pv)
            
            % Just replace if not carrying over old entries
            if ~pv
                t.(vc) = repmat(nv, height(t), 1);
            else
                % Otherwise loop through each row
                keep_ind = false(height(t),1);
                for z_v = 1:height(t)
                    
                    % Check for cell
                    if isa(t.(vc)(z_v),'cell')
                        
                        % Check for nested cell
                        if isa(t.(vc){z_v},'cell')
                            if isa(t.(vc){z_v}{1},'categorical')
                                keep_ind(z_v) = ~isundefined(t.(vc){z_v}{1});
                            elseif isa(t.(vc){z_v}{1},'string') || isnumeric(t.(vc){z_v}{1})
                                keep_ind(z_v) = ~isempty(t.(vc){z_v}{1});
                            end
                        else
                            if isa(t.(vc){z_v},'categorical')
                                keep_ind(z_v) = ~isundefined(t.(vc){z_v});
                            elseif isa(t.(vc){z_v},'string') || isnumeric(t.(vc){z_v})
                                keep_ind(z_v) = ~isempty(t.(vc){z_v});
                            end
                        end
                        
                        % Check for categorical
                    elseif isa(t.(vc)(z_v),'categorical')
                        keep_ind(z_v) = ~isundefined(t.(vc)(z_v));
                        % Check for numeric or string
                    elseif isa(t.(vc)(z_v),'string') || isnumeric(t.(vc)(z_v))
                        keep_ind(z_v) = ~isempty(t.(vc)(z_v));
                    end
                end
                % Replace values
                t.(vc)(~keep_ind) = nv;
            end
        end
        
    end


% LOAD TABLE
    function [ss_all, ss_icr] = LoadTables(io_dir)
        
        % Chenge directory
        if ~ismember(regexp(pwd,'\w+(?=$)','match'), regexp(io_dir,'\w+(?=$)','match'))
            cd(io_dir);
        end
        
        % Import session data
        s = load('SS_In_All.mat');
        ss_all = s.SS_In_All;
        s = load('SS_Out_ICR.mat');
        ss_icr = s.SS_Out_ICR;
        
    end



end