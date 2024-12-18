%% Definizione dell'ordine dei giunti
joint_order = { ...
    'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint', ...
    'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint', ...
    'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint', ...
    'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint'};

%% Estrazione dati dai messaggi
bSel = select(bag,'Topic',param.joint_readings);
messages = readMessages(bSel);

% Liste dinamiche (inizialmente vuote) da riempire solo se ci sono velocità
time_list = [];
joint_ang_list = [];
joint_vel_list = [];

for i=1:length(messages)
    % Verifica se esistono dati di velocity e non sono vuoti
    if isfield(messages{i}, 'velocity') && ~isempty(messages{i}.velocity)
        
        % Leggi il timestamp
        current_time = double(messages{i}.header.stamp.sec) + double(messages{i}.header.stamp.nanosec)*10^-9;

        % Ottieni i nomi dei giunti, posizioni e velocità
        current_names = messages{i}.name;
        current_positions = messages{i}.position;
        current_velocities = messages{i}.velocity;

        % Inizializza i vettori per angoli e velocità per questo messaggio
        current_jang = NaN(1,12);
        current_jvel = NaN(1,12);

        % Popola i vettori con l'ordine desiderato
        for j=1:length(joint_order)
            idx = find(strcmp(current_names, joint_order{j}));
            if ~isempty(idx)
                current_jang(j) = current_positions(idx);
                current_jvel(j) = current_velocities(idx);
            else
                % Se non trovo quel giunto, assegno NaN e continuo
                current_jang(j) = NaN;
                current_jvel(j) = NaN;
                warning('Giunto %s non trovato nel messaggio %d', joint_order{j}, i);
            end
        end

        % Aggiungo i dati alle liste
        time_list = [time_list; current_time];
        joint_ang_list = [joint_ang_list; current_jang];
        joint_vel_list = [joint_vel_list; current_jvel];
    else
        % Nessun dato di velocità per questo messaggio, si ignora
    end
end

% Creazione delle timeseries solo con i dati validi
j_ang = timeseries(joint_ang_list, time_list);
j_vel = timeseries(joint_vel_list, time_list);

% Controlli sul tempo
if length(time_list) > 1
    dt_list = diff(time_list);
    if any(dt_list <= 0)
        fprintf('dt_list non è strettamente maggiore di zero\n');
    end
    dt6 = mean(dt_list);
    disp('JOINT Simulation:')
    fprintf('T_start= %f s\n', time_list(1));
    fprintf('T_end= %f s\n', time_list(end));
    fprintf('T_tot= %f s\n', time_list(end)-time_list(1));
    fprintf('dt_mean= %f s\n', dt6);
    fprintf('dt_max= %f s\n', max(dt_list));
    fprintf('dt_min= %f s\n\n', min(dt_list));
else
    disp('Nessun dato valido (con velocità) trovato.');
end
