% conveyor_sort_sim.m
% Conveyor belt object sorting simulator
% Clean & corrected version (no '=' misuse errors)

clear; close all; clc;

%% Simulation parameters
T_sim = 40;           % total simulation time
dt = 0.02;            % timestep
time = 0:dt:T_sim;

% Conveyor geometry (meters)
L = 1.0;
x_spawn = 0.05;
x_exit  = L - 0.05;
x_pick  = 0.65;

% Object parameters
v = 0.08;             
obj_radius = 0.025;   
spawn_rate = 0.8;     
min_interarrival = 0.2;

% Sensor model
sensor_range = 0.06;
false_pos_rate = 0.01;
false_neg_rate = 0.05;
detection_jitter = 0.02;

% Actuator model
t_react = 0.06;
t_act = 0.35;
act_position = 0;     
act_state = 0;        % 0 = idle, 1 = active
act_timer = 0;

queueing_allowed = false;

rng(1234);

%% Dynamic variables
objs = struct('x', {}, 'id', {}, 'spawn_t', {}, 'picked', {}, 'sorted', {});
next_id = 1;
next_spawn_t = max(exprnd(1/spawn_rate), min_interarrival);

% Logging structure
events = [];
metrics = struct('total_spawn',0,'sorted',0,'missed',0, ...
                 'false_pos',0,'false_neg',0,'act_cycles',0, ...
                 'act_time',0,'latencies',[]);

logEvent = @(t, type, info) ...
    assignin('caller','events',[events struct('t',t,'type',type,'info',info)]);

%% --- Animation Setup ---
fig = figure('Name','Conveyor Simulation','Color',[1 1 1]);
ax = axes(fig); hold on; axis off;
axis([0 L -0.2 0.6]);

rectangle('Position',[0 0 L 0.2],'FaceColor',[0.9 0.9 0.92],'EdgeColor','none');
plot([x_pick x_pick],[0 0.4],'k--','LineWidth',1);
text(x_pick,0.42,'Sensor','HorizontalAlignment','center','FontSize',8);

div_x = x_pick + 0.12;
plot(div_x,0.2,'k.','MarkerSize',12);
text(div_x,0.25,'Diverter','HorizontalAlignment','center','FontSize',8);

arm_len = 0.12;
arm_h = 0.2;
arm_h_line = plot([div_x div_x+arm_len],[arm_h arm_h],'r-','LineWidth',4);

th_total = text(0.02,0.5,'','FontSize',9);

obj_patches = [];

%% --- Simulation Loop ---
for ti = 1:length(time)
    t = time(ti);

    %% Spawn objects
    if t >= next_spawn_t
        objs(end+1).x = x_spawn;
        objs(end).id = next_id;
        objs(end).spawn_t = t;
        objs(end).picked = false;
        objs(end).sorted = false;

        metrics.total_spawn = metrics.total_spawn + 1;

        next_id = next_id + 1;
        tau = exprnd(1/spawn_rate);
        next_spawn_t = t + max(tau, min_interarrival);
    end

    %% Move objects
    for k = 1:length(objs)
        if objs(k).sorted == false
            objs(k).x = objs(k).x + v*dt;
        end
    end

    %% Remove missed objects
    for k = length(objs):-1:1
        if objs(k).sorted == false && objs(k).x > x_exit
            metrics.missed = metrics.missed + 1;
            objs(k).sorted = true;
        end
    end

    %% Sensor detection
    true_detect = false;
    detected_obj_id = -1;

    for k = 1:length(objs)
        if objs(k).sorted == false && abs(objs(k).x - x_pick) <= sensor_range

            if rand() > false_neg_rate
                true_detect = true;
                detected_obj_id = objs(k).id;
                break;
            else
                metrics.false_neg = metrics.false_neg + 1;
            end
        end
    end

    detected_now = false;

    if true_detect
        if abs(randn * detection_jitter) < 0.05
            detected_now = true;
        end
    end

    % false positive
    if rand() < false_pos_rate*dt
        detected_now = true;
        detected_obj_id = -999;
        metrics.false_pos = metrics.false_pos + 1;
    end

    %% Handle detection → Actuator
    if detected_now == true
        if act_state == 0
            act_state = 1;
            act_timer = t_react + t_act;
            metrics.act_cycles = metrics.act_cycles + 1;
            metrics.latencies(end+1) = t_react;
        end
    end

    %% Actuator timing update
    if act_state == 1
        act_timer = act_timer - dt;
        act_position = max(0, min(1, act_timer/(t_react+t_act)));

        if act_timer < t_act
            % Actively pushing
            for k = 1:length(objs)
                if objs(k).sorted == false && abs(objs(k).x - x_pick) <= sensor_range
                    objs(k).sorted = true;
                    metrics.sorted = metrics.sorted + 1;
                    break;
                end
            end
        end

        % Finish cycle
        if act_timer <= 0
            act_state = 0;
            act_position = 0;
            metrics.act_time = metrics.act_time + (t_react + t_act);
        end
    end

    %% Animation
    if mod(ti,3) == 0
        delete(obj_patches); obj_patches = [];

        for k = 1:length(objs)
            if objs(k).sorted == false
                xk = objs(k).x;
                yk = 0.08;
                ph = rectangle('Position',...
                    [xk-obj_radius, yk-obj_radius, 2*obj_radius, 2*obj_radius], ...
                    'Curvature',[1 1], ...
                    'FaceColor',[0.1 0.5 0.9], 'EdgeColor','none');
                obj_patches(end+1) = ph;
            end
        end

        theta = -0.35 * act_position;
        x2 = div_x + arm_len*cos(theta);
        y2 = arm_h + arm_len*sin(theta);
        set(arm_h_line,'XData',[div_x x2],'YData',[arm_h y2]);

        th_total.String = sprintf("t=%.1fs | Spawned=%d | Sorted=%d | Missed=%d | FP=%d | FN=%d", ...
            t, metrics.total_spawn, metrics.sorted, metrics.missed, ...
            metrics.false_pos, metrics.false_neg);

        drawnow;
    end

end

%% Results
fprintf("\n--- SIMULATION COMPLETED ---\n");
fprintf("Spawned: %d\n", metrics.total_spawn);
fprintf("Sorted: %d\n", metrics.sorted);
fprintf("Missed: %d\n", metrics.missed);
fprintf("False Positives: %d\n", metrics.false_pos);
fprintf("False Negatives: %d\n", metrics.false_neg);
fprintf("Actuator Utilization: %.2f%%\n", (metrics.act_time/T_sim)*100);
