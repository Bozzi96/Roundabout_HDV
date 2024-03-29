%% Clean up
clc
clear
close all

%% Definizione rotonda

Ts = 0.1;
simtime = 90/Ts;
MAX_ITER = 1;
plotTimes = false;

lf = 0.125; % dist front wheel-CoG
lr = 0.125; % dist rear wheel-CoG
l = 0.25; % car length
w = 0.18; % car width
Hp = 10; % prediction horizon
Hc = 2; % control horizon

v_init = 0.1; % initial speed

% Definition of the roundabout
Round = Roundabout([1.5 1.5],0.5,2);
% Round.plot({});

WMRs = {}; % array of wmrs close to roundabout
HDVs = {};
nextWMRs = {}; % upcoming wmrs
nextHDVs = {}; % upcoming hdvs

% WMRs{1} = WMR([1.26 1.96],{ Round.arcs{3},  Round.arcs{4},  Round.arcs{1},  Round.arcs{2} Round.arcs{11}},[w l]);
% WMRs{2} = WMR([1.76 1.96],{ Round.arcs{2},  Round.arcs{3},  Round.arcs{4},  Round.arcs{9}},[w l]);
% WMRs{3} = WMR([0 1.5],{ Round.arcs{8},  Round.arcs{4},  Round.arcs{1},  Round.arcs{10}},[w l]);

% nextWMRs{3} = WMR("CAV3",[],v_init,{ Round.arcs{8},  Round.arcs{4},  Round.arcs{1},  Round.arcs{10}},[w l]);
% nextWMRs{2} = WMR([],v_init,{ Round.arcs{6}, Round.arcs{2}, Round.arcs{3}, Round.arcs{4}, Round.arcs{1}, Round.arcs{2} Round.arcs{11} Round.arcs{7}},[w l]);
% nextWMRs{3} = WMR([],v_init,{ Round.arcs{8}, Round.arcs{4}, Round.arcs{1}, Round.arcs{10}},[w l]);
nextWMRs{1} = WMR("CAV1",[],v_init,{Round.arcs{7}, Round.arcs{3}, Round.arcs{4}, Round.arcs{1}, Round.arcs{10}},[w l]);
nextWMRs{2} = WMR("CAV2",[],v_init,{Round.arcs{7}, Round.arcs{3}, Round.arcs{4}, Round.arcs{1}, Round.arcs{10}},[w l]);
nextWMRs{3} = WMR("CAV3",[],v_init,{Round.arcs{5}, Round.arcs{1}, Round.arcs{10}},[w l]);
% nextWMRs{5} = WMR([],v_init,{ Round.arcs{5}, Round.arcs{1}, Round.arcs{10}},[w l]);

% WMRs{2} = WMR(2,[],v_init,{ Round.arcs{7},  Round.arcs{3},  Round.arcs{12}},[w l]);
% WMRs{1} = WMR(1,[1.5 3],v_init,{ Round.arcs{7}, Round.arcs{3}, Round.arcs{4},  Round.arcs{1},  Round.arcs{10}},[w l]);
% WMRs{3} = WMR(3,[],v_init,{ Round.arcs{5},  Round.arcs{1},  Round.arcs{2},  Round.arcs{11}},[w l]);
% WMRs{4} = WMR(4,[],v_init,{ Round.arcs{8},  Round.arcs{4},  Round.arcs{1},  Round.arcs{10}},[w l]);

% HDVs{1} = HDV(1,[],v_init,{ Round.arcs{7},  Round.arcs{3},  Round.arcs{12}},[w l],Round,Ts);
nextHDVs{1} = HDV("HDV1",[],v_init,{ Round.arcs{8},  Round.arcs{4},  Round.arcs{1},  Round.arcs{10}},[w l],Round,Ts);
% nextHDVs{2} = HDV("HDV2",[],v_init,{ Round.arcs{8},  Round.arcs{4},  Round.arcs{1},  Round.arcs{10}},[w l],Round,Ts);
% nextWMRs{1} = WMR(1,[],v_init,{ Round.arcs{8},  Round.arcs{4},  Round.arcs{1},  Round.arcs{10}},[w l]);
% nextWMRs{2} = WMR(2,[0.4 1.5],v_init,{ Round.arcs{8},  Round.arcs{4},  Round.arcs{1},  Round.arcs{10}},[w l]);

[Round, WMRs, HDVs] = Round.plot(WMRs,HDVs,{},[]); % roundabout plot
longPlot = figure; % roundabout plot

toDel = [];

timeOfArrivalWMRs = [1 100 30]; % contains the time instant when each vehicle arrives close to the roundabout
timeOfArrivalHDVs = [95];
numWMR = length(WMRs); % current number of vehicle in the roundabout

%% Initialization of parameters for MPC-ADMM

[Q,R] = mpcWeightMatrices(numWMR);

dmin = l+0.2; % minimum distance to keep
dmax = inf; % maximum distance to keep

rho = 5;

idxWP = ones(1,numWMR);

%% Prepare data for ADMM
% ddes = (dmax-dmin)/2;
ddes = l+0.3; % desired distance
vref = 0.15; % desired speed

u = zeros(1,numWMR); % control vector


%% Simulation

w = {}; w_prev = {[]};
xx = {};
y = {};
Z = {};

X = cell(numWMR,1);
Y = cell(numWMR,1);


x0 = {}; % cell array di tutti i platoon
platoonIdx = {}; % mapping di x0 e indice dei WMR es: {1 id1 id2 id3
%                                                      2 id3 id4 id5}
WMR_order = {[]};
HDV_indexes = {[]};
xx1 = {};
HDV_indexes_history = zeros(4,simtime);

firstTime = 1;
beta = 1.3;
t = 1;
finish = false;
while t <= simtime || finish

    % muovo gli HDVs
    for i = 1:length(HDVs)
        HDVs{i} = HDVs{i}.move(WMRs);
    end

    % Controllo se gli HDV devono essere spostati di posizione nel platoon
    for i = 1:length(HDVs)
        if HDVs{i}.direction == 1
            for ii = 1:length(xx)
                platState(:,ii) = xx{ii}(:,end);
            end
            platDists = (platState(1,:)-platState(1,1));
            [sorted, indexes] = sort(platDists,2,"descend");
            sorted(2,:) = platState(2,indexes);
            if ~isequal(WMR_order{1},indexes)
                WMR_order{1} = indexes;
                HDV_indexes = HDV_indexes(indexes);
%                 w = w(indexes);
%                 y = y(indexes);
%                 Z = Z(indexes);
                [w,xx,y,Z] = ADMMdataStructures(numWMR,WMR_order{1},sorted,HDV_indexes);
                [Q,R] = mpcWeightMatrices(length(WMR_order{1}));
                for i = 1:length(WMR_order{1})
                    if ~HDV_indexes(i)
                        ii = WMR_order{1}(i);
                        WMRs{ii} = WMRs{ii}.setController(mpc_casadi(Hp,Hc,Ts,sorted,Q{i},R{i},i,WMRs{ii}.ID,rho,dmin,dmax,WMRs{ii}.lastControl));
                    end
                end
            end
        end
    end

    % Check if a new WMRs has arrived close to the roundabout
    newWmrIdx = find(timeOfArrivalWMRs == t);
    if ~isempty(newWmrIdx)
        WMRs{end+1} = nextWMRs{newWmrIdx};
    end
    newHDVIdx = find(timeOfArrivalHDVs == t);
    if ~isempty(newHDVIdx)
        HDVs{end+1} = nextHDVs{newHDVIdx};
    end
    % platoon manager %%%%%%% TODO: da fare per ogni platoon (indice PLAT) %%%%%%%%%%%%%
    if length(WMR_order{1}) < length(cat(1,[WMRs HDVs])) || mod(t,10)==0 % new vehicle has arrived or time of update
        newWMRnum = length(WMRs) - numWMR;
        numWMR = length(WMRs);

        % Define new platoon order
        % define the current virtual platoon state
        if ~isempty(xx1)
            temp = cat(1,xx1{:});
            temp2 = temp(:,t-1);
            temp3 = reshape(temp2,2,[]);
            x0 = temp3(:,WMR_order{1});
        end
        [x0,WMR_order,HDV_indexes] = platoonManager(Round,WMRs,HDVs,numWMR,x0,WMR_order{1},HDV_indexes);

        % update data structures
        [Round, WMRs, HDVs] = Round.plot(WMRs,HDVs,x0,WMR_order);
        X{end+1} = {};
        Y{end+1} = {};
    
        %% Additional plots [virtual platoon]
        faceColorsWMR = [0.64,0.08,0.18
                         0.00,0.45,0.74
                         0.85,0.33,0.10
                         0.93,0.69,0.13
                         0.49,0.18,0.56];

        figure(longPlot);
        clf(longPlot)
        hold on
        set(gca, 'YGrid', 'off', 'XGrid', 'on')
        axis image
        line([0 -3],[0 0],'color','k')
        plat = 1;
        vehicles = cat(1,[WMRs HDVs]);
        for i = 1:length(vehicles)
            wmrId = vehicles{i}.ID;
            jj = WMR_order{plat}(i);
            poly = polyshape([x0{plat}(1,jj)+0.125 x0{plat}(1,jj)-0.125 x0{plat}(1,jj)-0.125 x0{plat}(1,jj)+0.125]-x0{plat}(1,WMR_order{plat}(1)),[0 0 0.09 0.09]);
            textLabel(i) = text(x0{plat}(1,jj)+0.1-x0{plat}(1,WMR_order{plat}(1)),0.1,string(wmrId));
            wmrFig(i) = plot(poly,'FaceColor',faceColorsWMR(i,:),'FaceAlpha',0.35);
%             wmrFig(i) = plot(poly,'FaceAlpha',0.35);
        end
        drawnow
    
        %% ADMM data structures
        for plat = 1:length(x0)
            [w,xx,y,Z] = ADMMdataStructures(numWMR,WMR_order{plat},x0{plat},HDV_indexes);
            [Q,R] = mpcWeightMatrices(length(WMR_order{plat}));
            for i = 1:length(WMR_order{1})
                if ~HDV_indexes(i)
                    ii = WMR_order{plat}(i);
                    WMRs{ii} = WMRs{ii}.setController(mpc_casadi(Hp,Hc,Ts,x0{plat},Q{i},R{i},i,WMRs{ii}.ID,rho,dmin,dmax,WMRs{ii}.lastControl));
                end
            end
        end
    end

    %% ADMM - MPC
    for plat = 1:length(x0)
        if numWMR > 0
            for it = 1:MAX_ITER
                % Solver
                for i = 1:length(WMR_order{plat})
                    k = WMR_order{plat}(i);
                    if ~HDV_indexes(i)
                        y_temp{k} = y{k} - rho*(1-beta)*(reshape(xx{k},[],1) - Z{k});
                        if i == 1
                            xs = vref;
                            WMRs{k}.MPCobj = WMRs{k}.MPCobj.step(xs,Z{k},y_temp{k});
                        else
                            if i == 2
                                xs = [ddes; vref; vref];
                                WMRs{k}.MPCobj = WMRs{k}.MPCobj.step(xs,Z{k},y_temp{k});
                            else
                                xs = [ddes; (i-1)*ddes; ddes; vref; vref; vref];
                                WMRs{k}.MPCobj = WMRs{k}.MPCobj.step(xs,Z{k},y_temp{k});
                            end
                        end
                            
                        xx{k} = reshape(WMRs{k}.MPCobj.x0,2,[]);
                        w{k} = reshape(WMRs{k}.MPCobj.u_cl(2,:),1,[]);
                    else
                        hdvIndex = k-length(WMRs);
                        xx{k}(:,end) = [xx{k}(1,end) + Ts*HDVs{hdvIndex}.speed
                                        HDVs{hdvIndex}.speed];
                    end
                end
        
                % calcolo la media delle variabili condivise per consensus admm
                Z{WMR_order{plat}(1)} = 0;
                for j = 1:length(WMR_order{plat})
                    jj = WMR_order{plat}(j);
                    if ~HDV_indexes(j)
                        if j == 1
                            for k = 1:numWMR
                                if ~HDV_indexes(j)
                                    Z{jj} = Z{jj}+xx{k}(:,1);
                                end
                            end
                            Z{jj} = Z{jj}/numWMR;
                        else
                            if j == 2
                                Z{jj} = [Z{WMR_order{plat}(1)}
                                        (xx{jj}(:,end))];%+xx{WMR_order{plat}(j+1)}(:,2))/2];
                            else
                                if j < length(WMR_order{plat})
                                    if ~HDV_indexes(j-1) && ~HDV_indexes(j+1) % se davanti a me e dietro NON ci sono HDV
                                        Z{jj} = [Z{WMR_order{plat}(1)}
                                                (xx{WMR_order{plat}(j-1)}(:,end)+xx{jj}(:,2))/2
                                                (xx{jj}(:,end)+xx{WMR_order{plat}(j+1)}(:,2))/2];
                                    else
                                        if ~HDV_indexes(j-1) % se SOLO davanti c'è l'HDV
                                           Z{jj} = [Z{WMR_order{plat}(1)}
                                                    xx{jj}(:,2)
                                                    (xx{jj}(:,end)+xx{WMR_order{plat}(j+1)}(:,2))/2];
                                        else % se SOLO dietro c'è l'HDV
                                            Z{jj} = [Z{WMR_order{plat}(1)}
                                                    (xx{WMR_order{plat}(j-1)}(:,end)+xx{jj}(:,2))/2
                                                     xx{jj}(:,end)];
                                        end
                                    end
                                else
                                    if ~HDV_indexes(j-1) % guardo davanti
                                        Z{jj} = [Z{WMR_order{plat}(1)}
                                                (xx{WMR_order{plat}(j-1)}(:,end)+xx{jj}(:,2))/2
                                                xx{jj}(:,end)];
                                    else
                                        Z{jj} = [Z{WMR_order{plat}(1)}
                                            xx{jj}(:,2)
                                            xx{jj}(:,end)];
                                    end
                                end
                            end
                        end
                    end
                end
        
                % aggiorno Z media per ogni variabile condivisa e aggiorno moltiplicatori
                for j = 1:length(WMR_order{plat})
                    jj = WMR_order{plat}(j);
                    if ~HDV_indexes(j)
                        y{jj} = y_temp{jj} + rho*(reshape(xx{jj},[],1) - Z{jj});
                    end
                end
            end
    
            for j = 1:length(WMR_order{plat})
                jj = WMR_order{plat}(j);
                if j == 1
                    xx1{jj}(:,t) = xx{jj}(:,1);
                else
                    if j == 2
                        xx1{jj}(:,t) = xx{jj}(:,2);
                    else
                        xx1{jj}(:,t) = xx{jj}(:,end);
                    end
                end  
            end
            HDV_indexes_history(1:length(WMR_order{1}),t) = HDV_indexes;
        
            u = [];
            for j = 1:length(WMR_order{plat})
                jj = WMR_order{plat}(j);
                if j == 1
                    u = [u w{jj}(1)];
                else
                    if j == 2
                        u = [u w{jj}(2)];
                    else
                        u = [u w{jj}(end)];
                    end
                end 
            end
        
            % Check if a vehicle has exit the roundabout (outOfRange)
            i = 1;
            while i<=numWMR
                if ~WMRs{i}.stopped
                    WMRs{i} = WMRs{i}.move();
                    X{i} = [X{i} WMRs{i}.X];
                    Y{i} = [Y{i} WMRs{i}.Y];
                else
                    delete(WMRs{i}.figObj)
                    numWMR = numWMR - 1;
                    WMRs(i) = [];
                    X(i) = [];
                    Y(i) = [];
                    i = i-1;
    
                    % Define new platoon order
                    [x0,WMR_order] = platoonManager(Round,WMRs,HDVs,numWMR,x0);
                    % Define ADMM data structures
                    [w,xx,y,Z] = ADMMdataStructures(numWMR,WMR_order,x0);
                end
                i = i + 1;
            end
        end
    end
    figure(longPlot)
    delete(wmrFig)
    delete(textLabel)
    for i = 1:length(vehicles)
        jj = WMR_order{plat}(i);
        wmrId = vehicles{jj}.ID;
        poly = polyshape([xx1{jj}(1,t)+0.125 xx1{jj}(1,t)-0.125 xx1{jj}(1,t)-0.125 xx1{jj}(1,t)+0.125]-xx1{WMR_order{plat}(1)}(1,t),[0 0 0.09 0.09]);
        textLabel(i) = text(xx1{jj}(1,t)+0.1-xx1{WMR_order{plat}(1)}(1,t),0.1,string(wmrId));
        wmrFig(i) = plot(poly,'FaceColor',faceColorsWMR(i,:),'FaceAlpha',0.35);
%                 wmrFig(i) = plot(poly,'FaceAlpha',0.35);
    end
    drawnow

    t = t + 1;
    pause(Ts);
end

%% Plots
close all
time = (1:length(xx1{1}))*Ts;
% close all
figure

% subplot(3,1,1)
grid on
hold on
title("Longitudinal Displacement [m]",'FontSize',15)
xlabel("Time [s]",'FontSize',15)
lgd = {};
for i = 1:size(xx1,2)
    for ii = size(xx1,1)
        if any(HDV_indexes_history(i,:) == 1)
            plot(time,xx1{i}(1,:),'DisplayName',strcat('HDV_',num2str(i)),'LineWidth',3);
        else
            plot(time,xx1{i}(1,:),'DisplayName',strcat('WMR_',num2str(i)),'LineWidth',3);
        end
    end
end
legend("Location","bestoutside")

figure
% subplot(3,1,2)
grid on
hold on
title("Distances [m]",'FontSize',15)
xlabel("Time [s]",'FontSize',15)
lgd = {};
for i = 1:length(xx1)-1
    if HDV_indexes_history(i,:) == 1
        plot(time,xx1{WMR_order{1}(i)}(1,:)-xx1{WMR_order{1}(i+1)}(1,:),'LineWidth',3,'DisplayName',strcat('HDV_',num2str(WMR_order{1}(i)),'-WMR_',num2str(WMR_order{1}(i+1))));
    else
        plot(time,xx1{WMR_order{1}(i)}(1,:)-xx1{WMR_order{1}(i+1)}(1,:),'LineWidth',3,'DisplayName',strcat('WMR_',num2str(WMR_order{1}(i)),'-WMR_',num2str(WMR_order{1}(i+1))));
    end
end
plot(time, dmin*ones(1,length(time)),'b--','DisplayName','Bounds','LineWidth',3);
% plot(time, dmax*ones(1,length(time)),'b--',time, dmin*ones(1,length(time)),'b--','DisplayName','Bounds','LineWidth',3);
plot(time, ddes*ones(1,length(time)),'r--','DisplayName','Reference','LineWidth',3);
legend("Location","bestoutside")

figure
% subplot(3,1,3)
grid on
hold on
title("Velocity [m/s]",'FontSize',15)
xlabel("Time [s]",'FontSize',15)
for i = 1:length(xx1)
    if HDV_indexes_history(i,:) == 1
        plot(time,xx1{WMR_order{1}(i)}(2,:),'LineWidth',3,'DisplayName',strcat('HDV_',num2str(WMR_order{1}(i))));
    else
        plot(time,xx1{WMR_order{1}(i)}(2,:),'LineWidth',3,'DisplayName',strcat('WMR_',num2str(WMR_order{1}(i))));
    end
end
yline(vref,'--r','DisplayName','Reference','LineWidth',3)
legend(lgd,"Location","bestoutside")