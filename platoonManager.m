function [x0,WMR_order,HDV_indexes] = platoonManager(Round,WMRs,HDVs,numWMR,x0_prev,WMR_order_prev,platoonsIdx_prev)
    % Calcolo distanze e priorità
    % 1 - prendere distanze dai Joint della rotonda
    jts = Round.roundJoint;
    xx = [];
    dist = [];
    vehicles = cat(1,[WMRs(:)',HDVs(:)']);
    numWMR = length(vehicles);
    for i = 1:length(vehicles)
        if any(vehicles{i}.direction == [0 1]) % considero solo i mezzi che sono in rotonda o si stanno avvicinando
            currArc = vehicles{i}.currArc;
            wmr_path = vehicles{i}.path;
            jtwmr = [];
            for j = currArc:length(wmr_path)
                jtwmr(j,:) = wmr_path{j}.Joints(end,:);
            end
            [~,idx{i}] = intersect(jts,jtwmr,'rows');
        end
    end
    commonJt = idx{1};
    for i = 2:length(idx)
        commonJt = intersect(commonJt,idx{i});
    end
    % 3 - calcolo la distanza di ogni WMR ai joint comuni
    for i = 1:length(commonJt)
        for j = 1:length(vehicles)
            wmr_path = vehicles{j}.path;
            currArc = vehicles{j}.currArc;
            for k = 1:length(wmr_path)
                if wmr_path{k}.Joints(end,:) == Round.roundJoint(commonJt(i),:)
                    dist(i,j) = wmr_path{currArc}.arclength-vehicles{j}.sOnArc;
                    for kk = currArc+1:k
                        dist(i,j) = dist(i,j) + wmr_path{kk}.arclength;
                    end
                end
            end
        end
    end

    if ~isempty(dist)
        for i = 1:numWMR
            speed(i) = vehicles{i}.speed;
        end
        [~,I] = min(min(dist,[],2));
        [ordDist,WMR_order{1}] = sort(dist(I,:));
        HDV_indexes = (WMR_order{1} > length(WMRs)); % controllo chi è HDV
        % se HDV è il primo, lo escludo
        if HDV_indexes(1)
            virtPlatPos = -(ordDist(2:end)-ordDist(2));
            x0{1} = [virtPlatPos' speed(2:end)']';
            WMR_order{1}(1) = [];
            HDV_indexes(1) = []; 
        else
            virtPlatPos = -(ordDist-ordDist(1));
            if ~isempty(x0_prev)
                virtPlatPos(1,:) = virtPlatPos(1,:) + x0_prev(1,1);
            end
            x0{1} = [virtPlatPos' speed']';
        end
        
        
    else
        x0{1} = x0_prev;
        WMR_order{1} = WMR_order_prev;
        HDV_indexes = platoonsIdx_prev;
    end




%     leav_arcs = Round.leav_arcs;
%     leaving_WMRs = {};
%     inRound_WMRs = {};
%     approaching_WMRs = {};
%     toa = 1000*ones(length(jts),length(WMRs));
% 
%     for i = 1:length(WMRs)
%         if WMRs{i}.direction == 1
%             approaching_WMRs{end+1} = WMRs{i};
%         else
%             if WMRs{i}.direction == 0
%                 inRound_WMRs{end+1} = WMRs{i};
%             else
%                 leaving_WMRs{end+1} = WMRs{i};
%             end
%         end
%     end
%     % calcolo i joint comuni tra i wmr che si avvicinano o sono già in
%     % rotonda
%     for i = 1:length(WMRs)
%         if any(WMRs{i}.direction == [0 1]) % considero solo i mezzi che sono in rotonda o si stanno avvicinando
%             currArc = WMRs{i}.currArc;
%             wmr_path = WMRs{i}.path;
%             jtwmr = [];
%             for j = currArc:length(wmr_path)
%                 jtwmr(j,:) = wmr_path{j}.Joints(end,:);
%             end
%             jtwmr(end-1:end,:) = []; % rimuovo ultimi due joint
%             [~,idx{i}] = intersect(jts,jtwmr,'rows');
%         end
%     end
%     
%     x0 = {};
%     WMR_order = {};
%     for j = 1:length(idx)
%         for i = 1:length(idx)
%             commonJt{j,i} = intersect(idx{j},idx{i});
% 
%             wmr_path = WMRs{j}.path;
%             currArc = WMRs{j}.currArc;
%             for ii = 1:length(commonJt{j,i})
%                 for k = 1:length(wmr_path)
%                     if wmr_path{k}.Joints(end,:) == jts(commonJt{j,i}(ii),:)
%                         toa(commonJt{j,i}(ii),j) = wmr_path{currArc}.arclength-WMRs{j}.sOnArc;
%                         for kk = currArc+1:k
%                             toa(commonJt{j,i}(ii),j) = toa(commonJt{j,i}(ii),j) + wmr_path{kk}.arclength;
%                         end
%                     end
%                 end
%             end
%         end
%     end
%     [m,I]=min(toa,[],1);
%     
%     if isempty(commonJt) || length(idx) == 1
%         x0{end+1} = [0; WMRs{j}.speed];
%         WMR_order{end+1} = 1;
%     else
%         dist = [];
%         for i = 1:length(commonJt)
%             for j = 1:length(WMRs)
%                 wmr_path = WMRs{j}.path;
%                 currArc = WMRs{j}.currArc;
%                 for k = 1:length(wmr_path)
%                     if wmr_path{k}.Joints(end,:) == Round.roundJoint(commonJt{i},:)
%                         dist(i,j) = wmr_path{currArc}.arclength-WMRs{j}.sOnArc;
%                         for kk = currArc+1:k
%                             dist(i,j) = dist(i,j) + wmr_path{kk}.arclength;
%                         end
%                     end
%                 end
%             end
%         end
%     
%         if ~isempty(dist)
%             for i = 1:numWMR
%                 speed(i) = WMRs{i}.speed;
%             end
%             [~,I] = min(min(dist,[],2));
%             [ordDist,WMR_order] = sort(dist(I,:));
%             virtPlatPos = -(ordDist-ordDist(1));
%             x0 = [virtPlatPos' speed']';
%         else
%             x0 = x0_prev;
%         end
%     end
end

