classdef HDV
    %WMR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Roundabout
        X
        Y
        speed
        direction
        h_eq
        h
        v_eq
        vc_prev

        path % cell array di Arc objects
        currArc % index of path cell array
        sOnArc
        derOnArc
        curvOnArc
        stopped

        % Model params
        alpha = 0.2 % headway gain
        beta = 0.9 % relative velocity gain

        % plot only
        figObj
        textLabel
        w
        l

        % General
        pathJoints
        pathJointsIndex
        currObjective
        isJoint
        coneHandleLarge
        coneHandleSmall
        rangeBig = [60 60]
        distanceBig = 50
        rangeSmall = [45 45]
        distanceSmall = 35
        Ts

        % WMR
        ID
        speedHistory
    end
    
    methods
        function obj = HDV(ID,initPos,initSpeed,path,sizes,Round,Ts)
            if ~isempty(ID)
                obj.ID = ID;
            else
                obj.ID = randi(10000,1);
            end
            if isempty(initPos)
                obj.X = path{1}.Joints(1,1);
                obj.Y = path{1}.Joints(1,2);
            else
                obj.X = initPos(1);
                obj.Y = initPos(2);
            end
            obj.speed = initSpeed;
            obj.speedHistory = [];
            obj.vc_prev = initSpeed;
            obj.h_eq = 10;
            obj.v_eq = 15;
            obj.h = 100;
%             [obj,obj.v_eq] = obj.computeSpeed(obj.h_eq,false);

            obj.path = path;
            obj.currArc = 1;
            obj.sOnArc = 0;
            obj.derOnArc = obj.path{obj.currArc}.derivative(1);
            obj.curvOnArc = [obj.path{obj.currArc}.curvature obj.path{obj.currArc+1}.curvature];
            obj.stopped = 0;
            obj.direction = obj.path{obj.currArc}.direction;

            obj.w = sizes(1);
            obj.l = sizes(2);

            if ~isempty(initPos)
                pts = obj.path{obj.currArc}.points;
                [~,i] = min(pdist2(initPos,pts'));
                obj.sOnArc = obj.path{obj.currArc}.s(i);
                obj.derOnArc = obj.path{obj.currArc}.derivative(i);
                obj.curvOnArc = [obj.path{obj.currArc}.curvature(i:end) obj.path{obj.currArc+1}.curvature];
            end

            obj.pathJointsIndex = [];
            pJtnumb = 1;
            for i = 1:length(obj.path)
                tempJt = obj.path{i}.Joints(end,:);
                idxs = sum(tempJt == Round.roundJoint,2) == 2;
                if any(idxs)
                    obj.pathJoints(pJtnumb,:) = Round.roundJoint(idxs,:);
                    obj.pathJointsIndex(end+1) = i;
                    pJtnumb = pJtnumb + 1;
                end
            end
            obj.pathJoints = obj.pathJoints([1 end],:);
            obj.pathJointsIndex = obj.pathJointsIndex([1 end]);
            obj.currObjective = 1;
            obj.Ts = Ts;
            obj.isJoint = false;

        end

        function obj = modelUpdate(obj, WMRs)            
            [obj,obj.h,v_front,obj.isJoint] = obj.checkFrontalSpace(WMRs);
            [obj,vc] = obj.computeSpeed(obj.h_eq,obj.isJoint);
%             acc = (vc-obj.vc_prev)/obj.Ts;
%             obj.vc_prev = vc;

            tf = 1/vc;

            F = [0 -1
                 obj.alpha/tf -obj.alpha-obj.beta];
            G = [0 1
                 0 obj.beta];
            H = [0; obj.alpha*(obj.v_eq-obj.h_eq/tf)];

            newState = [obj.h; obj.speed] + obj.Ts*(F*[obj.h; obj.speed]+G*[obj.h; v_front]+H);
            obj.h = newState(1);
            if newState(2)>20, newState(2)=20; end
            obj.speed = newState(2);
            obj.speedHistory(end+1) = obj.speed;
            obj.sOnArc = obj.sOnArc + obj.Ts*obj.speed;

            
%             obj.coneHandleLarge = plotSectionCone(obj.coneHandleLarge,obj.derOnArc,obj.rangeBig,obj.X,obj.Y,obj.distanceBig);
%             obj.coneHandleSmall = plotSectionCone(obj.coneHandleSmall,obj.derOnArc,obj.rangeSmall,obj.X,obj.Y,obj.distanceSmall);
        end

        function [obj,v] = computeSpeed(obj,h,isJoint)
            if isJoint
                h0 = 0;
                hg = 20;
            else
                h0 = 5;
                hg = 35;
            end
            vm = 15;
            if h < h0
                v = 0;
            else
                if h > h0 && h < hg
%                     v = vm/2*(1-cos(pi*(h-h0)/(hg-h0)));
                    v = (vm*pi*sin((pi*(h - h0))/(h0 - hg)))/(2*(h0 - hg));
                else
%                     v = vm;
                    v = pi/4; % massimo in pi/4
                end
            end
        end

        function [obj,h,v_front,isJoint] = checkFrontalSpace(obj,WMRs)
            % prendo tutte le posizioni delle altre macchine
            wmrs = {};
            idxs = [];
            isJoint = false;
            for i = 1:length(WMRs)
                if (obj.direction == 1 && WMRs{i}.direction == 0) || (obj.direction == 0 && WMRs{i}.direction == -1)
                    if is_point_in_section(obj.X, obj.Y,obj.derOnArc, obj.distanceBig, WMRs{i}.X, WMRs{i}.Y,[obj.rangeBig(1) 0])
                        wmrs{end+1} = WMRs{i};
                        idxs(end+1) = -1;
                    end
                    if is_point_in_section(obj.X, obj.Y,obj.derOnArc, obj.distanceBig, WMRs{i}.X, WMRs{i}.Y,[0 obj.rangeBig(2)])
                        wmrs{end+1} = WMRs{i};
                        idxs(end+1) = 1;
                    end
                else
                    if (obj.direction == WMRs{i}.direction && isequal(obj.path{obj.currArc},WMRs{i}.path{WMRs{i}.currArc})) || (obj.direction == 0 && WMRs{i}.direction == 0)
                        if is_point_in_section(obj.X, obj.Y,obj.derOnArc, obj.distanceSmall, WMRs{i}.X, WMRs{i}.Y,obj.rangeSmall)
                            wmrs{end+1} = WMRs{i};
                            idxs(end+1) = 0;
                        end
                    end
                end
            end
            % Misuro la distanza dal prossimo joint
            v_front = 0;
            h = obj.path{obj.currArc}.arclength-obj.sOnArc;
            if obj.currObjective <= size(obj.pathJoints,1)
                for i = obj.currArc+1:obj.pathJointsIndex(obj.currObjective)
                    h = h + obj.path{i}.arclength;
                end
            else
                h = obj.path{obj.currArc}.arclength;
            end
            
            % Se non ci sono macchine in vista
            for i = 1:length(wmrs)
                % Se i due mezzi sono sullo stesso arco, diventa
                % l'obiettivo
                if isequal(obj.path{obj.currArc},wmrs{i}.path{wmrs{i}.currArc})
                    d2(i) = (obj.path{obj.currArc}.arclength-obj.sOnArc)-(wmrs{i}.path{wmrs{i}.currArc}.arclength-wmrs{i}.sOnArc);
                else
                    if obj.direction == 1 % se sto entrando
                        % controllo a sinistra
                        if is_point_in_section(obj.X, obj.Y,obj.derOnArc, obj.distanceBig, wmrs{i}.X, wmrs{i}.Y,[obj.rangeBig(1) 0])
                            d2(i) = (obj.path{obj.currArc}.arclength-obj.sOnArc) + (wmrs{i}.path{wmrs{i}.currArc}.arclength-wmrs{i}.sOnArc);
                        else
                            % controllo a destra
                            if is_point_in_section(obj.X, obj.Y,obj.derOnArc, obj.distanceBig, wmrs{i}.X, wmrs{i}.Y,[0 obj.rangeBig(2)])
                                d2(i) = (obj.path{obj.currArc}.arclength-obj.sOnArc) + wmrs{i}.sOnArc;
                            end
                        end
                    else % altrimenti se sono nella rotonda o sto uscendo
                        d2(i) = (obj.path{obj.currArc}.arclength-obj.sOnArc);
                        arcNum = obj.currArc;
                        for ii = obj.currArc:length(obj.path) % cerco l'arco in cui si trova il WMR rispetto all'HDV. Lo cerco esplorando il percorso dell'HDV
                            if isequal(obj.path{ii},wmrs{i}.path{wmrs{i}.currArc})
                                arcNum = ii;
                            end
                        end
                        % una volta trovato, prendo la distanza
                        for ii = obj.currArc+1:arcNum-1
                            d2(i) = d2(i) + obj.path{ii}.arclength;
                        end
                        d2(i) = d2(i) + wmrs{i}.sOnArc;
                    end
                end
            end
            if isempty(wmrs)
                isJoint = true;
                obj.h_eq = 0;
            else % altrimenti
                if any(idxs == 0) % do priorità ai mezzi sul mio stesso arco
                    dists = d2(idxs==0);
                    [~, I] = min(dists);
                    h = dists(I);
                else
                    dists = d2(idxs~=0);
                    [~, I] = min(dists);
                    h = dists(I);
                end
                v_front = wmrs{I}.speed;
                obj.h_eq = 10;
            end
%             if h > 35, h = 35; end
        end

        function obj = setFigureObj(obj,fig,textLabel)
            obj.figObj = fig;
            obj.textLabel = textLabel;
        end

        function obj = move(obj,WMRs)
            obj = obj.modelUpdate(WMRs);
            sMax = obj.path{obj.currArc}.arclength;
            if obj.h < 0.2 && obj.isJoint
                obj.currObjective = obj.currObjective + 1;
%                 if obj.currObjective <= size(obj.pathJoints,1)
%                     obj.h = obj.path{obj.currArc}.arclength - obj.sOnArc;
%                     for i = obj.currArc:obj.pathJointsIndex(obj.currObjective)
%                         obj.h = obj.h + obj.path{i}.arclength;
%                     end
%                 else
%                     obj.h = 2*obj.path{obj.currArc}.arclength; %%%%%%%%%%%%%%%%%% TAPULLO %%%%%%%%%%%%%%
%                 end
            end
            if obj.sOnArc > sMax % move to next arc
                obj.currArc = obj.currArc+1;
                if isequal(obj.path{obj.currArc}.Joints(1,:),obj.pathJoints(1,:))
                    obj.currObjective = obj.currObjective + 1;
                end
%                 if obj.currArc > length(obj.path)  % loop
%                     obj.currArc = 1;
%                 end
                if obj.currArc > length(obj.path)  % stop
                    obj.stopped = 1;
                end
                obj.sOnArc = obj.sOnArc - sMax;
            end
            % update WMR position if it has to continue
            if ~obj.stopped
                arc = obj.path{obj.currArc};
                [~,idxPt] = min(abs(arc.s-obj.sOnArc));
                obj.X = arc.points(1,idxPt);
                obj.Y = arc.points(2,idxPt);
                obj.derOnArc = arc.derivative(idxPt);
                if obj.currArc+1 <= length(obj.path)
                    arcNext = obj.path{obj.currArc+1};
                    obj.curvOnArc = [arc.curvature(idxPt:end) arcNext.curvature];
                else
                    obj.curvOnArc = [arc.curvature(idxPt:end) arc.curvature(end)*ones(1,length(arc.curvature))];
                end
                
                % update figure
                poly = polyshape([obj.X+(obj.l/2) obj.X-(obj.l/2) obj.X-(obj.l/2) obj.X+(obj.l/2)],...
                                 [obj.Y-(obj.w/2) obj.Y-(obj.w/2) obj.Y+(obj.w/2) obj.Y+(obj.w/2)]);
                poly_rot = rotate(poly,rad2deg(obj.derOnArc),[obj.X obj.Y]);
                obj.figObj.Shape.Vertices = poly_rot.Vertices;
%                 obj.figObj.XData = obj.X;
%                 obj.figObj.YData = obj.Y;
                obj.textLabel.Position = [obj.X+0.1 obj.Y+0.1];
                obj.textLabel.String = string(obj.ID+" | "+obj.h);
%                 obj.textLabel = text(obj.X+0.1,obj.Y+0.1,string(obj.ID+" | "+obj.h));
                drawnow
                obj.direction = arc.direction;
            end
        end
    end
end

