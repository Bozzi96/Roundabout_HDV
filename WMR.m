classdef WMR
    %WMR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Roundabout
        X
        Y
        speed
        direction
        path % cell array di Arc objects
        currArc % index of path cell array
        sOnArc
        derOnArc
        curvOnArc
        stopped

        % plot only
        figObj
        textLabel
        figObjVirtPlatoon
        textLabelVirtPlatoon

        w
        l

        % Controller
        MPCobj
        lastControl

        % WMR
        ID
    end
    
    methods
        function obj = WMR(ID,initPos,initSpeed,path,sizes)
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
        end

        function obj = setController(obj, controllerObj)
            obj.MPCobj = controllerObj;
        end

        function obj = setFigureObj(obj,fig,textLabel)
            obj.figObj = fig;
            obj.textLabel = textLabel;
        end

        function obj = setFigureVirtualPlatoon(obj,fig,textLabel)
            obj.figObjVirtPlatoon = fig;
            obj.textLabelVirtPlatoon = textLabel;
        end

        function obj = move(obj)
            sMax = obj.path{obj.currArc}.arclength;
            Ts = obj.MPCobj.Ts;
            obj.speed = obj.MPCobj.x0(end);
            if obj.MPCobj.WMRNumber == 1
                accel = obj.MPCobj.u_cl(2,1);
            else
                accel = obj.MPCobj.u_cl(2,end);
            end
            obj.lastControl = obj.MPCobj.u_cl;
            
            obj.sOnArc = obj.sOnArc+Ts*obj.speed+0.5*Ts*Ts*accel;
            if obj.sOnArc > sMax % move to next arc
                obj.currArc = obj.currArc+1;
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
                poly = polyshape([obj.X+(obj.l/2) obj.X-(obj.l/2) obj.X-(obj.l/2) obj.X+(obj.l/2)],[obj.Y-(obj.w/2) obj.Y-(obj.w/2) obj.Y+(obj.w/2) obj.Y+(obj.w/2)]);
                poly_rot = rotate(poly,rad2deg(obj.derOnArc),[obj.X obj.Y]);
                obj.figObj.Shape.Vertices = poly_rot.Vertices;
%                 obj.figObj.XData = obj.X;
%                 obj.figObj.YData = obj.Y;
                obj.textLabel.Position = [obj.X+1 obj.Y+1];
                drawnow
                obj.direction = arc.direction;

                %%%%%%%%%

%                 poly = polyshape([x0+0.125 x0-0.125 x0-0.125 x0+0.125]-x0(1,1),[0 0 0.09 0.09]);
%                 obj.figObjVirtPlatoon.Shape.Vertices = poly.Vertices;
%                 obj.textLabelVirtPlatoon.Position = [x0];
            end
        end
    end
end

