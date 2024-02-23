classdef Roundabout
    %Roundabout Definizione Rotonda
    %   Detailed explanation goes here
    
    properties
        arcs
        app_arcs
        leav_arcs
        round_arcs
        plt % plot figure object
        longPlot
        roundJoint
        net % matrix that contains the connection between arcs

        % Figure
        figureSize
        center

        faceColorsWMR = [0.64,0.08,0.18
                         0.00,0.45,0.74
                         0.85,0.33,0.10
                         0.93,0.69,0.13
                         0.49,0.18,0.56];
    end
    
    methods
        function obj = Roundabout(center,radius,lanes_length)
            cx = center(1);
            cy = center(2);
            xoff = 0.01;
            yoff = 0.01;
            obj.figureSize = (radius+lanes_length);
            obj.center = [cx cy];
            % Roundabout
            obj.arcs{1} = Arc('r1',[cx cy-radius; cx+radius cy-radius; cx+radius cy],[0 0],0);
            obj.arcs{2} = Arc('r2',[cx+radius cy; cx+radius cy+radius; cx cy+radius],[0 0],0);
            obj.arcs{3} = Arc('r3',[cx cy+radius; cx-radius cy+radius; cx-radius cy],[0 0],0);
            obj.arcs{4} = Arc('r4',[cx-radius cy; cx-radius cy-radius; cx cy-radius],[0 0],0);
            obj.roundJoint = [cx+radius cy
                              cx cy+radius
                              cx-radius cy
                              cx cy-radius];
            % Approaching lanes
            obj.arcs{5} = Arc('a1',[cx cy-radius-lanes_length; cx cy-radius],[xoff 0],1);
            obj.arcs{6} = Arc('a2',[cx+radius+lanes_length cy; cx+radius cy],[0 yoff],1);
            obj.arcs{7} = Arc('a3',[cx cy+radius+lanes_length; cx cy+radius],-[xoff 0],1);
            obj.arcs{8} = Arc('a4',[cx-radius-lanes_length cy; cx-radius cy],-[0 yoff],1);
            % Exit lanes
            obj.arcs{9} = Arc('e1',[cx cy-radius; cx cy-radius-lanes_length],-[xoff 0],-1);
            obj.arcs{10} = Arc('e2',[cx+radius cy; cx+radius+lanes_length cy],-[0 yoff],-1);
            obj.arcs{11} = Arc('e3',[cx cy+radius; cx cy+radius+lanes_length],[xoff 0],-1);
            obj.arcs{12} = Arc('e4',[cx-radius cy; cx-radius-lanes_length cy],[0 yoff],-1);

            obj.net = zeros(length(obj.arcs));
            obj.net(1,[2 10]) = 1;
            obj.net(2,[3 11]) = 1;
            obj.net(3,[4 12]) = 1;
            obj.net(4,[1 5]) = 1;
            obj.net(5,[1]) = 1;
            obj.net(6,[2]) = 1;
            obj.net(7,[3]) = 1;
            obj.net(8,[4]) = 1;

            obj.round_arcs = obj.arcs(1:4);
            obj.app_arcs = obj.arcs(5:8);
            obj.leav_arcs = obj.arcs(9:12);
        end

        function [obj,WMRs,HDVs] = plot(obj,WMRs,HDVs,xx1,WMR_order)
            if isempty(obj.plt)
                f = figure;
                obj.plt = f;
            else
                figure(obj.plt)
                clf(obj.plt)
            end
            hold on
            grid on
%             axis([obj.center(1)-obj.figureSize obj.center(1)+obj.figureSize obj.center(2)-obj.figureSize obj.center(2)+obj.figureSize])
            axis image
            for i = 1:length(obj.arcs)
                plot(obj.arcs{i}.points(1,:),obj.arcs{i}.points(2,:),'b')
            end
            plot(obj.roundJoint(:,1),obj.roundJoint(:,2),'Xr','MarkerSize',10)
            % plot WMRs if any
            if ~isempty(WMRs)
                for i = 1:length(WMRs)
                    poly = polyshape([WMRs{i}.X+(WMRs{i}.l/2) WMRs{i}.X-(WMRs{i}.l/2) WMRs{i}.X-(WMRs{i}.l/2) WMRs{i}.X+(WMRs{i}.l/2)],[WMRs{i}.Y-(WMRs{i}.w/2) WMRs{i}.Y-(WMRs{i}.w/2) WMRs{i}.Y+(WMRs{i}.w/2) WMRs{i}.Y+(WMRs{i}.w/2)]);
                    poly_rot = rotate(poly,rad2deg(WMRs{i}.derOnArc),[WMRs{i}.X WMRs{i}.Y]);
%                     wmrFig = plot(WMRs{i}.X,WMRs{i}.Y,'Or');
                    wmrFig = plot(poly_rot);
                    textLabel = text(WMRs{i}.X+0.1,WMRs{i}.Y+0.1,string(WMRs{i}.ID));
                    WMRs{i} = WMRs{i}.setFigureObj(wmrFig,textLabel);
                end
            end

            if ~isempty(HDVs)
                for i = 1:length(HDVs)
                    poly = polyshape([HDVs{i}.X+(HDVs{i}.l/2) HDVs{i}.X-(HDVs{i}.l/2) HDVs{i}.X-(HDVs{i}.l/2) HDVs{i}.X+(HDVs{i}.l/2)],[HDVs{i}.Y-(HDVs{i}.w/2) HDVs{i}.Y-(HDVs{i}.w/2) HDVs{i}.Y+(HDVs{i}.w/2) HDVs{i}.Y+(HDVs{i}.w/2)]);
                    poly_rot = rotate(poly,rad2deg(HDVs{i}.derOnArc),[HDVs{i}.X HDVs{i}.Y]);
%                     wmrFig = plot(WMRs{i}.X,WMRs{i}.Y,'Or');
                    wmrFig = plot(poly_rot);
                    textLabel = text(HDVs{i}.X+0.1,HDVs{i}.Y+0.1,string(HDVs{i}.ID+" | "+HDVs{i}.h));
                    HDVs{i} = HDVs{i}.setFigureObj(wmrFig,textLabel);
                end
            end

%             if isempty(obj.longPlot)
%                 obj.longPlot = figure;
%             else
%                 figure(obj.longPlot)
%                 clf(obj.longPlot)
%             end
%             vehicles = cat(1,[WMRs HDVs]);
%             for i = 1:length(vehicles)
%                 jj = WMR_order{1}(i);
%                 wmrId = vehicles{jj}.ID;
%                 poly = polyshape([xx1{jj}(1,t)+0.125 xx1{jj}(1,t)-0.125 xx1{jj}(1,t)-0.125 xx1{jj}(1,t)+0.125]-xx1{WMR_order{plat}(1)}(1,t),[0 0 0.09 0.09]);
%                 textLabel(i) = text(xx1{jj}(1,t)+0.1-xx1{WMR_order{1}(1)}(1,t),0.1,string(wmrId));
%                 wmrFig(i) = plot(poly,'FaceColor',obj.faceColorsWMR(i,:),'FaceAlpha',0.35);
%                 vehicles{i}.setFigureVirtualPlatoon(wmrFig(i),textLabel(i));
%             end

        end
        
    end
end

