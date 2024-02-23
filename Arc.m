classdef Arc
    %ARC Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        ID
        Joints {mustBeNumeric} % matrix degree X 2, each row contains a pivot. The first and last one are the joints
        degree {mustBeNumeric}
        pivot {mustBeNumeric}
        points {mustBeNumeric}
        derivative {mustBeNumeric}
        curvature {mustBeNumeric}
        s {mustBeNumeric}
        arclength {mustBeNumeric}
        direction {mustBeNumeric}
    end
    
    methods
        function obj = Arc(ID,joints,offset,dir)
            %ARC Construct an instance of this class
            %   Detailed explanation goes here
            obj.ID = ID;
            obj.Joints = joints([1 end],:);
            obj.degree = size(joints,1)-1;
            obj.pivot = joints+offset;
            obj.direction = dir;
            obj = obj.computesPoints();
        end
        function obj = computesPoints(obj)
            arc_res = 0.005;
            pts = [];
            ptsDer = [];
            obj.s = [];
            n = obj.degree;
            l = 0;
            prev = obj.pivot(1,:);
            for t = 0:arc_res:1
                bern = 0;
                for i = 0:n
                    bern = bern + (factorial(n)/(factorial(i)*factorial(n-i)))*t^i*(1-t)^(n-i).*obj.pivot(i+1,:);
                end
                l = l + pdist([prev; bern]);
                obj.s(end+1) = l;
                prev = bern;
                pts(:,end+1) = bern;

%                 % Curvature
%                 P1_d1 = 2*(obj.Joints(:,2)-obj.Joints(:,1));
%                 P2_d1 = 2*(obj.Joints(:,3)-obj.Joints(:,2));
%                 P1_d2 = P2_d1-P1_d1;
%                 x_d1 = P1_d1(1);
            end
            obj.points = pts;
            obj.arclength = l;

            [~,obj.curvature] = obj.calcolaCurvatura(obj.pivot(:,1),obj.pivot(:,2),0:arc_res:1);

            % Derivative
            for t = 0:arc_res:1
                bern = 0;
                for i = 0:n-1
                    bern = bern + (factorial(n-1)/(factorial(i)*factorial(n-1-i)))*t^i*(1-t)^(n-i).*(obj.pivot(i+2,:)-obj.pivot(i+1,:));
                end
                bern = bern*n;
                l = l + pdist([prev; bern]);
                prev = bern;
                ptsDer(:,end+1) = bern;
            end
            angle = atan2(ptsDer(2,1:end-1),ptsDer(1,1:end-1));
            obj.derivative = [angle angle(end)];

%             % 2nd derivative
%             for t = 0:arc_res:1
%                 bern = 0;
%                 for i = 0:n-1
%                     bern = bern + (factorial(n-1)/(factorial(i)*factorial(n-1-i)))*t^i*(1-t)^(n-i).*(obj.pivot(i+2,:)-obj.pivot(i+1,:));
%                 end
%                 bern = bern*n;
%                 l = l + pdist([prev; bern]);
%                 prev = bern;
%                 pts2Der(:,end+1) = bern;
%             end
        end

        function [obj,curvature] = calcolaCurvatura(obj,x, y, t)
            % Calcola la curvatura di una curva di Bezier di grado generico
            % Input:
            %   - x: vettore delle coordinate x dei punti di controllo
            %   - y: vettore delle coordinate y dei punti di controllo
            %   - t: parametro di interpolazione
            % Output:
            %   - curvature: vettore delle curvature
            
            n = length(x) - 1; % Grado della curva di Bezier
            
            % Calcola i coefficienti binomiali per il calcolo delle derivate
            binomCoeff = zeros(1, n+1);
            binomCoeff(1) = 1;
            
            for k = 1:n
                binomCoeff(k+1) = binomCoeff(k) * (n - k + 1) / k;
            end
            
            % Calcola le derivate della curva di Bezier
            dx_dt = zeros(size(t));
            dy_dt = zeros(size(t));
            
            for i = 0:n
                dx_dt = dx_dt + binomCoeff(i+1) * (1-t).^(n-i) .* t.^i * x(i+1);
                dy_dt = dy_dt + binomCoeff(i+1) * (1-t).^(n-i) .* t.^i * y(i+1);
            end
            
            % Calcola la seconda derivata della curva di Bezier
            d2x_dt2 = zeros(size(t));
            d2y_dt2 = zeros(size(t));
            
            for i = 0:n-1
                d2x_dt2 = d2x_dt2 + binomCoeff(i+1) * (1-t).^(n-1-i) .* t.^i * (x(i+2) - 2*x(i+1) + x(i+1));
                d2y_dt2 = d2y_dt2 + binomCoeff(i+1) * (1-t).^(n-1-i) .* t.^i * (y(i+2) - 2*y(i+1) + y(i+1));
            end
            
            % Calcola la curvatura come il valore assoluto del rapporto tra
            % il modulo del prodotto vettoriale delle derivate
            % e il cubo del quadrato della somma dei quadrati delle derivate
            curvature = abs(dx_dt .* d2y_dt2 - dy_dt .* d2x_dt2) ./ ((dx_dt.^2 + dy_dt.^2).^(3/2));
        end

    end
end

