function [w,xx,y,Z] = ADMMdataStructures(numWMR,WMR_order,x0,HDV_indexes)
    numWMR = length(WMR_order);
    for j = 1:numWMR
        k = WMR_order(j);
        if j == 1
            w{k} = zeros(1,1); % platoon member's controls
            xx{k} = x0(:,1); % platoon member's state
            y{k} = zeros(2,1); % lagrangian multipliers
            Z{k} = zeros(2,1); % consensus vector
        else
            if  j == 2
                w{k} = zeros(1,2);
                xx{k} = x0(:,j-1:j);
                y{k} = zeros(4,1);
                Z{k} = zeros(4,1);
            else
                w{k} = zeros(1,3);
                xx{k} = x0(:,[1 j-1:j]);
                y{k} = zeros(6,1);
                Z{k} = zeros(6,1);
            end
        end
    end

    Z{WMR_order(1)} = 0;
    for j = 1:length(WMR_order)
        jj = WMR_order(j);
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
                    Z{jj} = [Z{WMR_order(1)}
                            (xx{jj}(:,end))];%+xx{WMR_order{plat}(j+1)}(:,2))/2];
                else
                    if j < length(WMR_order)
                        if ~HDV_indexes(j-1) && ~HDV_indexes(j+1) % se davanti a me e dietro NON ci sono HDV
                            Z{jj} = [Z{WMR_order(1)}
                                    (xx{WMR_order(j-1)}(:,end)+xx{jj}(:,2))/2
                                    (xx{jj}(:,end)+xx{WMR_order(j+1)}(:,2))/2];
                        else
                            if ~HDV_indexes(j-1) % se SOLO davanti c'è l'HDV
                               Z{jj} = [Z{WMR_order(1)}
                                        xx{jj}(:,2)
                                        (xx{jj}(:,end)+xx{WMR_order(j+1)}(:,2))/2];
                            else % se SOLO dietro c'è l'HDV
                                Z{jj} = [Z{WMR_order(1)}
                                        (xx{WMR_order(j-1)}(:,end)+xx{jj}(:,2))/2
                                         xx{jj}(:,end)];
                            end
                        end
                    else
                        if ~HDV_indexes(j-1) % guardo davanti
                            Z{jj} = [Z{WMR_order(1)}
                                    (xx{WMR_order(j-1)}(:,end)+xx{jj}(:,2))/2
                                    xx{jj}(:,end)];
                        else
                            Z{jj} = [Z{WMR_order(1)}
                                xx{jj}(:,2)
                                xx{jj}(:,end)];
                        end
                    end
                end
            end
        end
    end
end

