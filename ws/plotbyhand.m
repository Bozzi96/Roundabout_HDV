% % 08 09 third to second
close all
time = (1:length(xx1{1}))*Ts;
toa = [1 31 32 30];
color = [0 0.45 0.74
         0.85,0.33,0.10
         0.93,0.69,0.13
         0.49,0.18,0.56];
% close all
figure

% subplot(3,1,1)
grid on
hold on
title("Longitudinal Displacement [m]",'FontSize',20)
xlabel("Time [s]",'FontSize',20)
lgd = {};

plot((1:length(xx1{1}(1,:)))*Ts,xx1{1}(1,:),'LineWidth',3,"Color",color(1,:))
plot((31+(1:length(xx1{2}(1,31:end))))*Ts,xx1{2}(1,31:end),'LineWidth',3,"Color",color(3,:))
plot((32+(1:length(xx1{3}(1,32:end))))*Ts,xx1{3}(1,32:end),'LineWidth',3,"Color",color(4,:))
plot((33+(1:length(xx1{4}(1,33:end))))*Ts,xx1{4}(1,33:end),'LineWidth',3,"Color",color(2,:))
legend("Location","southeast","FontSize",15)

% CAV1
% yy = xx1{1}(1,1:29);
% plot((1:length(yy))*Ts,yy,'LineWidth',3,"Color",color(1,:))
% yy = [xx1{1}(1,29) xx1{2}(1,19:50)];
% plot((18+(1:length(yy)))*Ts,yy,'LineWidth',3,"Color",color(2,:))
% yy = xx1{2}(1,50:end);
% plot((49+(1:length(yy)))*Ts,yy,'LineWidth',3,"Color",color(3,:))
% 
% % CAV2
% yy = xx1{1}(1,20:end);
% plot((20+(1:length(yy)))*Ts,yy,'LineWidth',3,"Color",color(1,:))
% 
% % CAV3
% yy = xx1{3}(1,35:end);
% plot((35+(1:length(yy)))*Ts,yy,'LineWidth',3,"Color",color(4,:))
% 
% % HDV
% yy = xx1{3}(1,30:34);
% plot((30+(1:length(yy)))*Ts,yy,'LineWidth',3,"Color",color(3,:))
% yy = [xx1{3}(1,34) xx1{4}(1,35:50)];
% plot((34+(1:length(yy)))*Ts,yy,'LineWidth',3,"Color",color(3,:))
% yy = xx1{4}(1,50:end);
% plot((50+(1:length(yy)))*Ts,yy,'LineWidth',3,"Color",color(2,:))
% 
% legend("Location","southeast")
%%
figure
tlc = tiledlayout(1,2);
title(tlc,"$\alpha$ = 0.2, $\beta$ = 0.9",'interpreter','latex',"FontSize",35)

nexttile
grid on
hold on
title("Distances [m]",'FontSize',20)
xlabel("Time [s]",'FontSize',20)
axis([0 18 0 30])
lgd = {};
vehicles = cat(1,[WMRs HDVs]);

offset = 33;
idx = WMR_order_history{offset}(1);
idx2 = WMR_order_history{offset}(2);
yy = xx1{idx}(1,offset:end);
yy2 = xx1{idx2}(1,offset:end);
plot((offset+1:offset+length(yy-yy2))*Ts,yy-yy2,'LineWidth',3,'DisplayName',strcat(vehicles{idx}.ID,"-",vehicles{idx2}.ID));
idx = WMR_order_history{offset}(2);
idx2 = WMR_order_history{offset}(3);
yy = xx1{idx}(1,offset:end);
yy2 = xx1{idx2}(1,offset:end);
plot((offset+1:offset+length(yy-yy2))*Ts,yy-yy2,'LineWidth',3,'DisplayName',strcat(vehicles{idx}.ID,"-",vehicles{idx2}.ID));
idx = WMR_order_history{offset}(3);
idx2 = WMR_order_history{offset}(4);
yy = xx1{idx}(1,offset:end);
yy2 = xx1{idx2}(1,offset:end);
plot((offset+1:offset+length(yy-yy2))*Ts,yy-yy2,'LineWidth',3,'DisplayName',strcat(vehicles{idx}.ID,"-",vehicles{idx2}.ID));

%%%%% 0 - 49
% offset = 33;
% offset2 = 0;
% idx = WMR_order_history{offset}(1);
% idx2 = WMR_order_history{offset}(2);
% yy = xx1{idx}(1,offset:end);
% yy2 = xx1{idx2}(1,offset:end);
% plot((offset+1:offset+length(yy-yy2))*Ts,yy-yy2,'LineWidth',3,'DisplayName',strcat(WMRs{idx}.ID,"-",WMRs{idx2}.ID));
% 
% idx = WMR_order_history{offset}(2);
% idx2 = WMR_order_history{offset}(3);
% yy = xx1{idx}(1,offset:end);
% yy2 = xx1{idx2}(1,offset:end);
% plot((offset+1:offset+length(yy-yy2))*Ts,yy-yy2,'LineWidth',3,'DisplayName',strcat(WMRs{idx}.ID,"-",WMRs{idx2}.ID));
% 
% idx = WMR_order_history{offset}(3);
% idx2 = WMR_order_history{offset}(4);
% yy = xx1{idx}(1,offset:end);
% yy2 = xx1{idx2}(1,offset:end);
% plot((offset+1:offset+length(yy-yy2))*Ts,yy-yy2,'LineWidth',3,'DisplayName',strcat(WMRs{idx}.ID,"-",WMRs{idx2}.ID));
% 
% 
% %%%%% 30 - 34
% offset = 30;
% offset2 = 34;
% idx = WMR_order_history{offset}(2);
% idx2 = WMR_order_history{offset}(3);
% yy = xx1{idx}(1,offset:offset2);
% yy2 = xx1{idx2}(1,offset:offset2);
% plot((offset+1:offset+length(yy-yy2))*Ts,yy-yy2,'LineWidth',3,'DisplayName',strcat(WMRs{idx}.ID,"-",WMRs{idx2}.ID));
% 
% %%%%% 35 - 49
% offset = 35;
% offset2 = 50;
% idx = WMR_order_history{offset}(2);
% idx2 = WMR_order_history{offset}(3);
% yy = [xx1{WMR_order_history{offset-1}(2)}(1,offset-1) xx1{idx}(1,offset:offset2-1)];
% yy2 = [xx1{WMR_order_history{offset-1}(3)}(1,offset-1) xx1{idx2}(1,offset:offset2-1)];
% plot((offset-1+(1:length(yy-yy2)))*Ts,yy-yy2,'LineWidth',3,'DisplayName',strcat(vehicles{idx}.ID,"-",vehicles{idx2}.ID));
% 
% idx = WMR_order_history{offset}(3);
% idx2 = WMR_order_history{offset}(4);
% yy = [xx1{idx}(1,offset:offset2-1)];
% yy2 = [xx1{idx2}(1,offset:offset2-1)];
% plot((offset+(1:length(yy-yy2)))*Ts,yy-yy2,'LineWidth',3,'DisplayName',strcat(vehicles{idx}.ID,"-",vehicles{idx2}.ID));
% 
% %%%%% 50 - end
% offset = 50;
% idx = WMR_order_history{offset}(1);
% idx2 = WMR_order_history{offset}(2);
% yy = [xx1{WMR_order_history{offset-1}(1)}(1,offset-1) xx1{idx}(1,offset:end)];
% yy2 = [xx1{WMR_order_history{offset-1}(2)}(1,offset-1) xx1{idx2}(1,offset:end)];
% plot((offset-1+(1:length(yy-yy2)))*Ts,yy-yy2,'LineWidth',3,'DisplayName',strcat(vehicles{idx}.ID,"-",vehicles{idx2}.ID));
% 
% idx = WMR_order_history{offset}(2);
% idx2 = WMR_order_history{offset}(3);
% yy = [xx1{WMR_order_history{offset-1}(2)}(1,offset-1) xx1{idx}(1,offset:end)];
% yy2 = [xx1{WMR_order_history{offset-1}(3)}(1,offset-1) xx1{idx2}(1,offset:end)];
% plot((offset-1+(1:length(yy-yy2)))*Ts,yy-yy2,'LineWidth',3,'DisplayName',strcat(vehicles{idx}.ID,"-",vehicles{idx2}.ID));
% 
% idx = WMR_order_history{offset}(3);
% idx2 = WMR_order_history{offset}(4);
% yy = [xx1{WMR_order_history{offset-1}(3)}(1,offset-1) xx1{idx}(1,offset:end)];
% yy2 = [xx1{WMR_order_history{offset-1}(4)}(1,offset-1) xx1{idx2}(1,offset:end)];
% plot((offset-1+(1:length(yy-yy2)))*Ts,yy-yy2,'LineWidth',3,'DisplayName',strcat(vehicles{idx}.ID,"-",vehicles{idx2}.ID));


plot(time, dmin*ones(1,length(time)),'b--','DisplayName','Bounds','LineWidth',3);
% plot(time, dmax*ones(1,length(time)),'b--',time, dmin*ones(1,length(time)),'b--','DisplayName','Bounds','LineWidth',3);
plot(time, ddes*ones(1,length(time)),'r--','DisplayName','Reference','LineWidth',3);
legend("Location","southeast","FontSize",15,"NumColumns",3)

% figure
nexttile
grid on
hold on
title("Velocity [m/s]",'FontSize',20)
xlabel("Time [s]",'FontSize',20)

toa = [1 35 20 30];
vehicles = cat(1,[WMRs HDVs]);
for i = 1:length(vehicles)
    yy = vehicles{i}.speedHistory;
    plot((toa(i)+1:toa(i)+length(yy))*Ts,yy,'LineWidth',3,'DisplayName',vehicles{i}.ID);
end
yline(vref,'--r','DisplayName','Reference','LineWidth',3)
legend(lgd,"Location","southeast","FontSize",15);