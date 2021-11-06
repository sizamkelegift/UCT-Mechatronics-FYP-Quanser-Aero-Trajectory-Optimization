function drawAero(t,z,p)

clf; hold on;

writerObj = VideoWriter('myVideo.avi');
writerObj.FrameRate = 10;
open(writerObj);

psi= z(2,:);
tht= z(1,:);

Dt = 0.158;
Dm = 0.35;

% Positions
%tempAlpha = alpha.signals.values; % uncomment if you simulated in Simulink

pos1=[zeros(length(psi),1),zeros(length(psi),1), ones(length(psi),1).*Dm];
pos2=[Dt*cos(tht).*cos(psi),Dt*cos(tht).*sin(psi),Dm + Dm*sin(tht)];
pos3=[-Dt*cos(tht).*cos(psi),-Dt*cos(tht).*sin(psi),Dm - Dm*sin(tht)];

%position of centre of mass
mass1=[((cos(tht).*sin(psi)).*Dt/2), (cos(tht).*cos(psi).*Dt/2),(Dt/2).*(sin(tht))+ones(length(psi),1)*Dm];
mass2=[-((cos(tht).*sin(psi)).*Dt/2), (cos(tht).*cos(psi).*Dt/2),(Dt/2).*(sin(tht))+ones(length(psi),1)*Dm];

%Create Figure Handles

h1 = line('Color', 'b', 'LineWidth', 4);
h2 = line('Color','r','LineWidth',4);
h3 = line('Color','r','LineWidth',4);

FigHandle = gcf;

axis([0 2 0 2])
view(45, 45)
grid on
hold on;

% Links

% Link plots
p1_0 = plot3(mass1(1,1),mass1(1,2),mass1(1,3),'o','MarkerFaceColor','green','MarkerSize',12);
p2_0 = plot3(mass2(1,1),mass2(1,2),mass1(1,3),'o','MarkerFaceColor','red','MarkerSize',12);

% general plot setup
xlabel({'X Position (m)'},'FontSize',14,'FontName','AvantGarde');
ylabel({'Y Position (m)'},'FontSize',14,'FontName','AvantGarde');
zlabel({'Z Position (m)'},'FontSize',14,'FontName','AvantGarde');

xlim([-0.5,0.5]);
ylim([-0.5,0.5]);
zlim([0,0.9]);


% Update the rod and masses in the plot in a loop
for i = 1:length(psi)
    plot3(pos3(i,1),pos3(i,2),pos3(i,3),'b-','LineWidth',2)
    % Rod 1
    set(h1, 'XData', [0, pos1(i,1)]);
    set(h1, 'YData', [0, pos1(i,2)]);
    set(h1, 'ZData', [0, pos1(i,3)]);
    
    %Rod 2
    set(h2, 'XData', [pos1(i,1), pos2(i,1)]);
    set(h2, 'YData', [pos1(i,2), pos2(i,2)]);
    set(h2, 'ZData', [pos1(i,3), pos2(i,3)]);
    
    %Rod 3
    set(h3, 'XData', [pos1(i,1), pos3(i,1)]);
    set(h3, 'YData', [pos1(i,2), pos3(i,2)]);
    set(h3, 'ZData', [pos1(i,3), pos3(i,3)]);
    
    % Link positions
    set(p1_0, 'XData', pos2(i,1));
    set(p1_0, 'YData', pos2(i,2));
    set(p1_0, 'ZData', pos2(i,3));
    set(p2_0, 'XData', pos3(i,1));
    set(p2_0, 'YData', pos3(i,2));
    set(p2_0, 'ZData', pos3(i,3));
    hold on;
    
    plot3(pos3(i,1),pos3(i,2),pos3(i,3),'b-','LineWidth',2)
    
    %quiver3(pos3(i,1),pos3(i,2),pos3(i,3),0.4*pos3(i,1),0.45*pos3(i,2),0.4*pos3(i,3))
    
    pause(0.1);  % the time per loop is (calculation/render time) + (pause)
                 % this doesn't need to be done properly -- we'll work on
                 % that in part 2 of this prac
    F(i) = getframe(gcf) ;
    writeVideo(writerObj, F(i));
    drawnow 
   
    xlim('manual')
    ylim('manual')
end
title(sprintf('Quanser Aero Animation,  t = %6.4f', t));

close(writerObj);

end