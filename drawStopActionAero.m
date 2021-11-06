function drawStopActionAero(t,p1,p2,nFrame)

clf; hold on;
xlabel('x position[m]')
ylabel('y position[m]')
zlabel('z position[m]')
Dt = 0.158;
Dm = 0.35;
%Pole_Width = 0.02;  %pixels
%width = 0.05;

%%%% Figure out the window size:

[xLow, xUpp, yLow, yUpp, zLow, zUpp] = getBounds(p1,p2);

xLow = xLow - 0.7*Dt;
xUpp = xUpp + 0.7*Dt;

yLow = yLow - 0.7*Dt;
yUpp = yUpp + 0.7*Dt;

zLow = 0;
zUpp = zUpp + 0.7*Dm;

Limits = [xLow,xUpp,yLow,yUpp,zLow,zUpp];

%%%% Get color map for the figure
map = colormap;
tMap = linspace(t(1),t(end),size(map,1))';

%%%% Plot Rails
%plot([Limits(1) Limits(2)],-0.5*Cart_Height*[1,1],'k-','LineWidth',2)

%%%% Draw the trace of the pendulum tip  (continuously vary color)

nTime = length(t);
for i=1:(nTime-1)
    idx = i:(i+1);
    x = p2(1,idx);
    y = p2(2,idx);
    Z = p2(3,idx);
    c = interp1(tMap,map,mean(t(idx)));
    plot3(x,y,Z,'Color',c,'LineWidth',2);
end

for i=1:(nTime-1)
    idx = i:(i+1);
    x = p1(1,idx);
    y = p1(2,idx);
    Z = p1(3,idx);
    c = interp1(tMap,map,mean(t(idx)));
    plot3(x,y,Z,'Color',c ,'LineWidth',2);
end

%%%% Compute the frames for plotting:
tFrame = linspace(t(1), t(end), nFrame);
back = interp1(t',p1',tFrame')';
front = interp1(t',p2',tFrame')';
pos=[zeros(length(back),1),zeros(length(back),1), ones(length(back),1).*Dm];

for i = 1:nFrame
    
    % Compute color:
    color = interp1(tMap,map,tFrame(i));
    
    %{
    Plot Cart
    x = back(1,i) - 0.5*Cart_Width;
    y = -0.5*Cart_Height;
    w = width;
    h = width;
    hCart = rectangle('Position',[0,0,0,w,h],'LineWidth',2);
    set(hCart,'FaceColor',color);
    set(hCart,'EdgeColor',0.8*color);
    %}
    %{
    %Plot 
    Rod_X = [back(1,i), front(1,i)];
    Rod_Y = [back(2,i), front(2,i)];
    Rod_Z = [back(3,i), front(3,i)];
    plot3(Rod_X,Rod_Y,Rod_Z,'k-','LineWidth',Pole_Width,'Color',color)
    plot3([0,0],[0,0],[0,Dm],'k-','LineWidth',Pole_Width,'Color','blue')
    %}
    %{
    %front
    Rod_X = [pos(i,1), front(1,i)];
    Rod_Y = [pos(i,2), front(2,i)];
    Rod_Z = [pos(i,3), front(3,i)];
    plot3(Rod_X,Rod_Y,Rod_Z,'k-','LineWidth',Pole_Width,'Color',color)
    %}
    %{
    Rod_X = [pos(i,1), back(1,i)];
    Rod_Y = [pos(i,2), back(2,i)];
    Rod_Z = [pos(i,3), back(3,i)];
    plot3(Rod_X,Rod_Y,Rod_Z,'k-','LineWidth',Pole_Width,'Color',color)
    plot3([0,0],[0,0],[0,Dm],'k-','LineWidth',Pole_Width,'Color','blue')
    %}
    
    %Plot Bob and hinge
    %plot3(front(1,i),front(2,i),front(3,i),'k.','MarkerSize',10,'Color',color)
    %plot3(back(1,i),back(2,i), back(3,i),'k.','MarkerSize',10,'Color',color)
    
end

%These commands keep the window from automatically rescaling in funny ways.
axis(Limits);
view([45,45])
grid on;
%%axis off;

end



function [xLow, xUpp, yLow, yUpp, zLow, zUpp] = getBounds(p1,p2)
%
% Returns the upper and lower bound on the data in val
%

val = [p1,p2];
xLow = min(val(1,:));
xUpp = max(val(1,:));
yLow = min(val(2,:));
yUpp = max(val(2,:));
zLow = min(val(3,:));
zUpp = max(val(3,:));

end