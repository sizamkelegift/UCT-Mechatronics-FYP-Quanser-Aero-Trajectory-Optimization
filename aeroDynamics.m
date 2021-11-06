function dz = aeroDynamics(z,u,p)

q1 = z(1,:);
q2 = z(2,:);
dq1 = z(3,:);
dq2 = z(4,:);

nTime = length(q1);
ddq = zeros(2,nTime);
for i=1:nTime
    ddq(1,i) = (5*u(1,i))/73 +(20*u(2,i))/219 - (125*q1(i))/73 - (226*dq1(i))/219;
    ddq(2,i) = 17*u(1,i)/220+(2*u(2,i))/11 - (211*dq2(i))/220;
end
dz = [dq1;dq2;ddq];
end