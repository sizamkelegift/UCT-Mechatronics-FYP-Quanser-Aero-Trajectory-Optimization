function plotAero(t,z,u)

subplot(3,2,1);
plot(t,z(1,:))
xlabel('t')
ylabel('q1')
title('pitch angle')

subplot(3,2,3);
plot(t,z(2,:))
xlabel('t')
ylabel('q2')
title('yaw angle')

subplot(3,2,2);
plot(t,z(3,:))
xlabel('t')
ylabel('dq1')
title('pitch angle rate')

subplot(3,2,4);
plot(t,z(4,:))
xlabel('t')
ylabel('dq2')
title('yaw angle rate')

subplot(3,2,5);
plot(t,u)
ylabel('u')
xlabel('t')
title('input voltage')
end