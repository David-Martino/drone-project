data = readtable("v_pid.csv")

t = 0.01:0.01:0.01*height(data)


% % z
% subplot(2,3,1);
% hold on
% grid on
% plot(t, data.vz)
% plot(t, data.vzctrl)
% legend('vz','control')
% 
% theme("light")
% 
% subplot(2,3,4); 
% plot(t,data.z)

% x
subplot(2,2,1);
hold on
grid on
plot(t, data.vx)
plot(t, data.vxctrl)
legend('vx','control')

theme("light")

subplot(2,2,3); 
plot(t,data.x)

% y
subplot(2,2,2);
hold on
grid on
plot(t, data.vy)
plot(t, data.vyctrl)
legend('vy','control')

theme("light")

subplot(2,2,4); 
plot(t,data.y)




