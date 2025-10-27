data = readtable("pitch_pid.csv")

t = 0.01:0.01:0.01*height(data)

fig1 = figure()
hold on
grid on
plot(t, data.pitch)
plot(t, data.cmd)
legend('pitch','cmd')

theme("light")

fig2 = figure()
hold on
grid on
plot(t, data.vx)
plot(t, data.vxctrl)
legend('vx','vxctrl')

theme("light")





