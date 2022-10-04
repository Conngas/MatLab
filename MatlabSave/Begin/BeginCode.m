x = 0:0.01:20;
y1 = sin(2*x);
y2 = log(x);

figure
[AX,H1,H2] = plotyy(x,y1,x,y2,'plot');
set(get(AX(1),'Ylabel'),'String','Slow Decay')
set(get(AX(2),'Ylabel'),'String','Fast Decay')
xlabel('Time(\sec)')
title('Dev Rate')
set(H1,'LineStyle','--')
set(H2,'LineStyle',':')

x = 0:0.01:20;
figure
plot3(log(x),sin(x),exp(x))
xlabel('log(x)')
ylabel('sin(x)')
zlabel('e(x)')
grid on
