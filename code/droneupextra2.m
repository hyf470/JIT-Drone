%ドローンが目的姿勢角まで傾く

function du = droneupextra2(t,u,k1,k2,k3,k4,k5,zd,pd,qd,rd,p1,q1,t1)

g = 9.8;

du = zeros(12,1);

pd = -p1*sin(2*pi/t1*t);%ローの目標姿勢角
qd = q1*sin(2*pi/t1*t);%ピッチの目標姿勢角




i1 = (1/(cos(u(7))*(cos(u(9)))))*(g-(k1*(u(5)-zd)+k2*u(6)));
i2 = -0.0591*(u(7)-pd)-k3*u(8);
i3 = -0.0591*(u(9)-qd)-k4*u(10);
i4 = -0.094*(u(11)-rd)-k5*u(12);
 

du(1) = u(2);
du(2) = (cos(u(7))*sin(u(9))*cos(u(11))+sin(u(9))*sin(u(11)))*4*i1;
du(3) = u(4);
du(4) = (cos(u(7))*sin(u(9))*sin(u(11))-sin(u(7))*cos(u(11)))*4*i1;
du(5) = u(6);
du(6) = (cos(u(7))*cos(u(9)))*i1-g; %%i1*4
du(7) = u(8);
du(8) = u(10)*u(12)*(-0.589)+(68.2)*i2;
du(9) = u(10);
du(10) = u(8)*u(12)*(0.589)+(68.2)*i3;
du(11) = u(12);
du(12) = (42.9)*i4;