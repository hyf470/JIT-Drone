%風外乱を計算する
filename = '../database/data_U.csv';
U = csvread(filename);
L=length(U);
a=0.35;%ドローンの縦
b=0.35;%ドローンの横
c=0.1;%ドローンの高さ
A1=b*c;%受風面積（x）
A2=a*c;%受風面積（y）
A3=a*b;%受風面積（z）
A=[A1;A2;A3];
m=0.4;%ドローンの重量（kg）
cd1=0.4;%ドローン局所座標の抗力係数（x）
cd2=0.4;%ドローン局所座標の抗力係数（y）
cd3=1.2;%ドローン局所座標の抗力係数（z）
cd=[cd1;cd2;cd3];
u1 = 1.5;%風速「m/s」
p=1.166;%空気密度
Dxyz=[];
for i = 1:L
    x=U(i,7);%x軸周りの姿勢角
    y=U(i,9);%y軸周りの姿勢角
    vx=U(i,2);%x速度
    vy=U(i,4);%y速度
    vz=U(i,6);%z速度
    Rx=[1 0 0;0 cos(x) -sin(x);0 sin(x) cos(x)];%x方向の座標変換行列
    Ry=[cos(y) 0 sin(y);0 1 0;-sin(y) 0 cos(y)];%y方向の座標変換行列
    Rz=[1 0 0;0 1 0;0 0 1];%z方向の座標変換行列
    R=Rx*Ry*Rz;%局所座標から全体座標へ変換行列
    vxyz=[U(i,2);U(i,4);U(i,6)];
    v123=inv(R)*vxyz;%相对坐标 = 旋转矩阵^T * (绝对坐标 - 平移向量)
    u123=[u1;0;0];%風速
    F=(p.*cd.*((v123-u123)).^2.*A)./2;%風外乱力を計算
    d123=F./m;%局所座標で風外乱
    dxyz=R*d123;%全体座標で風外乱
    dxyz=dxyz';
    Dxyz=[Dxyz;dxyz];
end
%%
%%% 後処理 %%%
csvwrite('data_dxyz.csv', Dxyz); % dataファイルをCSVファイルとして出力


