%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 風外乱データベース
%
% 入力：ドローンの速度vx,vy,vz;風速ux,uy,uz；
% 出力：風によりドローンに対して外乱dx,dy,dz;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%固定するパラメータ%%
h = 0.5; %サンプリング間隔
a=0.35;%ドローンの縦
b=0.35;%ドローンの横
c=0.1;%ドローンの高さ
p=1.166;%空気密度
cd1=0.4;%ドローン局所座標の抗力係数（x）
cd2=0.4;%ドローン局所座標の抗力係数（y）
cd3=1.2;%ドローン局所座標の抗力係数（z）
cd=[cd1;cd2;cd3];
g=9.8;
A1=b*c;%受風面積（x）
A2=a*c;%受風面積（y）
A3=a*b;%受風面積（z）
A=[A1;A2;A3];
m=0.4;%ドローンの重量（kg）

%% 
%%変化するパラメータ%%
u1min=-2;%局所座標x方向で最小風速
uh=0.1;%刻み幅
u1max=2;%局所座標x方向で最大風速
%u2min=-10;%局所座標y方向で最小風速
%u2max=10;%局所座標y方向で最大風速
%u3min=-10;%局所座標z方向で最小風速
%u3max=10;%局所座標z方向で最大風速
vxmin=0;%x方向で最小速度
vh=0.05;%刻み幅
vxmax=0.8;%x方向で最大速度
vymin=0;%y方向で最小速度
vymax=1.9;%y方向で最大速度
vzmin=0;%z方向で最小速度
vzmax=0.5;%z方向で最大速度
p1min = -pi/125; %x軸まわりの姿勢角の傾きの最小値 (pi/150) 前回pi/50
p1h = pi/1000; %x軸まわりの姿勢角の傾きの刻み幅(10000) 前回pi/5000
p1max = pi/125; %x軸まわりの姿勢角の傾きの最大値 (pi/50)
q1min = -pi/125; %y軸まわりの姿勢角の傾きの最小値
q1h = pi/1000;%y軸まわりの姿勢角の傾きの刻み幅
q1max = pi/125; %y軸まわりの姿勢角の傾きの最大値
a = ((u1max-u1min+uh)/uh)*((vxmax-vxmin+vh)/vh)*((vymax-vymin+vh)/vh)*((vzmax-vzmin+vh)/vh)*((p1max-p1min+p1h)/p1h)*((q1max-q1min+q1h)/q1h);
%a = ((u1max-u1min+uh)/uh)*((vxmax-vxmin+vh)/vh)*((vymax-vymin+vh)/vh)*((vzmax-vzmin+vh)/vh);
b = round(a);
%% 
%%データベースの作成%%
data = zeros(b,9);
%data = zeros(b,7);
n=1;
for p1 = p1min:p1h:p1max
for q1 = q1min:q1h:q1max
for u1 = u1min:uh:u1max
%for u2 = u2min:uh:u2max
%for u3 = u3min:uh:u3max
for vx = vxmin:vh:vxmax
for vy = vymin:vh:vymax
for vz = vzmin:vh:vzmax
    Rx=[1 0 0;0 cos(p1) -sin(p1);0 sin(p1) cos(p1)];%x方向の座標変換行列
    Ry=[cos(q1) 0 sin(q1);0 1 0;-sin(q1) 0 cos(q1)];%y方向の座標変換行列
    Rz=[1 0 0;0 1 0;0 0 1];%z方向の座標変換行列
    % Rx=[1 0 0;0 1 0;0 0 1];%x方向の座標変換行列
    % Ry=[1 0 0;0 1 0;0 0 1];%y方向の座標変換行列
    % Rz=[1 0 0;0 1 0;0 0 1];%z方向の座標変換行列
    R=Rx*Ry*Rz;%局所座標から全体座標へ変換行列
    vxyz=[vx;vy;vz];
    v123=inv(R)*vxyz;%相对坐标 = 旋转矩阵^T * (绝对坐标 - 平移向量)
    u123=[u1;0;0];
    F=(p.*cd.*((v123-u123)).^2.*A)./2;%風により外乱力
    d123=F./m;%局所座標で風外乱
    dxyz=R*d123;%全体座標で風外乱
    data(n,1)=vx;
    data(n,2)=vy;
    data(n,3)=vz;
    data(n,4)=u1;
    data(n,5)=p1;
    data(n,6)=q1;
    %data(n,5)=u2;
    %data(n,6)=u3;
    data(n,7)=dxyz(1);
    data(n,8)=dxyz(2);
    data(n,9)=dxyz(3);
    n=n+1;
end
end
end
end
end
end
%%
%%% 後処理 %%%
data = data(1:n-1,:); % data変数の切り出し(余分な箇所をカットする)
csvwrite('data_wind_xyz.csv', data); % dataファイルをCSVファイルとして出力
n = n - 1; % nは1個余分にカウントしているので引いておく

