%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%空中で滞空するドローンを上昇させ，同時に機体を傾ける制御(xyz飛行)
%1回ソートのJIT
%
%　入力値 座標の目標値 xd,yd
%　出力値 x軸周りの操舵角 p ,y軸周りの操舵角 q 最新10/13
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%固定するパラメータ%
h = 0.01; % 時間のサンプリング間隔
k3 = 0.08; %x軸周りの角速度に関わるゲイン　
k4 = 0.08; %y軸周りの角速度に関わるゲイン
k5 = 0.09; %z軸周りの角速度に関わるゲイン
pd = 0; %x軸周りの角度の目標値[rad]
qd = 0; %y軸周りの角度の目標値[rad]
rd = 0; %z軸周りの角度の目標値[rad]
t1 = 6; %姿勢制御時間[s]
tup = 10; %目標高度到達時刻[s]

%%% JITで計算する要求データ %%%
xo = 0; %x座標の初期値
yo = 0; %y座標の初期値
zo = 0; %z座標の初期値
xd = 1.21; %x座標の目標値(-5.0~5.0)[m]
yd = 3.11; %y座標の目標値(-5.0~5.0)[m]
zd = 2.45; %目標高度(0~10.0)[m]

rx = xd - xo;%相対値x[m]
ry = yd - yo;%相対値y[m]
rzd = zd - zo; %相対高度[m]

%%% dataファイルの読み込み(高度制御のデータ) %%%
filename = 'data_drone_3.csv';
d = csvread(filename);
n = length(d);

%%% 要求データとデータベースの距離を計算 %%%
for i=1:n
    d(i,5) = sqrt((rzd-d(i,1))^2+(tup-d(i,4))^2);
end

%%% 距離が小さい順に行をソートする %%%
dz = sortrows(d,5);

%%% JITのアルゴリズム：距離の逆数を重みとした平均 %%%
nn = round(n/100); % 近傍データの数
K1 = 0; % 求めるゲインk1の値の初期化
K2 = 0; % 求めるゲインk2の値の初期化
DZ = 0; % 距離の逆数の総和の初期化
if dz(1,5) == 0 
    K1 = dz(1,2); % もし要求データがデータベースに一致すればそのデータを使う
    K2 = dz(1,3);
else
    for i=1:40
        K1 = K1 + dz(i,2)/dz(i,5);
        K2 = K2 + dz(i,3)/dz(i,5);
        DZ = DZ + 1/dz(i,5);
    end
    K1 = K1/DZ; % JITで求められたゲインk1
    K2 = K2/DZ; % JITで求められたゲインk2 
end

%%% dataファイルの読み込み(姿勢制御のデータ) %%%
filename = 'data_fly_xyz.csv';
d2 = csvread(filename);
n2 = length(d2);

 %%% 要求データとデータベースの距離を計算 %%%
for i=1:n2
   d2(i,5) = (rx - (d2(i,1)))^2;
end

 %%% ユークリッド距離が小さい順に行をソートする(1回目のソート) %%%
 dd1 = sortrows(d2,5);

 %%% dd1内の上位　組のデータを抽出 %%%
 dd2 = zeros(250,4);

for i = 1:250
        dd2(i,1) = dd1(i,1);
        dd2(i,2) = dd1(i,2);
        dd2(i,3) = dd1(i,3);
        dd2(i,4) = dd1(i,4);
end

%%% 姿勢角が小さい順に行をソートする(2回目のソート) %%%
 for i=1:250
        dd2(i,5) = sqrt((rx-dd2(i,1))^2 + (ry-dd2(i,2))^2);
 end
        dd = sortrows(dd2,5);
    

 %%% JITのアルゴリズム：距離の逆数を重みとした平均 %%%
 nn = round(n2/100); % 近傍データの数
 P = 0; % 求める最大姿勢角p1の値の初期化
 Q = 0; % 求める最大姿勢角p1の値の初期化
 DD = 0; % 距離の逆数の総和の初期化
 if dd(1,5) == 0 
   P = dd(1,4); % もし要求データがデータベースに一致すればそのデータを使う
   Q = dd(1,3); % もし要求データがデータベースに一致すればそのデータを使う
 else
 for i=1:30
   P = P + dd(i,4)/dd(i,5)^2;
   Q = Q + dd(i,3)/dd(i,5)^2;
   DD = DD + 1/dd(i,5)^2;
 end
   P = P/DD; % JITで求めた目標姿勢角
   Q = Q/DD; % JITで求めた目標姿勢角
 end
 
%%%微分方程式を解く%%%
[T1, U1] = ode45(@droneupextra2,(0:h:3),[xo;0;yo;0;zo;0;0;0;0;0;0;0],[],K1,K2,k3,k4,k5,zd,pd,qd,rd,P,Q,t1);
U0 = U1(301,:);

[T2, U2] =  ode45(@droneupextrabackl,(3:h:20),U0,[],K1,K2,k3,k4,k5,zd,pd,qd,rd);

T = [T1;T2]; % 時間変数を結合
U = [U1;U2]; % 状態変数を結合

xo = U(2002,1); 
yo = U(2002,3); 
zo = U(2002,5); 


% Side lengths
%l = 0.248; %ドローンの足の長さ
l = 0.6; %ドローンの足の長テスト用
%l = 0.74; %ドローンの足の長テスト用
Lz = 0.05;

% Vertices
vertices = [ 0 -l  0;
             l  0  0;
            -l  0  0;
             0 -l Lz;
             0  l  0;
            -l  0 Lz;
             l  0 Lz;
             0  l Lz];

% Faces
faces = [
    1, 2, 5, 3;  % #1
    1, 3, 6, 4;  % #2
    1, 4, 7, 2;  % #3
    4, 7, 8, 6;  % #4
    2, 5, 8, 7;  % #5
    3, 6, 8, 5]; % #6

%aviファイルに保存
set(gca,'nextplot','replacechildren');
v = VideoWriter('simulation1.avi');
open(v);


% Draw initial figure 
figure(1);
h = patch('Faces', faces, 'Vertices', vertices, 'FaceColor', 'g','FaceAlpha',.3,'EdgeAlpha',.3);
xlabel('x'); ylabel('y'); zlabel('z');
%axis vis3d equal;
axis tight;
%view([90, 0]); %視点真横
view([-60, 28]);
camlight;
grid on;
xlim([-1, 5]);
ylim([-1, 5]);
zlim([0, 5]);

xlabel('x[m]')
ylabel('y[m]')
zlabel('z[m]')
grid on
grid minor


% Animation loop 
for i = 2:5:1001 %5

% Vertices option
R = [cos(U(i,11))*cos(U(i,9)) -sin(U(i,11))*cos(U(i,7))+cos(U(i,11))*sin(U(i,7))*sin(U(i,7)) sin(U(i,11))*sin(U(i,9))+cos(U(i,11))*sin(U(i,9))*cos(U(i,7));
     sin(U(i,11))*cos(U(i,9)) cos(U(i,11))*cos(U(i,7))+sin(U(i,11))*sin(U(i,9))*sin(U(i,7)) sin(U(i,11))*cos(U(i,9))*cos(U(i,7))-cos(U(i,11))*sin(U(i,7));
     -sin(U(i,9)) cos(U(i,9))*sin(U(i,7)) cos(U(i,9))*cos(U(i,7))];

P1 = R*[U(i,1); U(i,3)-l; U(i,5)];
P2 = R*[U(i,1)+l; U(i,3); U(i,5)];
P3 = R*[U(i,1)-l; U(i,3); U(i,5)];
P4 = R*[U(i,1); U(i,3)-l; U(i,5)+Lz];
P5 = R*[U(i,1); U(i,3)+l; U(i,5)];
P6 = R*[U(i,1)-l; U(i,3); U(i,5)+Lz];
P7 = R*[U(i,1)+l; U(i,3); U(i,5)+Lz];
P8 = R*[U(i,1); U(i,3)+l; U(i,5)+Lz];

vertices = [P1(1) P1(2) P1(3);
            P2(1) P2(2) P2(3);
            P3(1) P3(2) P3(3);
            P4(1) P4(2) P4(3);
            P5(1) P5(2) P5(3);
            P6(1) P6(2) P6(3);
            P7(1) P7(2) P7(3);
            P8(1) P8(2) P8(3)];

faces = [
    1, 2, 5, 3;  % #1
    1, 3, 6, 4;  % #2
    1, 4, 7, 2;  % #3
    4, 7, 8, 6;  % #4
    2, 5, 8, 7;  % #5
    3, 6, 8, 5]; % #6

%%%h = patch('Faces', faces, 'Vertices', vertices, 'FaceColor', 'y');
 
     set(h, 'Vertices', vertices);
     
     hold on;
     plot3(xd,yd,zd,'r*');
     plot3(U(i,1),U(i,3),U(i,5),'ob');
     hold off;  

    drawnow;
    
    %更新して保存
    frame = getframe(gcf);
    writeVideo(v,frame);
   
end



xlim([-1 5])
ylim([-1 5])
zlim([0 5])
xlabel('x[m]')
ylabel('y[m]')
zlabel('z[m]')
grid on
grid minor