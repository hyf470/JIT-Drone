
%%%空中で滞空するドローンを上昇させ，同時に機体を傾ける制御(y-z飛行)のデータベース%%%

%固定するパラメータ%
h = 0.01; % 時間のサンプリング間隔
k1 = 0.066; %z軸方向の偏差に関わるゲイン
k2 = 0.45; %z軸方向の速度に関わるゲイン
k3 = 0.08; %x軸周りの角速度に関わるゲイン　元は0.015 前回0.15
k4 = 0.08; %y軸周りの角速度に関わるゲイン  元は0.015
k5 = 0.09; %z軸周りの角速度に関わるゲイン
zd = 7.3; %z軸の目標値[m]
pd = 0; %x軸周りの角度の目標値[rad]
qd = 0; %y軸周りの角度の目標値[rad]
rd = 0; %z軸周りの角度の目標値[rad]
t1 = 6; %姿勢制御時間[s](3)


%JITで変化させるパラメータ% 
p1min = -pi/40; %x軸まわりの姿勢角の傾きの最小値 (pi/150) 前回pi/50
p1h = pi/5000; %x軸まわりの姿勢角の傾きの刻み幅(10000) 前回pi/5000
p1max = pi/40; %x軸まわりの姿勢角の傾きの最大値 (pi/50)

q1min = -pi/40; %y軸まわりの姿勢角の傾きの最小値
q1h = pi/5000; %y軸まわりの姿勢角の傾きの刻み幅
q1max = pi/40; %y軸まわりの姿勢角の傾きの最大値

a = ((p1max-p1min+p1h)+(q1max-q1min+q1h))/p1h;
b = round(a);

%%% データベースの作成 %%%
data = zeros(b,4);
n = 1;

for q1 = q1min:q1h:q1max
for p1 = p1min:p1h:p1max


[T1, U1] = ode45(@droneupextra2,(0:h:3),[0;0;0;0;0;0;0;0;0;0;0;0],[],k1,k2,k3,k4,k5,zd,pd,qd,rd,p1,q1,t1);
U0 = U1(301,:);

[T2, U2] =  ode45(@droneupextrabackl,(3:h:40),U0,[],k1,k2,k3,k4,k5,zd,pd,qd,rd);

T = [T1;T2]; % 時間変数を結合
U = [U1;U2]; % 状態変数を結合

data(n,1) = U(4002,1); %制御終了後の機体重心のx座標
data(n,2) = U(4002,3); %制御終了後の機体重心のy座標
data(n,3) = q1; %最大姿勢角ピッチ　ここの順番はピッチ-x座標，ロー-y座標と対応しているため
data(n,4) = p1; %最大姿勢ロー
n = n + 1;

end
end

%%% 後処理 %%%
data = data(1:n-1,:); % data変数の切り出し(余分な箇所をカットする)
csvwrite('data_fly_xyz.csv', data); % dataファイルをCSVファイルとして出力
n = n - 1; % nは1個余分にカウントしているので引いておく
