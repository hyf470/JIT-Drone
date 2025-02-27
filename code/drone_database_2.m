%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% JITモデリングによるドローンの浮上の安定制御のデータベース(細かく)
%
% 入力：高さの目標値 zd
% 出力：制御入力のゲイン k1, k2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% 固定するパラメータ %%%
h = 0.5; %　サンプリング間隔
t = 10 ; %　浮上に適切な時間を算出するため変数
k3 = 0.08; %x軸周りの角速度に関わるゲイン　
k4 = 0.08; %y軸周りの角速度に関わるゲイン
k5 = 0.09; %z軸周りの角速度に関わるゲイン
pd = 0; %x軸周りの角度の目標値[rad]
qd = 0; %y軸周りの角度の目標値[rad]
rd = 0; %z軸周りの角度の目標値[rad]

%%% JITで変化させるパラメータ %%%
k1min = 0.08; %z軸方向の偏差に関わるゲインの最小値(0.02)
k1h = 0.005; %z軸方向の偏差に関わるゲインの刻み幅(0.001)
k1max = 0.6; %z軸方向の偏差に関わるゲインの最大値(0.4)

k2min = 0.5; %z軸方向の速度に関わるゲインの最小値(0.2)
k2h = 0.001; %z軸方向の速度に関わるゲインの刻み幅(0.005)
k2max = 1.2; %z軸方向の速度に関わるゲインの最大値(1.3)

zdmin = -5; %目標高度の最小値
zdh = 0.5; %目標高度の刻み幅
zdmax = 5; %目標高度の最大値

a = ((k1max-k1min+k1h)/k1h)*((k2max-k2min+k2h)/k2h)*((zdmax-zdmin+zdh)/zdh);
b = round(a);

%%% データベースの作成 %%%
data = zeros(b,4);
n = 1;
for zd = zdmin:zdh:zdmax
for k1 = k1min:k1h:k1max
for k2 = k2min:k2h:k2max

   rzd = 10 + zd; 

[T, U] = ode45(@droneupextrabackl,(0:0.2:100),[0;0;0;0;10;0;0;0;0;0;0;0],[],k1,k2,k3,k4,k5,rzd,pd,qd,rd);

%%% オーバーシュートと上昇時間の判定
zc = zeros(501,1); % 行き過ぎ量を格納する変数
C = zeros(501,1); % オーバーシュート判定定数(C=0はオーバーシュートなし,C=1はオーバーシュートあり)
D = zeros(501,1); % 上昇時刻判定定数(C=0は未上昇,C=1は上昇達成)
CC = 0; % オーバーシュートフラグ定数
CD = 0; % 上昇時刻フラグ定数



if rzd > 10
    for i=1:501
        zc(i) = (U(i,5)/rzd)-1;
        if zc(i) < 0.001
            C(i) = 0;
            CC = 0;
        else
            C(i) = 1;
            CC = 1;
            break;
        end
    end
    for i=1:501
        if zc(i) < -0.001
            C(i) = 0;
            CD = 0;
        else    
            tup = (i-1)*0.2;
            D(i) = 1;
            CD = 1;
        break;
        end
    end
else    
    for i=1:501
        zc(i) = (U(i,5)/rzd)-1;
        if zc(i) > -0.001
            C(i) = 0;
            CC = 0;
        else
            C(i) = 1;
            CC = 1;
            break;
        end
    end
    for i=1:501
        if zc(i) > 0.001
            C(i) = 0;
            CD = 0;
        else    
            tup = (i-1)*0.2;
            D(i) = 1;
            CD = 1;
        break;
        end
    end
end
    
if (CC == 0) && (CD == 1)
    data(n,1) = zd;
    data(n,2) = k1;
    data(n,3) = k2;
    data(n,4) = tup;
    n = n + 1;
    break;
end
end
end
end%%
data = data(1:n-1,:); % data変数の切り出し(余分な箇所をカットする)
csvwrite('data_drone_3.csv', data); % dataファイルをCSVファイルとして出力
n = n - 1; % nは1個余分にカウントしているので引いておく