filename = 'data_U.csv';%速度読み込み
U = csvread(filename);
L=length(U);
filename = 'data_wind_xyz.csv';%風速読み込み
d = csvread(filename);
n=length(U);
fore=[];
%%
for i=1:L
vx=U(i,2);
vy=U(i,4);
vz=U(i,6);
x=U(i,7);
y=U(i,9);
%x=U(i,7);%x軸周りの姿勢角
%y=U(i,9);%y軸周りの姿勢角
%x_mean=mean(U(:,7));
%y_mean=mean(U(:,9));
vx_mean=mean(U(:,2));
vy_mean=mean(U(:,4));
vz_mean=mean(U(:,6));
x_mean=mean(U(:,7));
y_mean=mean(U(:,9));
dd=[];
for j=1:n
    %if x-2*x_mean<=d(j,1)&&d(j,1)<=x+2*x_mean
    if vx-2*vx_mean<=d(j,1)&&d(j,1)<=vx+2*vx_mean
    if vy-2*vy_mean<=d(j,2)&&d(j,2)<=vy+2*vy_mean
    if vz-2*vz_mean<=d(j,3)&&d(j,3)<=vz+2*vz_mean
    if x-2*x_mean<=d(j,5)&&d(j,5)<=x+2*x_mean
    if y-2*y_mean<=d(j,6)&&d(j,6)<=y+2*y_mean
        dd=[dd;d(j,1),d(j,2),d(j,3),d(j,5),d(j,6),d(j,7)];%条件を満たすデータを選ぶ
    end
    end
    end
    end
    end
end
[row_count, column_count] = size(dd);
if  isempty(dd)
    dd=[0,0,0,0,0,0,0];
end
for k=1:row_count
    dd(k,7) = sqrt((vx-dd(k,1))^2+(vy-dd(k,2))^2+(vz-dd(k,3))^2);%距離を求める
end
dz = sortrows(dd,7);%距離によりソート
Dx = 0; % 求めるdxの値の初期化
Dy = 0; % 求めるdyの値の初期化
Dz = 0; % 求めるdzの値の初期化
D  = 0;% 距離の逆数の総和の初期化
if dd(1,7) == 0 
    Dx= dd(1,4); % もし要求データがデータベースに一致すればそのデータを使う
    Dy= dd(1,5);% もし要求データがデータベースに一致すればそのデータを使う
    Dz= dd(1,6);
else
    for l=1:row_count
        Dx= Dx + dd(l,4)/dd(l,7);
        Dy= Dy + dd(l,5)/dd(l,7);
        Dz= Dz + dd(l,6)/dd(l,7);
        D = D + 1/dd(l,7);
    end
     Dx = Dx/D; % JITで求めたdx
     Dy = Dy/D; % JITで求めたdy
     Dz = Dz/D; % JITで求めたdz
end
isZeroMatrix = all(dd(:) == 0);
if isZeroMatrix
    Dx=0.0428;
    Dy=0.0033;
    Dz=0.0091;
end
fore=[fore;Dx,Dy,Dz];
end
%%
%%% 後処理 %%%
csvwrite('data_dxyz_fore.csv', fore); % dataファイルをCSVファイルとして出力






