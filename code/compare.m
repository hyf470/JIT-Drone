filename = '../database/data_dxyz.csv';
data = csvread(filename);
filename = '../database/data_dxyz_fore.csv';
data_fore = csvread(filename);
data_com = data-data_fore;%実際と予測データ比較
x = data_com(:, 1);%x外乱の誤差をとる
y = data_com(:, 2);%y外乱の誤差をとる
z = data_com(:, 3);%z外乱の誤差をとる
% 図を描く
subplot(3, 1, 1);
plot(x);
title('x方向外乱（実際ー予測）');
xlabel('x');
ylabel('予測偏差');
subplot(3, 1, 2);
plot(y);
title('y方向外乱（実際ー予測）');
xlabel('y');
ylabel('予測偏差');
subplot(3, 1, 3);
plot(z);
title('z方向外乱（実際ー予測）');
xlabel('z');
ylabel('予測偏差');