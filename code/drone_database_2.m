%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% JIT���f�����O�ɂ��h���[���̕���̈��萧��̃f�[�^�x�[�X(�ׂ���)
%
% ���́F�����̖ڕW�l zd
% �o�́F������͂̃Q�C�� k1, k2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% �Œ肷��p�����[�^ %%%
h = 0.5; %�@�T���v�����O�Ԋu
t = 10 ; %�@����ɓK�؂Ȏ��Ԃ��Z�o���邽�ߕϐ�
k3 = 0.08; %x������̊p���x�Ɋւ��Q�C���@
k4 = 0.08; %y������̊p���x�Ɋւ��Q�C��
k5 = 0.09; %z������̊p���x�Ɋւ��Q�C��
pd = 0; %x������̊p�x�̖ڕW�l[rad]
qd = 0; %y������̊p�x�̖ڕW�l[rad]
rd = 0; %z������̊p�x�̖ڕW�l[rad]

%%% JIT�ŕω�������p�����[�^ %%%
k1min = 0.08; %z�������̕΍��Ɋւ��Q�C���̍ŏ��l(0.02)
k1h = 0.005; %z�������̕΍��Ɋւ��Q�C���̍��ݕ�(0.001)
k1max = 0.6; %z�������̕΍��Ɋւ��Q�C���̍ő�l(0.4)

k2min = 0.5; %z�������̑��x�Ɋւ��Q�C���̍ŏ��l(0.2)
k2h = 0.001; %z�������̑��x�Ɋւ��Q�C���̍��ݕ�(0.005)
k2max = 1.2; %z�������̑��x�Ɋւ��Q�C���̍ő�l(1.3)

zdmin = -5; %�ڕW���x�̍ŏ��l
zdh = 0.5; %�ڕW���x�̍��ݕ�
zdmax = 5; %�ڕW���x�̍ő�l

a = ((k1max-k1min+k1h)/k1h)*((k2max-k2min+k2h)/k2h)*((zdmax-zdmin+zdh)/zdh);
b = round(a);

%%% �f�[�^�x�[�X�̍쐬 %%%
data = zeros(b,4);
n = 1;
for zd = zdmin:zdh:zdmax
for k1 = k1min:k1h:k1max
for k2 = k2min:k2h:k2max

   rzd = 10 + zd; 

[T, U] = ode45(@droneupextrabackl,(0:0.2:100),[0;0;0;0;10;0;0;0;0;0;0;0],[],k1,k2,k3,k4,k5,rzd,pd,qd,rd);

%%% �I�[�o�[�V���[�g�Ə㏸���Ԃ̔���
zc = zeros(501,1); % �s���߂��ʂ��i�[����ϐ�
C = zeros(501,1); % �I�[�o�[�V���[�g����萔(C=0�̓I�[�o�[�V���[�g�Ȃ�,C=1�̓I�[�o�[�V���[�g����)
D = zeros(501,1); % �㏸��������萔(C=0�͖��㏸,C=1�͏㏸�B��)
CC = 0; % �I�[�o�[�V���[�g�t���O�萔
CD = 0; % �㏸�����t���O�萔



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
data = data(1:n-1,:); % data�ϐ��̐؂�o��(�]���ȉӏ����J�b�g����)
csvwrite('data_drone_3.csv', data); % data�t�@�C����CSV�t�@�C���Ƃ��ďo��
n = n - 1; % n��1�]���ɃJ�E���g���Ă���̂ň����Ă���