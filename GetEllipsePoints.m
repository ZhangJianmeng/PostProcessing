function [ points ] = GetEllipsePoints( theta,M,t,Z_omega,R,B )
%GETELLIPSEPOINTS ��������Բ
%   MΪ��Ӧ�ı仯����thetaΪ�����ķ�Χ��tΪ��ɢ������Z_omegaΪë������߶�,RΪƽ�׵����߰뾶
%  BΪ�Ƕ�
points=zeros(4,t);
theta=linspace(theta(1),theta(2),t);
l=Z_omega-M(3,1)*(R*cos(B))-M(3,4);%������ʱ������ͨ����
l=l/M(3,3);
for i=1:1:t
    value=[R*cos(theta(i));R*sin(theta(i));l;1];
    points(:,i)=M*value;
end

