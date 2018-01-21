function [ points ] = GetLinePoints( Z_omega,M,t,R)
%GETLINEPOINTS �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
s=linspace(-1,1,t);
points=zeros(4,t);
X_LT=(Z_omega-M(3,4))/M(3,1);
for i=1:1:t
    value=[X_LT;s*(R.^2-X_LT.^2)^0.5;0;1];
    points(i)=M*value;
end

end

