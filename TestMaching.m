% load datas.mat
%datas.mat�����ṩһ���������machine��������λ��Machinginput1��Machinginput2
%Ӧ��������λ����йؼ���������Ϣ����������Machingpoisition
%MyWorkingMachine�������ǵĹ����������ṩ����;����󵼵�����

[high_z,~]=Machingposition.GetHighest(10);%���ص��ǹ�������ϵ�µĸ߶�

%֪����ߴ���ֻ��Ҫ�жϸô�����߲�֮��Ĺ�ϵ���ɣ�Ҳ�����ǿ��Կ��ǵ�������ߵ����⣬�������
z_theta=1;
%��ÿһ����з���
%����Ƿ���ÿ��Ĺؼ�������꣬���ڹ�������ϵ��
cnt=1;
key_points=cell(1,cnt);
for h=high_z+10:z_theta:high_z+20
    key_points{1,cnt}=Machingposition.CalKeyPoints(h,10);%����5��
    cnt=cnt+1;
end
%
% indx=[];
% a=zeros(1,1);
% for i=1:1:length(key_points)
%     for j=1:1:6
%         if isempty(key_points{1,i}{1,j})
%             indx=[indx,[i;j]];
%         end
%     end
% end

%��������λ�㻭����
[ points ] = PlotStartEnd(high_z+10,Machingposition );
n=size(points,2);
for i=1:1:n
    plot3(points(1,i),points(2,i),points(3,i),'*');
    hold on;
end

for i=1:1:length(key_points)
    points=cell2mat(key_points{1,i});
    n=size(points,2);
    for j=1:1:n%���еĸ�����ȡ
        plot3(points(1,j),points(2,j),points(3,j),'*');
        hold on;
    end
end