%�ṩԭʼ���ݣ��������Ѿ��洢��datas.mat�У������޸�
clear all;clc;
%ȷ������
detP=[0 0 100];%������ڻ�������ϵ�е����꣨�������У�
detW=[10 10 200];%�ӹ�����ϵԭ���ڻ�������ϵ�е����꣨ͨ���Ե��õ���
L=50;%������������
machine=FiveAxisMachine(detP,detW,L);

%ȷ����λ��1
Point_X1=100;%��������ߵ�λ���ڼӹ�����ϵ�е�����
Point_Y1=300;
Point_Z1=300;
Middle_Vector=[0 0 1]; %����ʸ���ڼӹ�����ϵ�еı�ʾ
Middle_Vector=Middle_Vector/norm(Middle_Vector);
Nx=Middle_Vector(1);%x����
Ny=Middle_Vector(2);%y����
Nz=Middle_Vector(3);%z����
Machinginput1 = machine.ComputePoisition(Point_X1,Point_Y1,Point_Z1,Nx,Ny,Nz );

%ȷ����λ��2
Point_X2=95;%��������ߵ�λ���ڼӹ�����ϵ�е�����
Point_Y2=310;
Point_Z2=300;
Middle_Vector2=[0 0 1]; %����ʸ���ڼӹ�����ϵ�еı�ʾ
Middle_Vector2=Middle_Vector2/norm(Middle_Vector2);
Nx2=Middle_Vector2(1);%x����
Ny2=Middle_Vector2(2);%y����
Nz2=Middle_Vector2(3);%z����
Machinginput2= machine.ComputePoisition(Point_X2,Point_Y2,Point_Z2,Nx2,Ny2,Nz2 );
%ȷ������
cutter=FilletGrindingWheel(10,0,30);
%ȷ����������
MyWorkingMachine=WorkingMachine(machine,cutter);
%������������λ��֮��Ĺ�ϵ
Machingposition=MachingPosition(Machinginput1,Machinginput2,MyWorkingMachine,machine);
