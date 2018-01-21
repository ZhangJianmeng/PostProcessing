function [ points ] = PlotStartEnd(z_omega,Machingposition )
%PLOTSTARTEND �ú������ڻ���ĳ���ض�zomega����ʼ��λ�����ֹ��λ�㴦�ĵ�
%   ������鵶λ���Ƿ�����
Machingposition.Z_omega=z_omega;
AllMachinginput=[Machingposition.CutterLocation1,Machingposition.CutterLocation2];
points=[];
for i=1:1:2
    Machinginput=AllMachinginput(i);
    Matrix=Machingposition.Workingmachine.ConstructionMatrix(Machinginput);
    R=Machingposition.Workingmachine.cutter.R1;
    l=(z_omega-Matrix(3,4))/Matrix(3,3);
    %����l���ж�
    Lmin=-R*tan(abs(Machinginput.B));
    Lmax=-Lmin;
    Ltop=Machingposition.Workingmachine.cutter.L-Lmax;
    t=100;
    if (l<=Lmin) || (l>=Ltop)
        continue;
    elseif (Lmax<=l) && (l<=Ltop)
        %ȫ�е��ˣ��Ǹ���������Բ
        theta=[0,2*pi];%���������ʽ�洢��Ӧ��ֵ
        %����theta������
        points=[points,GetEllipsePoints(theta,Matrix,t,z_omega,R,Machinginput.B)];
    elseif (Lmin<l)  && (l<Lmax)
        theta_T=asin(l/(R*tan(Machinginput.B)));
        if Machinginput.B<0
            theta=[pi/2+theta_T,3*pi/2-theta_T];
        else
            theta=[-pi/2-theta_T,pi/2+theta_T];
        end
        points=GetEllipsePoints(theta,Matrix,t,z_omega,R,Machinginput.B);
        points=[points,GetLinePoints(z_omega,Matrix,t,R)]
        %����Բ��ֱ�߶����
    end
end
end

