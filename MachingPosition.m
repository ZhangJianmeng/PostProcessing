classdef MachingPosition < handle
    %MACHINGPOISITION �������ڴ���������λ��֮��Ĺ�ϵ
    %   ������Ҫ����������λ��֮��Ĳ�ֵ�����Ҹ��ݲ�ֵ���
    %   ���MachingInput��WorkingMachine�У������Ӧ�ľ���
    %   �õ���Ӧ�ľ���������ڸ�����ڲ����йؼ���ļ���
    
    properties
        CutterLocation1   %��λ��1
        CutterLocation2   %��λ��2
        Workingmachine    %��������
        divide_t          %��ɢ������
        Z_omega           %��ǰ�жϵĹ���������λ�ã�z�����ϵĸ߶�
        FiveMachine       %ʹ�õ��������
    end
    
    methods
        %���캯��
        function obj=MachingPosition(Machinginput1,Machinginput2,machine,fivemachine)
            obj.CutterLocation1=Machinginput1;
            obj.CutterLocation2=Machinginput2;
            obj.Workingmachine=machine;
            obj.FiveMachine=fivemachine;
        end
        
        %������Զ����Ϊ2����λ������о�,ȷ��������λ��֮��ӹ�����͵�,�Ӷ���ȡ���жϵĲ������
        function [min_Z,max_Z]=GetHighest(obj,t)
            obj.divide_t=t;
            zmins=zeros(1,obj.divide_t+1);
            zmaxs=zeros(1,obj.divide_t+1);
            for i=1:1:obj.divide_t+1
                %����ʼλ��ҲҪ��������
                Machinginput=obj.CalLinearInput(i-1);
                [zmins(i),zmaxs(i)]=obj.Workingmachine.GetHighOfTools(Machinginput);
            end
            min_Z=max(zmins);
            max_Z=max(zmaxs);%��ߴ�
        end
        
        %����ؼ��㣬ͨ����������λ��֮����л��֣���ȡN������������
        %��ÿһ��tʱ�̶�Ӧ�Ļ������������ж���ӹ������
        %�ж���Ϻ������Ӧ�Ĺؼ��㣬���洢����
        function key_points=CalKeyPoints(obj,z_omega,t)
            %���ȸ��ݵ�ǰ���Z_omega��������l,������l���ж��Ƿ�������С�
            %ͬ������CL1��CL2֮��������Էָ�
            %һ���Ƿ���t��ʱ��
            obj.Z_omega=z_omega;%�����ʱ��͸�ֵ��
            obj.divide_t=t;
            key_points=cell(1,t+1);%һ����ô����ڵ�
            for i=1:1:t+1
                Machinginput=obj.CalLinearInput(i-1);%��ȡ��ǰʱ���µĻ���������
                Matrix=obj.Workingmachine.ConstructionMatrix(Machinginput);
                R=obj.Workingmachine.cutter.R1;
                %l����⹫ʽ,�����ʦ�ֵĲ�һ��
                %                 l=z_omega-Matrix(3,1)*(R*cos(Machinginput.B))-Matrix(3,4);%������ʱ������ͨ����
                %                 l=l/Matrix(3,3);
                l=(z_omega-Matrix(3,4))/Matrix(3,3);
                %����l���ж�
                Lmin=-R*tan(abs(Machinginput.B));
                Lmax=-Lmin;
                Ltop=obj.Workingmachine.cutter.L-Lmax;
                
                %����ؼ��㣬��Ϊ�������
                if (l<=Lmin) || (l>=Ltop)
                    key_points{1,i}=[];%ɶ��û�е�
                elseif (Lmax<=l) && (l<=Ltop)
                    %ȫ�е��ˣ��Ǹ���������Բ
                    theta=[0,2*pi];%���������ʽ�洢��Ӧ��ֵ
                    key_points{1,i}=obj.GetsEllipseKeyPoints(theta,Machinginput);
                elseif (Lmin<l)  && (l<Lmax)
                    theta_T=asin(l/(R*tan(Machinginput.B)));
                    if Machinginput.B<0
                        theta=[pi/2+theta_T,3*pi/2-theta_T];
                    else
                        %�������ܻ���ָ�ֵ��
                        %�Ƿ���Ҫͬ��ת��Ϊmod��2*pi���������Ҫ˼��
                        %��������������İ취��������Ҫת��
                        theta=[-pi/2-theta_T,pi/2+theta_T];
                    end
                    %����Բ��ֱ�߶����
                    points=obj.GetsEllipseKeyPoints(theta,Machinginput);
                    key_points{1,i}=[points,obj.GetsLinearKeyPoints(Machinginput)];
                end
                
                %�����Ҫ�üӹ��Ĺ켣��ʾ��������ֱ�Ӷ�theta�����߾���
            end
        end
        
        %��������Ļ�����������thetaֵ��������Ӧ�Ĺؼ��㣬��Ե�����Բ
        function [keypoints]=GetsEllipseKeyPoints(obj,theta,Machinginput)
            alpha=obj.calEllipseKeyPointsAlpha(Machinginput);
            beta=obj.calEllipseKeyPointsBeta(alpha);
            t = solveQuartic(1, beta(4), beta(3), beta(2), beta(1));%����Ĵη���
            idx = abs(imag(t)) < 1e-5;
            key_theta = 2*atan(real(t(idx)));%��ȡ��Ӧ��thetaֵ
            %��theta����[-pi, pi]��ֵת����[0,2*pi], ���Ұ��� ���� ��������
            key_theta = sort(mod(key_theta,2*pi));
            key_theta=key_theta';%���1*n�ľ���
            points=[];
            for i=1:1:length(key_theta)
                %������õ���ֵ��ȷ�����ǵ�theta��Χ��
                %(theta-theta(0))*(theta-theta(1))
                value1=key_theta(:,i)-theta(1);
                value2=key_theta(:,i)-theta(2);
                if (value1*value2)<=0
                    points=[points,key_theta(:,i)];
                end
            end
            M=obj.Workingmachine.ConstructionMatrix(Machinginput);%�任����
            R=obj.Workingmachine.cutter.R1;
            %�����йؼ����ɲ�����Ϊ��������ϵ�еĵ�
            keypoints=zeros(4,length(points));
            l=obj.Z_omega-M(3,1)*(R*cos(Machinginput.B))-M(3,4);%������ʱ������ͨ����
            l=l/M(3,3);
            for i=1:1:length(points)
                value=[R*cos(points(i));R*sin(points(i));l;1];
                keypoints(:,i)=M*value;
            end
        end
        
        %����ֱ�߶εĹؼ��㣬��Ӧֱ����Ӧ�Ĺ�ʽ
        function [keypoints]=GetsLinearKeyPoints(obj,Machinginput)
            R=obj.Workingmachine.cutter.R1;
            M=obj.Workingmachine.ConstructionMatrix(Machinginput);%�任����
            [DeltaX,DeltaY,DeltaZ,DeltaB,DeltaC] = obj.calcInputDelta;%�任�������
            DM=obj.Workingmachine.ConstructionDerMatrix(Machinginput,DeltaX,DeltaY,DeltaZ,DeltaB,DeltaC);
            X_LT=(obj.Z_omega-M(3,4))/M(3,1);
            dX_LT=DM(3,4)*M(3,1)+(obj.Z_omega-M(3,4))*DM(3,1);
            dX_LT=-dX_LT/(M(3,1)^2);
            s=(M(1,2)*DM(2,1)-M(2,2)*DM(1,1))*X_LT+(M(1,2)*DM(2,4)-M(2,2)*DM(1,4))...
                +(M(1,2)*M(2,1)-M(2,2)*M(1,1))*dX_LT;
            s_l=(M(1,2)*DM(2,2)-DM(1,2)*M(2,2))*(R^2-X_LT^2)^0.5;
            s=-s/s_l;
            %����ת��Ϊ��������ϵ�µĵ�
            %Ŀǰ�����Ϊ�ؼ��㣬����ͨ�����������õ�
            if(-1<=s)&&(s<=1)
                %��ת��Ϊ��������ϵ
                %                 keypoints=s;
                %ת��Ϊ��������ϵ�µ�����
                value=[X_LT;s*(R.^2-X_LT.^2)^0.5;0;1];
                keypoints=M*value;
            else
                keypoints=[];
            end
        end
        
        
        %����ĳ�������tʱ�̶�Ӧ�Ļ������������,ֻ��Ҫ�����ʱ�̶�Ӧ�Ļ�������ϵ���꼴��
        %�ú��������㹫ʽ�е�alpha����Ӧsintheta,costheta�ȵ�ϵ��
        function [alpha]=calEllipseKeyPointsAlpha(obj,Machinginput)
            [DeltaX,DeltaY,~,DeltaB,DeltaC] = obj.calcInputDelta;
            Xm=Machinginput.Xm;
            Ym=Machinginput.Ym;
            B=Machinginput.B;
            x0=Xm-obj.FiveMachine.detP(1);
            R=obj.Workingmachine.cutter.R1;
            
            %Ӧ�õĳ�����
            Xconst=Xm-obj.FiveMachine.detP(1);
            Yconst=Ym-obj.FiveMachine.detP(2);
            Zconst=obj.Z_omega-obj.FiveMachine.detP(3)+obj.FiveMachine.detP(3);
            
            %ȷ��alpha1��ֵ
            alpha1=(Zconst-(sin(B)*Xconst))*DeltaB-DeltaX*cos(B)-Yconst*DeltaC*(cos(B)^2);
            alpha1=alpha1*R/(cos(B))^2;
            %ȷ��alpha2��ֵ
            alpha2=DeltaC*x0-DeltaY*cos(B)+DeltaC*sin(B)*(-Zconst);
            alpha2=alpha2/(cos(B)^2);
            %ȷ��alpha3��ֵ
            alpha3=DeltaC*sin(B)*(R)^2;
            alpha3=alpha3/(cos(B)^2);
            %ȷ��alpha4��ֵ
            alpha4=-DeltaC*sin(B)/(cos(B)^2);
            
            alpha=[alpha1,alpha2,alpha3,alpha4];
        end
        
        %���������alphaֵ�����õ���ʽ����Ӧ��beta
        %alpha��һ��1*4�ľ���
        %���ص�beta��һ��1*4�ľ��󣬶�Ӧ4��ϵ��
        function [beta]=calEllipseKeyPointsBeta(obj,alpha)
            beta=ones(1,4);%һ��5��
            beta(1)=-(alpha(1)+alpha(4))/(alpha(1)-alpha(4));%��Ӧ��ʽ���beta0
            beta(2)=-2*(alpha(2)+alpha(3))/(alpha(1)-alpha(4));%��Ӧ��ʽ���beta1
            beta(3)=2*alpha(4)/(alpha(1)-alpha(4));%��Ӧ��ʽ���beta2
            beta(4)=-2*(alpha(2)-alpha(3))/(alpha(1)-alpha(4));%��Ӧ��ʽ���beta3
        end
        
        %���������ֵ���������λ��֮��ĳ���ض�ʱ�̴��Ļ����������
        function [Machinginput]=CalLinearInput(obj,t)
            t=t/obj.divide_t;
            Xm=obj.CutterLocation1.Xm+(obj.CutterLocation2.Xm-obj.CutterLocation1.Xm)*t;
            Ym=obj.CutterLocation1.Ym+(obj.CutterLocation2.Ym-obj.CutterLocation1.Ym)*t;
            Zm=obj.CutterLocation1.Zm+(obj.CutterLocation2.Zm-obj.CutterLocation1.Zm)*t;
            B=obj.CutterLocation1.B+(obj.CutterLocation2.B-obj.CutterLocation1.B)*t;
            C=obj.CutterLocation1.C+(obj.CutterLocation2.C-obj.CutterLocation1.C)*t;
            Machinginput=MachingInput(Xm,Ym,Zm,B,C);
        end
        
        %��ȡ����֮��Ĳ�ֵ����������ֵһ��������������λ��ͷָ�����divide_t)��Ͳ����ٸı�
        function [DeltaX,DeltaY,DeltaZ,DeltaB,DeltaC] = calcInputDelta(obj)
            DeltaX = (obj.CutterLocation2.Xm-obj.CutterLocation1.Xm)/obj.divide_t;
            DeltaY = obj.CutterLocation2.Ym-obj.CutterLocation1.Ym/obj.divide_t;
            DeltaZ = obj.CutterLocation2.Zm-obj.CutterLocation1.Zm/obj.divide_t;
            DeltaB = obj.CutterLocation2.B-obj.CutterLocation1.B/obj.divide_t;
            DeltaC = obj.CutterLocation2.C-obj.CutterLocation1.C/obj.divide_t;
        end
    end
end

