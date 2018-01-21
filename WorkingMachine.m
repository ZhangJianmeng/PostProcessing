classdef WorkingMachine < handle
    %MACHINEINPUT �������ڰ���������ô�����صľ���
    %   ���ڴ洢��Ӧ�ı任����
    %   �����������ĵ�λ���ڼӹ�����ϵ�е��������Ӧ�ĵ���ʸ��
    %   �������ڴ����������صļ���
    %   ֻ�����������ϵ�µ����꣬��Ҫ��ʱ��ʲô�ĳ��Ϲ�ϵ
    %   ʱ��Ĵ������������ĽӴ�
    
    properties
        
        Matrix           %���ô���ľ���4X4����
        Machine          %ʹ�õĻ������ṩXm,Ym,Zm,B,C�ļ���
        Der_Matrix       %���ô���4X4�������
        cutter            %����
    end
    
    methods
        %���캯������������ʹ�õĻ���
        function obj=WorkingMachine(Input_Machine,tools)
            obj.Machine=Input_Machine;
            obj.cutter=tools;
        end
        
        %�����BCΪ����ֵ,ʵ�ֶԱ任��������,��Ȼ�����˷���ֵ����ͬ����洢��������
        function [Comput_Matrix]=ConstructionMatrix(obj,Machinginput)
            Xm=Machinginput.Xm;
            Ym=Machinginput.Ym;
            Zm=Machinginput.Zm;
            B=Machinginput.B;
            C=Machinginput.C;
            Comput_Matrix=zeros(4,4);
            %�����һ��
            M11=cos(B)*cos(C);
            M12=sin(C);
            M13=-sin(B)*cos(C);
            M14=(Xm-obj.Machine.detP(1,1))*M11+(Ym-obj.Machine.detP(1,2))*M12+(Zm-obj.Machine.detP(1,3)-obj.Machine.L)*M13+(obj.Machine.detP(1,1)-obj.Machine.detW(1,1));
            Comput_Matrix(1,:)=[M11,M12,M13,M14];
            
            %����ڶ���
            M21=-cos(B)*sin(C);
            M22=cos(C);
            M23=sin(B)*sin(C);
            M24=(Xm-obj.Machine.detP(1,1))*M21+(Ym-obj.Machine.detP(1,2))*M22+(Zm-obj.Machine.detP(1,3)-obj.Machine.L)*M23+(obj.Machine.detP(1,2)-obj.Machine.detW(1,2));
            Comput_Matrix(2,:)=[M21,M22,M23,M24];
            
            %���������
            M31=sin(B);
            M33=cos(B);
            M34=(Xm-obj.Machine.detP(1,1))*M31+(Zm-obj.Machine.detP(1,3)-obj.Machine.L)*M33+(obj.Machine.detP(3)-obj.Machine.detW(1,3));
            Comput_Matrix(3,:)=[M31,0,M33,M34];
            %���������
            Comput_Matrix(4,4)=1;
            
            %��ֵ
            obj.Matrix=Comput_Matrix;
            
        end
        
        %�����BCΪ����ֵ��ʵ�ֶԱ任�����󵼵���⣬ͬ������һ��4X4�ľ��󣬲��洢�ڸ�����
        %������ĵ�ǰtʱ�̵�B,Cֵ���Լ���Ӧ���Բ�ֵ���õ�dX,dY,dZ,dC,dB
        %ע�⣬�����dX,dY,dZ,dC,dB��Ϊ��ʼ�ͽ���������λ�����Բ�ֵ���ã�����2����λ�����Ͳ������
        %��B��C��Xm��Ym��Zm�⼸��ֵ���ڲ�ֵ�����в��ϱ任�ģ��ڸ�����������λ��֮����ܲ�ֵ�õ���100���µĵ�
        %��100���µĵ�ÿ������һ�������Լ���B��C��Xm��Ym��Zm
        function [ComputDer_Matrix]=ConstructionDerMatrix(obj,Machinginput,dX,dY,dZ,dB,dC)
            Xm=Machinginput.Xm;
            Ym=Machinginput.Ym;
            Zm=Machinginput.Zm;
            B=Machinginput.B;
            C=Machinginput.C;
            ComputDer_Matrix=zeros(4,4);
            x0=Xm-obj.Machine.detP(1);
            y0=Ym-obj.Machine.detP(2);
            z0=obj.Machine.L-Zm+obj.Machine.detP(3);
            
            %�����һ��
            M11=-dB*sin(B)*cos(C)-dC*cos(B)*sin(C);
            M12=dC*cos(C);
            M13=-dB*cos(B)*cos(C)+dC*sin(B)*sin(C);
            M14=dX*cos(B)*cos(C)+x0*(-dB*sin(B)*cos(C)-dC*cos(B)*sin(C))+dY*sin(C)+...
                y0*dC*cos(C)-dZ*sin(B)*cos(C)+z0*(dB*cos(B)*cos(C)-dC*sin(B)*sin(C));
            
            ComputDer_Matrix(1,:)=[M11,M12,M13,M14];
            
            %����ڶ���
            M21=dB*sin(B)*sin(C)-dC*cos(B)*cos(C);
            M22=-dC*sin(C);
            M23=dB*cos(B)*sin(C)+dC*sin(B)*cos(C);
            M24=-dX*cos(B)*sin(C)+x0*(dB*sin(B)*sin(C)-dC*cos(B)*cos(C))+dY*cos(C)...
                -y0*dC*sin(C)+dZ*sin(B)*sin(C)-z0*(dB*cos(B)*sin(C)+dC*sin(B)*cos(C));
            
            ComputDer_Matrix(2,:)=[M21,M22,M23,M24];
            
            %���������
            M31=dB*cos(B);
            M32=0;
            M33=-dB*sin(B);
            M34=dX*sin(B)+x0*dB*cos(B)+dZ*cos(B)+z0*dB*sin(B);
            ComputDer_Matrix(3,:)=[M31,M32,M33,M34];
            
            obj.Der_Matrix=ComputDer_Matrix;
        end
        
        %�ú�����Ӧ����P28,(4-5),����ʦ�ֵĳ����Լ�����P39��5-12�ı��ʽ
        %�˴���thetaΪת���ĽǶ�B������ǰ�����ת���ĽǶ�
        %B�ķ�Χ�޶���0��pi/2
        %�ú����ܹ�����������ĳ���ض������������������
        function [z0min,z0max]=GetHighOfTools(obj,Machinginput)
            %��ʱ�Ѿ��õ��˵�ǰʱ���µĵ�λ�������ˣ�
            M=obj.ConstructionMatrix(Machinginput);
            B=Machinginput.B;
            M33=M(3,3);%�����е�����
            %�����ھ���Ӧ��ʱ��R2Ϊ0��
            l0min=obj.cutter.R2*(1-cos(abs(B)))-tan(abs(B))*(obj.cutter.R1+obj.cutter.R2*sin(abs(B)));
            z0min=M33*l0min+M(3,4);
            z0max=M(3,2)*(obj.cutter.R1+obj.cutter.R2)+obj.cutter.L*M(3,3)+M(3,4);
        end
        
    end
end
