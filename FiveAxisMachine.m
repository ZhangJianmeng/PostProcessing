classdef FiveAxisMachine < handle
    %FIVEAXISMACHING ����Ϊ�������
    %   ���ڼ�������ĵ�λ���ڹ����µ�����ת��Ϊ��������ϵ�µ�����
    %   �洢���������������ͶԵ��Ĺ�������ϵƫ����
    %   ��ǰ��;����Ϊ���㵶λ���ڻ�������ϵ�µ����꼴��
    
    properties
        detP             %������ڻ�������ϵ�е����꣨�������У�
        detW             %�ӹ�����ϵԭ���ڻ�������ϵ�е����꣨ͨ���Ե��õ���
        L                %���߳���
    end
    
    methods
        %���캯�������벻���ٸı��ֵ
        function obj=FiveAxisMachine(detP_Input,detW_Input,L_Input)
            obj.detP=detP_Input;
            obj.detW=detW_Input;
            obj.L=L_Input;
        end
        
        %����õ���ǰ��λ��͵���ʸ����Ӧ�Ļ�������ϵ�е��������Ӧ��B��Cֵ
        %BCΪ����ֵ
        function [Machinginput]=ComputePoisition(obj,Point_X,Point_Y,Point_Z,Nx,Ny,Nz)
            if Nz~=1
                B=acos(Nz);
                if Ny~=0
                    if  (Nx>0)&&(Ny>0)
                        C=pi-abs(atan(Ny/Nx));
                    elseif (Nx==0)&&(Ny>0)
                        C=pi/2;
                    elseif (Nx<0)&&(Ny>0)
                        C=atan(-Ny/Nx);
                    elseif (Nx<0)&&(Ny<0)
                        C=atan(-Ny/Nx);
                    elseif (Nx==0)&&(Ny<0)
                        C=-pi/2;
                    elseif (Nx>0)&&(Ny<0)
                        C=-pi+atan(-Ny/Nx);
                    end
                else
                    B=atan(-Nx/Nz);
                    C=0;
                end
            else
                B=0;C=0;
            end
            
            X_const=Point_X-obj.detP(1)+obj.detW(1);
            Y_const=Point_Y-obj.detP(2)+obj.detW(2);
            Z_const=Point_Z-obj.detP(3)+obj.detW(3);
            
            Xm=cos(B)*cos(C)*X_const-cos(B)*sin(C)*Y_const+sin(B)*Z_const+obj.detP(1);
            
            Ym=sin(C)*X_const+cos(C)*Y_const+obj.detP(2);
            
            Zm=-sin(B)*cos(C)*X_const+sin(B)*sin(C)*Y_const+cos(B)*Z_const+obj.detP(3)+obj.L;
            
            Machinginput=MachingInput(Xm,Ym,Zm,B,C);
        end
    end
    
end

