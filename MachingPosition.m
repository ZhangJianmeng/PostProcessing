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
    end
    
    methods
        function obj=MachingPosition(Machinginput1,Machinginput2,machine)
            obj.CutterLocation1=Machinginput1;
            obj.CutterLocation2=Machinginput2;
            obj.Workingmachine=machine;
        end
        
        %������Զ����Ϊ2����λ������о�
        function high_Z=GetHighest(obj,t)
            obj.divide_t=t;
            a=zeros(1,obj.divide_t);
            for i=1:1:obj.divide_t
                Machinginput=obj.CalLinearInput(i);
                a(i)=obj.Workingmachine.GetHighOfTools(Machinginput);
            end
            high_Z=max(a);
        end
        
        %���������ֵ���������λ��֮��ĳ���ض�ʱ�̴��Ļ����������
        function Machinginput=CalLinearInput(obj,t)
            t=t/obj.divide_t;
            Xm=obj.CutterLocation1.Xm+(obj.CutterLocation2.Xm-obj.CutterLocation1.Xm)*t;
            Ym=obj.CutterLocation1.Ym+(obj.CutterLocation2.Ym-obj.CutterLocation1.Ym)*t;
            Zm=obj.CutterLocation1.Zm+(obj.CutterLocation2.Zm-obj.CutterLocation1.Zm)*t;
            B=obj.CutterLocation1.B+(obj.CutterLocation2.B-obj.CutterLocation1.B)*t;
            C=obj.CutterLocation1.C+(obj.CutterLocation2.C-obj.CutterLocation1.C)*t;
            Machinginput=MachingInput(Xm,Ym,Zm,B,C);
        end
    end
end

