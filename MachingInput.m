classdef MachingInput < handle
    %������� ��������� λ�Ʋ��� [Xm,Ym,Zm,B,C] 
    %ר�����ڴ洢��Ӧ��ֵ
    properties
        Xm
        Ym
        Zm
        B
        C
    end
    
    methods
        function obj=MachingInput(Xm,Ym,Zm,B,C)
            obj.Xm = Xm;
            obj.Ym = Ym;
            obj.Zm = Zm;
            obj.B = B;
            obj.C = C;
        end
    end
end

