classdef MachingPosition < handle
    %MACHINGPOISITION 该类用于处理两个刀位点之间的关系
    %   该类主要处理两个刀位点之间的插值，并且根据插值结果
    %   输出MachingInput到WorkingMachine中，获得相应的矩阵
    %   得到相应的矩阵后，拿来在该类的内部进行关键点的计算
    
    properties
        CutterLocation1   %刀位点1
        CutterLocation2   %刀位点2
        Workingmachine    %工作机床
        divide_t          %离散的数量
    end
    
    methods
        function obj=MachingPosition(Machinginput1,Machinginput2,machine)
            obj.CutterLocation1=Machinginput1;
            obj.CutterLocation2=Machinginput2;
            obj.Workingmachine=machine;
        end
        
        %问题永远都简化为2个刀位点进行研究
        function high_Z=GetHighest(obj,t)
            obj.divide_t=t;
            a=zeros(1,obj.divide_t);
            for i=1:1:obj.divide_t
                Machinginput=obj.CalLinearInput(i);
                a(i)=obj.Workingmachine.GetHighOfTools(Machinginput);
            end
            high_Z=max(a);
        end
        
        %根据输入的值，求解两个位置之间某个特定时刻处的机床输入参数
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

