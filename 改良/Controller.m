classdef Controller
    properties
    type
    L_Kp = [2950.547028; 927.519291];
    L_Kv = [15.125556; 28.364980];

    CTC_Kp = [1000.000000; 800.000000];
    CTC_Kv = [200.000000; 160.000000];

    DFF_Kp = [654.144153; 301.901881];
    DFF_Kv = [4.522670; 1.442230];
    end
    
    methods
        function this = Controller(type)
            this.type = type;
        end  
        
        % 控制器型態
        function Output = getType(this)
            Output = this.type;
        end
        
        %扭矩
        function ControllerTorque = Torque(this, PCmd, P, VCmd, V, ACmd, M, C)
                switch this.type
                    case 'Linear'
                        %PD
                        ControllerTorque = this.L_Kp .* (PCmd - P) + ...
                                           this.L_Kv .* (VCmd - V);
                    case 'CTC'
                        %CTC
                        ControllerTorque = M * (this.CTC_Kp .* (PCmd - P) + ...
                                                this.CTC_Kv .* (VCmd - V) + ...
                                                ACmd) + ...
                                           C * VCmd;
                    case 'DFF'
                        %DFF
                        ControllerTorque = M * ACmd + ...
                                           C * VCmd + ...
                                           this.DFF_Kp .* (PCmd - P) + ...
                                           this.DFF_Kv .* (VCmd - V);
                end
        end
        
    end
    
end
    