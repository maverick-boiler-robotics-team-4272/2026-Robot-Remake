package frc.robot.hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HoppahIO { 
    @AutoLog
    public class HoppahIOInputs {
        public boolean motorLIsConnected = false; 
        public double motorLStatorCurrent = 0;
        public double motorLSupplyCurrent = 0;
        public double motorLVelocityRPS = 0;
        public double motorLSupplyVoltage = 0;
        public double motorLOutputVolts = 0;

        public boolean motorRIsConnected = false; 
        public double motorRStatorCurrent = 0;
        public double motorRSupplyCurrent = 0;
        public double motorRVelocityRPS = 0;
        public double motorRSupplyVoltage = 0;
        public double motorROutputVolts = 0;

        public boolean motorBLIsConnected = false; 
        public double motorBLStatorCurrent = 0;
        public double motorBLSupplyCurrent = 0;
        public double motorBLVelocityRPS = 0;
        public double motorBLSupplyVoltage = 0;
        public double motorBLOutputVolts = 0;

        public boolean motorBRIsConnected = false; 
        public double motorBRStatorCurrent = 0;
        public double motorBRSupplyCurrent = 0;
        public double motorBRVelocityRPS = 0;
        public double motorBRSupplyVoltage = 0;
        public double motorBROutputVolts = 0;
    }//please work 

    public default void updateInputs (HoppahIOInputs inputs) {}

    public default void setHoppahState(double bottomSpin, double topSpin) {} //Jesus take the button board

    public default void defaultState() {}
}