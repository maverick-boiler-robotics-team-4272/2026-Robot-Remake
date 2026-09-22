package frc.robot.hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HoppahIO { 
    @AutoLog
    public class HoppahIOInputs {
        public boolean motorLeftIsConnected = false; 
        public double motorLeftStatorCurrent = 0;
        public double motorLeftSupplyCurrent = 0;
        public double motorLeftVelocityRPS = 0;
        public double motorLeftSupplyVoltage = 0;
        public double motorLeftOutputVolts = 0;

        public boolean motorRightIsConnected = false; 
        public double motorRightStatorCurrent = 0;
        public double motorRightSupplyCurrent = 0;
        public double motorRightVelocityRPS = 0;
        public double motorRightSupplyVoltage = 0;
        public double motorRightOutputVolts = 0;

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
    }//please work 🙏

    public default void updateInputs (HoppahIOInputs inputs) {}

    public default void setHoppahState(double bottomSpin, double topSpin) {} //Jesus take the button board

    public default void defaultState() {}
}