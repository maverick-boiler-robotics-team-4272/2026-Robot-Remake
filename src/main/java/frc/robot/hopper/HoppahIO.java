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

        public boolean motorUpIsConnected = false; 
        public double motorUpStatorCurrent = 0;
        public double motorUpSupplyCurrent = 0;
        public double motorUpVelocityRPS = 0;
        public double motorUpSupplyVoltage = 0;
        public double motorUpOutputVolts = 0;
    }//please work 🙏

    public default void updateInputs (HoppahIOInputs inputs) {}

    public default void setHoppahState(double bottomSpin, double topSpin) {} //🙏🙏🙏

    public default void defaultState() {}
}