package frc.robot.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
    @AutoLog
    public class FeederIOInputs {
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
    }
     public default void updateInputs (FeederIOInputs inputs) {}

    public default void setFeederState(double spin) {}

    public default void defaultState() {}
    
}
