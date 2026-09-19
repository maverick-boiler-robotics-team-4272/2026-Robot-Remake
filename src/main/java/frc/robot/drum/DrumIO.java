package frc.robot.drum;

import org.littletonrobotics.junction.AutoLog;

public interface DrumIO {
    @AutoLog
    public static class DrumIOInputs{
        public boolean motorLIsConnected = false;
        public double motorLStatorCurrent = 0;
        public double motorLSupplyCurrent = 0;
        public double motorLVelocityRPS = 0;
        public double motorLSupplyVoltage = 0;
        public double motorLOutputVoltage = 0;

        public boolean motorRIsConnected = false;
        public double motorRStatorCurrent = 0;
        public double motorRSupplyCurrent = 0;
        public double motorRVelocityRPS = 0;
        public double motorRSupplyVoltage = 0;
        public double motorROutputVoltage = 0;

        public boolean motorHoodIsConnected = false;
        public double motorHoodPosition = 0;
        public double motorHoodStatorCurrent = 0;
        public double motorHoodSupplyCurrent = 0;
        public double motorHoodVelocityRPS = 0;
        public double motorHoodSupplyVoltage = 0;
        public double motorHoodOutputVoltage = 0;
    }
    public default void updateInputs(DrumIOInputs inputs) {}

    public default void setDrumState(double rps, double ang) {}

    public default void defaultState () {}
}
