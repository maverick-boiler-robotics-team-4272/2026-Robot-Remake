package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public boolean rollerConnected = false;
        public double rollerVelocityRadPerSec = 0.0;
        public double rollerAppliedVolts = 0.0;
        public double rollerStatorCurrentAmps = 0.0;
        public double rollerSupplyCurrentAmps = 0.0;
        public double rollerTempCelsius = 0.0;

        public boolean rollerFollowerConnected = false;
        public double rollerFollowerVelocityRadPerSec = 0.0;
        public double rollerFollowerAppliedVolts = 0.0;
        public double rollerFollowerStatorCurrentAmps = 0.0;
        public double rollerFollowerSupplyCurrentAmps = 0.0;
        public double rollerFollowerTempCelsius = 0.0;

        public boolean pivotConnected = false;
        public Rotation2d pivotPosition = Rotation2d.kZero;
        public double pivotVelocityRadPerSec = 0.0;
        public double pivotAppliedVolts = 0.0;
        public double pivotStatorCurrentAmps = 0.0;
        public double pivotSupplyCurrentAmps = 0.0;
        public double pivotTempCelsius = 0.0;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    /** Roller velocity setpoint in mechanism rotations/sec. */
    public default void setRollerVelocity(double rotationsPerSec) {}

    /** Pivot position setpoint in mechanism rotations. */
    public default void setPivotPosition(double rotations) {}

    public default void resetPivotPosition(Rotation2d position) {}

    public default void stop() {}
}
