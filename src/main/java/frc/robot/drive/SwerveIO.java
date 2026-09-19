package frc.robot.drive;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public interface SwerveIO {
    @AutoLog
    public static class SwerveIOInputs extends SwerveDriveState {
        double yawVelocityRadPerSec = 0.0;
        double yawAccelerationRadPerSecPerSec = 0.0;

        void fromDriveState(SwerveDriveState state) {
            this.Pose = state.Pose;
            this.Speeds = state.Speeds;
            this.ModuleStates = state.ModuleStates;
            this.ModuleTargets = state.ModuleTargets;
            this.ModulePositions = state.ModulePositions;
            this.RawHeading = state.RawHeading;
            this.Timestamp = state.Timestamp;
            this.OdometryPeriod = state.OdometryPeriod;
            this.SuccessfulDaqs = state.SuccessfulDaqs;
            this.FailedDaqs = state.FailedDaqs;
        }
    }

    @AutoLog
    public static class ModuleIOInputs {
        public boolean driveConnected = false; //TODO: hmmm... implemet these somehow. probably another method...
        public double driveStatorCurrent = 0.0;
        public double driveSupplyCurrent = 0.0;
        public double driveVelocity = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveTemp = 0.0;

        public boolean steerConnected = false; //TODO: hmmm... implemet these somehow. probably another method...
        public double steerStatorCurrent = 0.0;
        public double steerSupplyCurrent = 0.0;
        public double steerVelocity = 0.0;
        public double steerAppliedVolts = 0.0;
        public double steerTemp = 0.0;
        
        public Rotation2d moduleAngle = Rotation2d.kZero;
    }

    default void updateSwerveInputs(SwerveIOInputs inputs) {};

    default void updateModuleInputs(ModuleIOInputs... inputs) {};

    default void logModules(SwerveDriveState state) {};

    default void setControl(SwerveRequest request) {}

    default void resetPose(Pose2d pose) {}

    default void resetHeading() {}

    default void setStatorCurrentLimit(double limit) {}

    default void setSupplyCurrentLimit(double limit) {}

    default void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs) {}
}
