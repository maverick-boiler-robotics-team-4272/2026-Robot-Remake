package frc.robot.drive;

import java.util.HashMap;
import java.util.Map;
import java.util.stream.Stream;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import frc.robot.constants.TunerConstants;
import frc.robot.constants.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.util.StatusSignalRefresher;

import static frc.robot.constants.FieldConstants.*;
import static frc.robot.constants.SubsystemConstants.DriveConstants.*;

public class SwerveIOCTRE extends TunerSwerveDrivetrain implements SwerveIO {
    private interface ModuleSignalRefresher {
        void refreshSignals(ModuleIOInputs inputs);
    }
    private final Map<Integer, ModuleSignalRefresher> moduleRefreshFunctions = new HashMap<>();

    protected final StatusSignal<AngularVelocity> angularYawVelocity;
    protected final StatusSignal<LinearAcceleration> accelerationZ;

    private final String[] moduleNames = {"Drive/Modules/FL", "Drive/Modules/FR", "Drive/Modules/BL", "Drive/Modules/BR"};

    public SwerveIOCTRE() {
        super(TunerConstants.DrivetrainConstants, ODOMETRY_UPDATE_FREQUENCY, 
            MODULE_CONSTANTS);

        final int moduleCount = getModules().length;

        for(int i = 0; i < moduleCount; ++i) {
            final var module = getModule(i);

            final TalonFX drive = module.getDriveMotor();
            final TalonFX steer = module.getSteerMotor();

            var drivePosition = drive.getPosition();
            var driveStatorCurrent = drive.getStatorCurrent();
            var driveSupplyCurrent = drive.getSupplyCurrent();
            var driveVelocity = drive.getVelocity();
            var driveAppliedVolts = drive.getMotorVoltage();
            var driveTemp = drive.getDeviceTemp();

            var steerStatorCurrent = steer.getStatorCurrent();
            var steerSupplyCurrent = steer.getSupplyCurrent();
            var steerVelocity = steer.getVelocity();
            var steerAppliedVolts = steer.getMotorVoltage();
            var steerTemp = steer.getDeviceTemp();
            var moduleAngle = steer.getPosition();

            moduleRefreshFunctions.put(i, (ModuleIOInputs inputs) -> {
                inputs.driveStatorCurrent = driveStatorCurrent.getValueAsDouble();
                inputs.driveSupplyCurrent = driveSupplyCurrent.getValueAsDouble();
                inputs.driveVelocity = driveVelocity.getValueAsDouble();
                inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
                inputs.driveTemp = driveTemp.getValueAsDouble();

                inputs.steerStatorCurrent = steerStatorCurrent.getValueAsDouble();
                inputs.steerSupplyCurrent = steerSupplyCurrent.getValueAsDouble();
                inputs.steerVelocity = steerVelocity.getValueAsDouble();
                inputs.steerAppliedVolts = steerAppliedVolts.getValueAsDouble();
                inputs.steerTemp = steerTemp.getValueAsDouble();
                inputs.moduleAngle = Rotation2d.fromRadians(moduleAngle.getValueAsDouble());
            });

            StatusSignalRefresher.getInstance().addStatusSignals(
                drivePosition,
                driveSupplyCurrent,
                driveVelocity,
                driveAppliedVolts,
                driveTemp,
                steerStatorCurrent,
                steerSupplyCurrent,
                steerVelocity,
                steerAppliedVolts,
                steerTemp,
                moduleAngle
            );
        }
        angularYawVelocity = getPigeon2().getAngularVelocityZWorld();
        accelerationZ = getPigeon2().getAccelerationZ();
        BaseStatusSignal.setUpdateFrequencyForAll(250, angularYawVelocity, accelerationZ);
        StatusSignalRefresher.getInstance().addStatusSignals(angularYawVelocity, accelerationZ);
    }

    @Override
    public void updateSwerveInputs(SwerveIOInputs inputs) {
        inputs.fromDriveState(this.getState());

        inputs.yawVelocityRadPerSec = Units.degreesToRadians(angularYawVelocity.getValueAsDouble());
        inputs.yawAccelerationRadPerSecPerSec = accelerationZ.getValueAsDouble() * 9.80665; //gravity

        Logger.recordOutput("Drive/Speeds/Desired", getKinematics().toChassisSpeeds(inputs.ModuleTargets));
    }

    @Override
    public void updateModuleInputs(ModuleIOInputs... inputs) {
        for(int i = 0; i < inputs.length; i++) {
            moduleRefreshFunctions.get(i).refreshSignals(inputs[i]);
        }
    }

    private String[][] outNames;

    @Override
    public void logModules(SwerveDriveState state) {
        if(state == null) {
            return;
        }

        if(outNames == null) {
            outNames = new String[4][5];
            for(int i = 0; i < getModules().length; ++i) {
                String module = moduleNames[i];
                outNames[i][0] = module + " Absolute Angle";
                outNames[i][1] = module + " Steering Angle";
                outNames[i][2] = module + " Desired Steering Angle";
                outNames[i][3] = module + " Drive Velocity";
                outNames[i][4] = module + " Desired Drive Velocity";
            }
        }

        for(int i = 0; i < getModules().length; i++) {
            Logger.recordOutput(outNames[i][0], Rotation2d.fromRotations(getModule(i).getEncoder().getAbsolutePosition().getValueAsDouble()));
            Logger.recordOutput(outNames[i][1], state.ModuleStates[1].angle);
            Logger.recordOutput(outNames[i][2], state.ModuleTargets[i].angle);
            Logger.recordOutput(outNames[i][3], state.ModuleStates[i].speedMetersPerSecond);
            Logger.recordOutput(outNames[i][4], state.ModuleTargets[i].speedMetersPerSecond);
        }
    }

    @Override
    public void setControl(SwerveRequest request) {
        super.setControl(request);
    }

    @Override
    public void resetPose(Pose2d pose) {
        super.resetPose(pose);
    }
    

    @Override
    public void resetHeading() {
        this.resetRotation(isRedSide() ? Rotation2d.k180deg : Rotation2d.kZero);
    }

    @Override
    public void setStatorCurrentLimit(double limit) {
        Stream.of(getModules()).map(SwerveModule::getDriveMotor).forEach(motor -> {
            final TalonFXConfiguration config = new TalonFXConfiguration();
            final TalonFXConfigurator configurator = motor.getConfigurator();

            configurator.refresh(config);
            configurator.apply(config.CurrentLimits.withStatorCurrentLimit(limit));
        });
    }

    @Override
    public void setSupplyCurrentLimit(double limit) {
        Stream.of(getModules()).map(SwerveModule::getDriveMotor).forEach(motor -> {
            final TalonFXConfiguration config = new TalonFXConfiguration();
            final TalonFXConfigurator configurator = motor.getConfigurator();

            configurator.refresh(config);
            configurator.apply(config.CurrentLimits.withSupplyCurrentLimit(limit));
        });
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate
     * while still accounting for measurement noise.
     *
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue
     * to apply to future measurements until a subsequent call to {@link
     * #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters    The pose of the robot as measured by the
     *                                 vision camera.
     * @param timestampSeconds         The timestamp of the vision measurement in
     *                                 seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose
     *                                 measurement in the form
     *                                 [x, y, theta]ᵀ, with units in meters and
     *                                 radians.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(
            visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }
}