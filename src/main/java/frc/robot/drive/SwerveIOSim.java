package frc.robot.drive;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.google.flatbuffers.Constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.constants.TunerConstants;
import frc.robot.util.PhoenixUtil;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.SubsystemConstants.DriveConstants.*;

import java.util.function.Consumer;

public class SwerveIOSim extends SwerveIOCTRE {
    private SwerveDriveSimulation drivetrainSim;

    public SwerveIOSim() {
        super();
        startSimThread();
    }

    private void startSimThread() {
        drivetrainSim = new SwerveDriveSimulation(DriveTrainSimulationConfig.Default()
        .withRobotMass(Kilograms.of(ROBOT_MASS_KG))
        .withCustomModuleTranslations(MODULE_TRANSLATIONS)
        .withGyro(COTS.ofPigeon2())
        // TODO: .withBumperSize(null, null)
        .withSwerveModule(new SwerveModuleSimulationConfig(
            DCMotor.getKrakenX60Foc(1),
            DCMotor.getKrakenX44Foc(1),
            TunerConstants.FrontLeft.DriveMotorGearRatio, 
            TunerConstants.FrontLeft.SteerMotorGearRatio, 
            Volts.of(TunerConstants.FrontLeft.DriveFrictionVoltage), 
            Volts.of(TunerConstants.FrontLeft.SteerFrictionVoltage), 
            Meters.of(TunerConstants.FrontLeft.WheelRadius), 
            KilogramSquareMeters.of(TunerConstants.FrontLeft.SteerInertia), 
            WHEEL_COF)),
            new Pose2d(1, 1, Rotation2d.kZero));
        for(int i = 0; i < drivetrainSim.getModules().length; i++) {
            drivetrainSim.getModules()[i].useDriveMotorController(new PhoenixUtil.TalonFXMotorControllerSim(getModule(i).getDriveMotor()));
            drivetrainSim.getModules()[i].useSteerMotorController(new PhoenixUtil.TalonFXMotorControllerWithRemoteCancoderSim(getModule(i).getSteerMotor(), getModule(i).getEncoder()));
        }
        SimulatedArena.getInstance().addDriveTrainSimulation(drivetrainSim);
    }

    @Override
    public void resetPose(Pose2d pose) {
        drivetrainSim.setSimulationWorldPose(pose);
        super.resetPose(pose);
    }

    @Override
    public void updateSwerveInputs(SwerveIOInputs inputs) {
        super.updateSwerveInputs(inputs);
        updateGyroSim();
        Pose2d realRobotPose = drivetrainSim.getSimulatedDriveTrainPose();
        inputs.Pose = realRobotPose;

        Logger.recordOutput("Sim/Drive/RealPose", realRobotPose);
    }

    public SwerveDriveSimulation getDriveSim() {
        return drivetrainSim;
    }

    private void updateGyroSim() {
        Pigeon2SimState pigeonSim = getPigeon2().getSimState();
        pigeonSim.setRawYaw(drivetrainSim.getSimulatedDriveTrainPose().getRotation().getDegrees());
        pigeonSim.setAngularVelocityZ(
            edu.wpi.first.math.util.Units.radiansToDegrees(drivetrainSim.getDriveTrainSimulatedChassisSpeedsFieldRelative().omegaRadiansPerSecond)
        );
    }
}
