// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.constants.FieldConstants.isRedSide;
import static frc.robot.constants.SubsystemConstants.IntakeConstants.*;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.opencv.objdetect.CharucoBoard;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.google.flatbuffers.Constants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.constants.SubsystemConstants.DriveConstants;
import frc.robot.drive.*;
import frc.robot.hopper.*;
import frc.robot.intake.*;
import frc.robot.util.FuelUtil;
import frc.robot.util.bump.Bump;
import frc.robot.util.bump.BumpUtil;
import frc.robot.feeder.*;
import frc.robot.drum.*;

public class RobotContainer {
  Drive drive;
  Hoppah hoppah;
  Feeder feeder;
  Drum drum;
  Intake intake;
  SwerveIOCTRE swerveIO = null;

  //bump stuff
  private static final double BUMP_SPEED_SCALE_MIN = 0.2; // minimum speed multiplier
  private static final double BUMP_SPEED_SCALE_TIME = 0.75; // seconds to reach minimum
  private BumpUtil bump = null;
  private boolean wasOnBump = false;
  private Pose2d bumpVisualizationPose = null;
  private double lastBumpUpdateTime = -1;
  private double bumpTimeSeconds = 0.0;

  private static final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  public static final CommandXboxController joystick = new CommandXboxController(0);

  public RobotContainer() {
    switch (FieldConstants.currentMode) {
      case REAL:
        swerveIO = new SwerveIOCTRE();
        drive = new Drive(swerveIO, 
          new VisionIOPhotonVision(VisionConstants.leftCameraName, VisionConstants.leftCameraTransform)
        );
        hoppah = new Hoppah(new HoppahIOReal());
        feeder = new Feeder(new FeederIOReal());
        drum = new Drum(new DrumIOReal());
        intake = new Intake(new IntakeIOReal());
        break;
      case SIM:
        swerveIO = new SwerveIOSim();
        drive = new Drive(swerveIO,
            new VisionIOPhotonVisionSim(VisionConstants.leftCameraName, VisionConstants.leftCameraTransform, () -> swerveIO.getDriveSim().getSimulatedDriveTrainPose())
        );
        hoppah = new Hoppah(new HoppahIOSim());
        feeder = new Feeder(new FeederIOSim());
        drum = new Drum(new DrumIOSim());
        intake = new Intake(new IntakeIOSim());
        bump = new BumpUtil(() -> swerveIO.getDriveSim().getSimulatedDriveTrainPose(), this::getBumpPose);
        break;
      default:
        drive = new Drive(new SwerveIO() {
        }, new VisionIO() {
        });
        hoppah = new Hoppah(new HoppahIO() {
        });
        feeder = new Feeder(new FeederIO() {
        });
        drum = new Drum(new DrumIO() {
        });
        intake = new Intake(new IntakeIO() {
        });
        break;
    }
    configureBindings();
    setDefaultCommands();

    SmartDashboard.putNumber("RPS", 30);
    SmartDashboard.putNumber("Angle Deg", 10);
  }

  private void setDefaultCommands() {
    drive.setDefaultCommand(drive.joystickDrive(joystick::getLeftX, joystick::getLeftY, joystick::getRightX));
    hoppah.setDefaultCommand(hoppah.defaultCommand());
    feeder.setDefaultCommand(feeder.defaultCommand());
    drum.setDefaultCommand(drum.defaultCommand());
    intake.setDefaultCommand(intake.defaultCommand());
  }

  private void configureBindings() {
    /* Reset Heading */ joystick.b().onTrue(drive.runOnce(swerveIO::seedFieldCentric));
    /* Intake */ joystick.leftTrigger().whileTrue(intake.setIntake(ROLLER_INTAKE_RPS, PIVOT_DEPLOYED_ROTATIONS));
    joystick.rightTrigger().whileTrue(drum.setDrumState(100.0, Rotation2d.fromDegrees(30)));
    joystick.rightBumper().whileTrue(feeder.feedRun(50)); // Changed to negative as a temp fix to cause to spin in the
                                                           // correct direction --mitchell
    joystick.a().whileTrue(hoppah.setHoppahState(50, 50));
    joystick.x().whileTrue(drum.setDrumState(SmartDashboard.getNumber("RPS", 0), Rotation2d.fromDegrees(SmartDashboard.getNumber("Angle Deg", 0))));
  }

    private Pose2d getBumpPose() {
        if (bump != null && bump.isOnBump()) {
            wasOnBump = true;
            double now = Timer.getFPGATimestamp();

            if (bumpVisualizationPose == null) {
                bumpVisualizationPose = swerveIO.getDriveSim().getSimulatedDriveTrainPose();
                lastBumpUpdateTime = now;
                bumpTimeSeconds = 0.0;
                return bumpVisualizationPose;
            }

            double dt = now - lastBumpUpdateTime;
            lastBumpUpdateTime = now;

            boolean isOnPhysicalBump = false;
            for (Bump b : Bump.BUMPS) {
                if (b.isInPhysicalBump(swerveIO.getDriveSim().getSimulatedDriveTrainPose())) {
                    isOnPhysicalBump = true;
                    break;
                }
            }

            if (isOnPhysicalBump) {
                bumpTimeSeconds += dt;
            }

            double speedScale = isOnPhysicalBump ? Math.max(
                BUMP_SPEED_SCALE_MIN,
                1.0 - (bumpTimeSeconds / BUMP_SPEED_SCALE_TIME) * (1.0 - BUMP_SPEED_SCALE_MIN)
            ) : 1.0;

            ChassisSpeeds speeds = new ChassisSpeeds(isRedSide() ? -joystick.getLeftY() * TunerConstants.kSpeedAt12Volts.magnitude() : joystick.getLeftY() * TunerConstants.kSpeedAt12Volts.magnitude(), isRedSide() ? joystick.getLeftX() * TunerConstants.kSpeedAt12Volts.magnitude(): -joystick.getLeftX() * TunerConstants.kSpeedAt12Volts.magnitude(), isRedSide() ? joystick.getRightX() *DriveConstants.MAX_ROTATIONAL_RATE : -joystick.getRightX() * DriveConstants.MAX_ROTATIONAL_RATE); //desired

            double newX = bumpVisualizationPose.getX() + speeds.vxMetersPerSecond * dt * speedScale;
            double newY = bumpVisualizationPose.getY() + speeds.vyMetersPerSecond * dt * speedScale;

            for (Bump b : Bump.BUMPS) {
                if (b.isInBump(bumpVisualizationPose)) {
                    double minY = Math.min(b.getCorner1().getY(), b.getCorner2().getY());
                    double maxY = Math.max(b.getCorner1().getY(), b.getCorner2().getY());
                    newY = MathUtil.clamp(newY, minY, maxY);
                    break;
                }
            }

            Rotation2d newHeading = bumpVisualizationPose.getRotation()
                .plus(Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * dt));

            bumpVisualizationPose = new Pose2d(newX, newY, newHeading);
            return bumpVisualizationPose;
        }

        bumpVisualizationPose = null;
        lastBumpUpdateTime = -1;
        bumpTimeSeconds = 0.0;
        return swerveIO.getDriveSim().getSimulatedDriveTrainPose();
    }

    public void updateSimulation() {
        if(Robot.isReal()) return;

        Pose3d bumpRobotPose = bump.bumpVisualizePose();

        if (bump.isOnBump() && bumpVisualizationPose != null) {
            for (Bump b : Bump.BUMPS) {
                if (b.isInBump(bumpVisualizationPose)) {
                    double minY = Math.min(b.getCorner1().getY(), b.getCorner2().getY()) + 0.065;
                    double maxY = Math.max(b.getCorner1().getY(), b.getCorner2().getY()) - 0.06;
                    double clampedY = MathUtil.clamp(bumpVisualizationPose.getY(), minY, maxY);
                    bumpVisualizationPose = new Pose2d(
                        bumpVisualizationPose.getX(), 
                        clampedY, 
                        bumpVisualizationPose.getRotation());
                    swerveIO.getDriveSim().setSimulationWorldPose(bumpVisualizationPose);
                    break;
                }
            }
        }

        bump.updateChecks(bumpVisualizationPose);

        SimulatedArena.getInstance().simulationPeriodic();

        for(Bump b : Bump.BUMPS) {
            if(wasOnBump && !b.isInBump(bumpRobotPose.toPose2d())) {
                swerveIO.getDriveSim().setSimulationWorldPose(bumpRobotPose.toPose2d());
                wasOnBump = false;
                break;
            }
        }
        
        // Pose3d[] robotFuel = FuelUtil.getFuelPosesFromRobot(bumpRobotPose, intake.getFuelCount(), intake.getExtendDistance());

        Logger.recordOutput("Sim/Robot/Real Sim Pose", bumpRobotPose);
        // Logger.recordOutput("Simulation/Fuel/Fuel in robot", robotFuel);
        // Logger.recordOutput("Simulation/Fuel/Fuel on field", fuel);
        // Logger.recordOutput("Simulation/Fuel/Fuel on fly", LaunchCalculator.simulatePoses(bumpRobotPose, shooter.getCurrentLaunchParameters()));
    }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
