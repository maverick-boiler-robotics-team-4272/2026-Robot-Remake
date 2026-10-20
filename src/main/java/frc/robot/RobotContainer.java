// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.constants.SubsystemConstants.IntakeConstants.*;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.FieldConstants;
import frc.robot.drive.*;
import frc.robot.hopper.*;
import frc.robot.intake.*;
import frc.robot.feeder.*;
import frc.robot.drum.*;

public class RobotContainer {
  Drive drive;
  Hoppah hoppah;
  Feeder feeder;
  Drum drum;
  Intake intake;
  SwerveIOCTRE swerveIO = null;

  private static final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  public static final CommandXboxController joystick = new CommandXboxController(0);

  public RobotContainer() {
    switch (FieldConstants.currentMode) {
      case REAL:
        swerveIO = new SwerveIOCTRE();
        drive = new Drive(swerveIO, 
          new VisionIOPhotonVision(VisionConstants.leftCameraName, VisionConstants.leftCameraTransform),
          new VisionIOPhotonVision(VisionConstants.rightCameraName, VisionConstants.rightCameraTransform)
        );
        hoppah = new Hoppah(new HoppahIOReal());
        feeder = new Feeder(new FeederIOReal());
        drum = new Drum(new DrumIOReal());
        intake = new Intake(new IntakeIOReal());
        break;
      case SIM:
        swerveIO = new SwerveIOSim();
        drive = new Drive(swerveIO,
            new VisionIOPhotonVisionSim(VisionConstants.leftCameraName, VisionConstants.leftCameraTransform, () -> swerveIO.getDriveSim().getSimulatedDriveTrainPose()),
            new VisionIOPhotonVisionSim(VisionConstants.rightCameraName, VisionConstants.rightCameraTransform, () -> swerveIO.getDriveSim().getSimulatedDriveTrainPose())
        );
        hoppah = new Hoppah(new HoppahIOSim());
        feeder = new Feeder(new FeederIOSim());
        drum = new Drum(new DrumIOSim());
        intake = new Intake(new IntakeIOSim());
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

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
