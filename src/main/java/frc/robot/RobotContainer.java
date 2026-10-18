// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Newton;
import static edu.wpi.first.units.Units.derive;
import static frc.robot.constants.SubsystemConstants.IntakeConstants.*;
import static frc.robot.drive.VisionConstants.camera0Name;
import static frc.robot.drive.VisionConstants.robotToCamera0;

import java.io.IOException;
import java.text.ParseException;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import static frc.robot.constants.FieldConstants.getHubLocation;
import static frc.robot.constants.SubsystemConstants.FeederConstants.*;
import static frc.robot.constants.SubsystemConstants.HoppahConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.FieldConstants;
import frc.robot.drive.Drive;
import frc.robot.drive.SwerveIO;
import frc.robot.drive.SwerveIOCTRE;
import frc.robot.drive.SwerveIOSim;
import frc.robot.drive.VisionConstants;
import frc.robot.drive.VisionIO;
import frc.robot.drive.VisionIOPhotonVision;
import frc.robot.drive.VisionIOPhotonVisionSim;
import frc.robot.hopper.HoppahIO;
import frc.robot.hopper.Hoppah;
import frc.robot.hopper.HoppahIOReal;
import frc.robot.hopper.HoppahIOSim;
import frc.robot.intake.Intake;
import frc.robot.intake.IntakeIO;
import frc.robot.intake.IntakeIOReal;
import frc.robot.intake.IntakeIOSim;
import frc.robot.feeder.Feeder;
import frc.robot.feeder.FeederIO;
import frc.robot.feeder.FeederIOReal;
import frc.robot.feeder.FeederIOSim;
import frc.robot.drum.Drum;
import frc.robot.drum.DrumIO;
import frc.robot.drum.DrumIOReal;
import frc.robot.drum.DrumIOSim;

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
        drive = new Drive(swerveIO);
        hoppah = new Hoppah(new HoppahIOReal());
        feeder = new Feeder(new FeederIOReal());
        drum = new Drum(new DrumIOReal());
        intake = new Intake(new IntakeIOReal());
        break;
      case SIM :
        swerveIO =  new SwerveIOSim();
        drive = new Drive(swerveIO, new VisionIOPhotonVision(camera0Name, robotToCamera0));
        hoppah = new Hoppah(new HoppahIOSim());
        feeder = new Feeder(new FeederIOSim());
        drum = new Drum(new DrumIOSim());
        intake = new Intake(new IntakeIOSim());
        break;
      default:
        drive = new Drive(new SwerveIO() {}, new VisionIO() {});
        hoppah = new Hoppah(new HoppahIO() {});
        feeder = new Feeder(new FeederIO() {});
        drum = new Drum(new DrumIO() {});
        intake = new Intake(new IntakeIO() {});
        break;
    }
    configureBindings();
    setDefaultCommands();
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
    /* Shoot */ joystick.rightTrigger().whileTrue(new ParallelCommandGroup(
      drum.setDrumState(40, Rotation2d.fromDegrees(20)),
      feeder.feedRun(FEEDER_RPS),
      hoppah.hopRun(),
      drive.applyRequest(() -> brake)
    ));
    /* Rev */ joystick.rightBumper().whileTrue(new ParallelCommandGroup(
      drum.setDrumState(40, Rotation2d.fromDegrees(20)),
      feeder.feedRun(-FEEDER_RPS * 0.3),
      hoppah.setHoppahState(-HOPPAH_RPS * 0.75, HOPPAH_RPS),
      drive.pointTowardsPoint(() -> getHubLocation().getTranslation(), joystick::getLeftX, joystick::getLeftY)
    ));
  }

  public Command getAutonomousCommand(){
    try {
      // Load the named auto so its resetOdom setting and any auto-level commands run.
      return new PathPlannerAuto("AUTO");
    } catch (Exception ex) {
      return Commands.print("Big oops: " + ex.getMessage());
    }
  }
}
