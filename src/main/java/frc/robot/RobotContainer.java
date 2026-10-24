// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.constants.FieldConstants.isRedSide;
import static frc.robot.constants.SubsystemConstants.FeederConstants.FEEDER_RPS;
import static frc.robot.constants.SubsystemConstants.HoppahConstants.HOPPAH_RPS;
import static frc.robot.constants.SubsystemConstants.IntakeConstants.ROLLER_INTAKE_RPS;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.RobotCommands;
import frc.robot.constants.FieldConstants;
import frc.robot.drive.Drive;
import frc.robot.drive.SwerveIO;
import frc.robot.drive.SwerveIOCTRE;
import frc.robot.drive.SwerveIOSim;
import frc.robot.drive.VisionConstants;
import frc.robot.drive.VisionIO;
import frc.robot.drive.VisionIOPhotonVision;
import frc.robot.drive.VisionIOPhotonVisionSim;
import frc.robot.drum.Drum;
import frc.robot.drum.DrumIO;
import frc.robot.drum.DrumIOReal;
import frc.robot.drum.DrumIOSim;
import frc.robot.feeder.Feeder;
import frc.robot.feeder.FeederIO;
import frc.robot.feeder.FeederIOReal;
import frc.robot.feeder.FeederIOSim;
import frc.robot.hopper.Hoppah;
import frc.robot.hopper.HoppahIO;
import frc.robot.hopper.HoppahIOReal;
import frc.robot.hopper.HoppahIOSim;
import frc.robot.intake.Intake;
import frc.robot.intake.IntakeIO;
import frc.robot.intake.IntakeIOReal;
import frc.robot.intake.IntakeIOSim;

public class RobotContainer {
  Drive drive;
  Hoppah hoppah;
  Feeder feeder;
  Drum drum;
  Intake intake;
  SwerveIOCTRE swerveIO = null;

  private SendableChooser<PathPlannerAuto> autChooser;

  private static final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  public static final CommandXboxController joystick = new CommandXboxController(0);
  public static final CommandXboxController operator = new CommandXboxController(1);

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
    setDefaultCommands();
    configureDriverBindings();
    configureOperatorBindings();
    registeredNamedCommands();
    setUpAutos();

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

  
  private void configureDriverBindings() {
    /* Reset Heading */ joystick.b().onTrue(drive.runOnce(() -> swerveIO.seedFieldCentric()));
    /* Intake */ joystick.leftTrigger().whileTrue(intake.setIntake(ROLLER_INTAKE_RPS));
    joystick.leftBumper().whileTrue(intake.setIntake(-ROLLER_INTAKE_RPS));
    /* Shoot Shooter */ joystick.rightTrigger().whileTrue(RobotCommands.revScoreEditCommand(drive, drum, feeder, hoppah,
        intake, joystick::getLeftX, joystick::getLeftY, joystick.a()));
        joystick.x().whileTrue(RobotCommands.revScoreCommand(drive, drum, feeder, hoppah, intake, joystick::getLeftX, joystick::getLeftY, joystick.a()));
    joystick.rightBumper().whileTrue(RobotCommands.revShuttleCommand(drive, drum, feeder, hoppah, intake,
        joystick::getLeftX, joystick::getLeftY, joystick.a()));
    joystick.povLeft().whileTrue(drum.resetHood());
    joystick.y().whileTrue(drive.applyRequest(() -> brake));
  }

  private void configureOperatorBindings() {
    operator.rightTrigger().whileTrue(
        drum.setDrumState(44.3, 15));
    operator.rightBumper().whileTrue(drum.setDrumState(100, 30));
    operator.a().whileTrue(new ParallelCommandGroup(
        feeder.feedRun(FEEDER_RPS),
        hoppah.hopRun(),
        intake.setIntake(ROLLER_INTAKE_RPS)));
    // operator.povDownLeft().whileTrue(drum.resetHood());
    operator.povUp().onTrue(RobotCommands.addAngleAdd());
    operator.povDown().onTrue(RobotCommands.subtractAngleAdd());
    operator.povLeft().onTrue(RobotCommands.subtractShooterAdd());
    operator.povRight().onTrue(RobotCommands.addShooterAdd());
    operator.y().whileTrue(hoppah.setHoppahState(-HOPPAH_RPS, -HOPPAH_RPS * 0.5));
  }

  private void registeredNamedCommands() {
    NamedCommands.registerCommand("intake", intake.setIntake(ROLLER_INTAKE_RPS));
    NamedCommands.registerCommand("shoot",
        RobotCommands.autoScoreCommand(drive, drum, feeder, hoppah, intake, 0.5, 2)
    );
  }

  private void setUpAutos() {
    autChooser = new SendableChooser<PathPlannerAuto>();
    autChooser.setDefaultOption("2.5 Left Close", new PathPlannerAuto("2.5 pass close", false));
    autChooser.addOption("2.5 Left Far", new PathPlannerAuto("2.5 pass far", false));
    autChooser.addOption("2.5 Right Close", new PathPlannerAuto("2.5 pass close", true));
    autChooser.addOption("2.5 Right Far", new PathPlannerAuto("2.5 pass far", true));

    SmartDashboard.putData("Auto", autChooser);
  }

  public Command getAutonomousCommand(){
    try {
      return autChooser.getSelected();
    } catch (Exception ex) {
      return Commands.print("Big oops: " + ex.getMessage());
    }
  }
}
