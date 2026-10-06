// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Newton;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
  SwerveIOCTRE swerveIO = null;
  public static final CommandXboxController joystick = new CommandXboxController(0);
  public RobotContainer() {
    switch (FieldConstants.currentMode) {
      case REAL:
        swerveIO = new SwerveIOCTRE();
        drive = new Drive(swerveIO, new VisionIOPhotonVision("Limelight", VisionConstants.robotToCamera0));
        hoppah = new Hoppah(new HoppahIOReal());
        feeder = new Feeder(new FeederIOReal());
        drum = new Drum(new DrumIOReal());
        break;
      case SIM :
        swerveIO =  new SwerveIOSim();
        drive = new Drive(swerveIO,  new VisionIOPhotonVisionSim("Limelight", VisionConstants.robotToCamera0, () -> drive.getState().Pose));
      default:
        drive = new Drive(new SwerveIO() {}, new VisionIO() {});
        hoppah = new Hoppah(new HoppahIO() {});
        feeder = new Feeder(new FeederIO() {});
        drum = new Drum(new DrumIO() {});
        break;
    }
    configureBindings();
  }

  private void configureBindings() {
    drive.setDefaultCommand(drive.joystickDrive(joystick::getLeftX, joystick::getLeftY, joystick::getRightX));
    joystick.b().onTrue(drive.runOnce(swerveIO::seedFieldCentric));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
