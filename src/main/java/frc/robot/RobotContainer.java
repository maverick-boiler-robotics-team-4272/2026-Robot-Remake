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

public class RobotContainer {
  Drive drive;
  Hoppah hoppah;
  SwerveIOCTRE realIO = null;
  public static final CommandXboxController joystick = new CommandXboxController(0);
  public RobotContainer() {
    switch (FieldConstants.currentMode) {
      case REAL:
        realIO = new SwerveIOCTRE();
        drive = new Drive(realIO, new VisionIOPhotonVision("Limelight", VisionConstants.robotToCamera0));
        hoppah = new Hoppah(new HoppahIOReal());
        break;
      case SIM :
        drive = new Drive(new SwerveIOSim(),  new VisionIOPhotonVisionSim("Limelight", VisionConstants.robotToCamera0, () -> drive.getState().Pose));
        hoppah = new Hoppah(new HoppahIOSim());
        break;
      default:
        drive = new Drive(new SwerveIO() {}, new VisionIO() {});
        hoppah = new Hoppah(new HoppahIO() {});
        break;
    }
    configureBindings();
  }

  private void configureBindings() {
    drive.setDefaultCommand(drive.joystickDrive(joystick::getLeftX, joystick::getLeftY, joystick::getRightX));
    joystick.b().onTrue(drive.runOnce(realIO::seedFieldCentric));
    joystick.a().onTrue(hoppah.hopRun());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
