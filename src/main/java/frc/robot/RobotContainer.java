// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

public class RobotContainer {
  Drive drive;
  SwerveIOCTRE realIO = null;
  public static final CommandXboxController joystick = new CommandXboxController(0);
  public RobotContainer() {
    switch (FieldConstants.currentMode) {
      case REAL:
        realIO = new SwerveIOCTRE();
        drive = new Drive(realIO, new VisionIOPhotonVision("Limelight", VisionConstants.robotToCamera0));
        break;
      case SIM :
        drive = new Drive(new SwerveIOSim(),  new VisionIOPhotonVisionSim("Limelight", VisionConstants.robotToCamera0, () -> drive.getState().Pose));
    
      default:
        drive = new Drive(new SwerveIO() {}, new VisionIO() {});
        break;
    }
    configureBindings();
  }

  private void configureBindings() {
    drive.setDefaultCommand(drive.joystickDrive(joystick::getLeftX, joystick::getLeftY, joystick::getRightX));
    joystick.b().onTrue(drive.runOnce(realIO::seedFieldCentric));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
