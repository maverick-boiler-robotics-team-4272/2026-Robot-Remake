package frc.robot.commands;

import static frc.robot.constants.FieldConstants.getHubLocation;
import static frc.robot.constants.SubsystemConstants.DrumConstants.ANGLE_DEG_MAP;
import static frc.robot.constants.SubsystemConstants.DrumConstants.SPEED_MAP;
import static frc.robot.constants.SubsystemConstants.FeederConstants.FEEDER_RPS;
import static frc.robot.constants.SubsystemConstants.HoppahConstants.HOPPAH_RPS;
import static frc.robot.constants.SubsystemConstants.IntakeConstants.ROLLER_INTAKE_RPS;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.drive.Drive;
import frc.robot.drum.Drum;
import frc.robot.feeder.Feeder;
import frc.robot.hopper.Hoppah;
import frc.robot.intake.Intake;

public class RobotCommands {
    private static double shooterAdd = 0;
    private static double angleAdd = 0;

    public static Command revScoreCommand(Drive drive, Drum drum, Feeder feeder, Hoppah hop, Intake intake, DoubleSupplier joystickX, DoubleSupplier joystickY, Trigger trigger) {
        return new ParallelCommandGroup(
            drum.setDrumState(() -> SPEED_MAP.get(drive.getDistanceToPose(getHubLocation()).getAsDouble()), () -> ANGLE_DEG_MAP.get(drive.getDistanceToPose(getHubLocation()).getAsDouble())),
            new SequentialCommandGroup(
                new ParallelRaceGroup(
                    drive.pointTowardsPointInAuto(() -> getHubLocation().getTranslation(), joystickX, joystickY),
                    feeder.feedRun(-FEEDER_RPS),
                    hop.setHoppahState(-HOPPAH_RPS, -HOPPAH_RPS),
                    new WaitUntilCommand(trigger)
                ),
                new ParallelCommandGroup(
                    feeder.feedRun(FEEDER_RPS),
                    hop.hopRun(),
                    intake.setIntake(ROLLER_INTAKE_RPS),
                    drive.pointTowardsPointInAuto(() -> getHubLocation().getTranslation(), joystickX, joystickY)
                )
                
            )
        );
    }

    public static Command revScoreEditCommand(Drive drive, Drum drum, Feeder feeder, Hoppah hop, Intake intake, DoubleSupplier joystickX, DoubleSupplier joystickY, Trigger trigger) {
        return new ParallelCommandGroup(
            Commands.repeatingSequence(
                drum.setDrumState(() -> SPEED_MAP.get(drive.getDistanceToPose(getHubLocation()).getAsDouble()) + getShooterAdd().getAsDouble() , () -> ANGLE_DEG_MAP.get(drive.getDistanceToPose(getHubLocation()).getAsDouble()) + getAngleAdd().getAsDouble())
            ),
            new SequentialCommandGroup(
                new ParallelRaceGroup(
                    drive.pointTowardsPoint(() -> getHubLocation().getTranslation(), joystickX, joystickY),
                    feeder.feedRun(-FEEDER_RPS),
                    hop.setHoppahState(-HOPPAH_RPS, -HOPPAH_RPS),
                    new WaitUntilCommand(trigger)
                ),
                new ParallelCommandGroup(
                    feeder.feedRun(FEEDER_RPS),
                    hop.hopRun(),
                    intake.setIntake(ROLLER_INTAKE_RPS),
                    drive.pointTowardsPoint(() -> getHubLocation().getTranslation(), joystickX, joystickY)
                )
                
            )
        );
    }

    public static Command autoScoreCommand(Drive drive, Drum drum, Feeder feeder, Hoppah hop, Intake intake, double revTime, double shootTime) {
        return new ParallelRaceGroup(
            drum.setDrumState(() -> SPEED_MAP.get(drive.getDistanceToPose(getHubLocation()).getAsDouble()), () -> ANGLE_DEG_MAP.get(drive.getDistanceToPose(getHubLocation()).getAsDouble())),
            new SequentialCommandGroup(
                new ParallelRaceGroup(
                    feeder.feedRun(-FEEDER_RPS),
                    hop.setHoppahState(-HOPPAH_RPS, -HOPPAH_RPS),
                    drive.pointTowardsPointInAuto(() -> getHubLocation().getTranslation(), () -> 0, () -> 0),
                    new WaitCommand(revTime)
                ),
                new ParallelRaceGroup(
                    new ParallelCommandGroup(
                        feeder.feedRun(FEEDER_RPS),
                        hop.hopRun(),
                        intake.setIntake(ROLLER_INTAKE_RPS),
                        drive.pointTowardsPointInAuto(() -> getHubLocation().getTranslation(), () -> 0, () -> 0)
                    ),
                    new WaitCommand(shootTime)
                )
            )
        );
    }

    public static Command revShuttleCommand(Drive drive, Drum drum, Feeder feeder, Hoppah hop, Intake intake, DoubleSupplier joystickX, DoubleSupplier joystickY, Trigger trigger) {
        return new ParallelCommandGroup(
            drum.setDrumState(100,30),
            new SequentialCommandGroup(
                new ParallelRaceGroup(
                    drive.pointToAllianceWall(joystickX, joystickY),
                    feeder.feedRun(-FEEDER_RPS),
                    hop.setHoppahState(-HOPPAH_RPS, -HOPPAH_RPS),
                    new WaitUntilCommand(trigger)
                ),
                new ParallelCommandGroup(
                    feeder.feedRun(FEEDER_RPS),
                    hop.hopRun(),
                    intake.setIntake(ROLLER_INTAKE_RPS),
                    drive.pointToAllianceWall(joystickX, joystickY)
                )
            )
        );
    }

        public static Command addShooterAdd() {
            return Commands.runOnce(() -> shooterAdd += 0.5);
        }

        public static Command subtractShooterAdd() {
            return Commands.runOnce(() -> shooterAdd -= 0.5);
        }

        public static DoubleSupplier getShooterAdd() {
            return () -> shooterAdd;
        }

        public static Command addAngleAdd() {
            return Commands.runOnce(() -> angleAdd += 1);
        }

        public static Command subtractAngleAdd() {
            return Commands.runOnce(() -> angleAdd -= 1);
        }

        public static DoubleSupplier getAngleAdd() {
            return ()-> angleAdd;
        }
}
