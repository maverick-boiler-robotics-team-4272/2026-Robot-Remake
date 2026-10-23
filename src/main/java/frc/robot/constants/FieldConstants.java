package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;

public class FieldConstants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static boolean isRedSide() {
        return DriverStation.getAlliance()
                .map(alliance -> alliance == Alliance.Red)
                .orElse(false);
    }

    public static final double FIELD_LENGTH_M = 16.54;
    public static final double FIELD_WIDTH_M = 8.07;

    
    public static final Pose2d RED_HUB_POSE = new Pose2d(FIELD_LENGTH_M - 4.6, 4.035, Rotation2d.kZero);
    public static final Pose2d BLUE_HUB_POSE = new Pose2d(4.6, 4.035, Rotation2d.kZero);

    public static Pose2d getHubLocation() {
            return isRedSide() ? RED_HUB_POSE : BLUE_HUB_POSE;
    }
}
