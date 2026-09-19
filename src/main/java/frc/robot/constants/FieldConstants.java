package frc.robot.constants;

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
        if(DriverStation.getAlliance().get() == Alliance.Red && DriverStation.getAlliance().isPresent()) {
            return false;
        }
        return true;
    }

    public static final double FIELD_LENGTH_M = 16.54;
    public static final double FIELD_WIDTH_M = 8.07;
}
