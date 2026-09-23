package frc.robot.constants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.PhoenixUtil;

public class SubsystemConstants {
    public static class DriveConstants {
        public static final double ODOMETRY_UPDATE_FREQUENCY = 250;
         public static final double DRIVE_BASE_RADIUS = Math.max(
            Math.max(
                Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
                Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
            Math.max(
                Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
                Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));
        public static final double DRIVE_BASE_HYPOTONUSE = 
            new Translation2d(
                TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY)
            .getDistance(new Translation2d(
                TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY));
        public static final Translation2d[] MODULE_TRANSLATIONS =  new Translation2d[] {
            new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
            new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
            new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
            new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
        };
        public static final double DRIVE_BASE_HALF_WIDTH = Math.abs(TunerConstants.FrontLeft.LocationY);
        public static final double ROBOT_MASS_KG = 68.4; 
        public static final double WHEEL_COF = 1.1;
        public static final double MAX_DRIVE_SPEED = TunerConstants.kSpeedAt12Volts.magnitude();
        public static final double MAX_ROTATIONAL_RATE = 8;

        @SuppressWarnings("unchecked")
        public static SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[] MODULE_CONSTANTS = new SwerveModuleConstants[] {
            PhoenixUtil.regulateModuleConstantForSimulation(TunerConstants.FrontLeft),
            PhoenixUtil.regulateModuleConstantForSimulation(TunerConstants.FrontRight),
            PhoenixUtil.regulateModuleConstantForSimulation(TunerConstants.BackLeft),
            PhoenixUtil.regulateModuleConstantForSimulation(TunerConstants.BackRight)
        };
    }

 public static class HoppahConstants {
        public static final double BELT_GEARING = 1.0;
        public static final double BELT_KP = 5;
        public static final double BELT_KD = 0.0;
        public static final double BELT_KS = 0.4;
        public static final double BELT_KV = 0.1243 * BELT_GEARING;
        public static final boolean BELT_INVERTED = true;
    
        public static final double FEEDER_GEARING = 348.0 * 24.0 / (16.0 * 11.0);
        public static final double FEEDER_KP = 300;
        public static final double FEEDER_KD = 0.0;
        public static final double FEEDER_KS = 0.4;
        public static final double FEEDER_KV = 0.1243 * FEEDER_GEARING;
        public static final boolean FEEDER_INVERTED = true;

        public static final double HOPPAH_VOLTAGE = 10.0;
    }
}