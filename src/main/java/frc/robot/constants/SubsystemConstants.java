package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.util.PhoenixUtil;

public class SubsystemConstants {
    public static class IntakeConstants {
        // TODO: all values below are placeholders, replace with the real robot values.
        public static final CANBus CAN_BUS = new CANBus(""); // "" = roboRIO bus

        public static final int ROLLER_LEADER_ID = 20;
        public static final int ROLLER_FOLLOWER_ID = 21;
        public static final int PIVOT_ID = 22;

        /** Number of roller motors (leader + follower). */
        public static final int ROLLER_MOTOR_COUNT = 2;

        // Motor model used by the sim, chosen per mechanism. Swap to getKrakenX44Foc(...) if a
        // mechanism uses X44s. The real IO is identical for both (TalonFX).
        public static final DCMotor ROLLER_MOTORS = DCMotor.getKrakenX60Foc(ROLLER_MOTOR_COUNT);
        public static final DCMotor PIVOT_MOTOR = DCMotor.getKrakenX60Foc(1);

        /** Motor rotations per mechanism rotation. */
        public static final double ROLLER_GEAR_RATIO = 1.0;
        public static final double PIVOT_GEAR_RATIO = 1.0;

        public static final boolean ROLLER_INVERTED = false;
        /** True if the follower spins opposite the leader (motors mounted facing each other). */
        public static final boolean ROLLER_FOLLOWER_OPPOSED = true;
        public static final boolean PIVOT_INVERTED = false;

        /** Pivot software limits, in mechanism rotations (after PIVOT_GEAR_RATIO). Motor output is cut past these. */
        public static final double PIVOT_FORWARD_LIMIT_ROTATIONS = 0.30;
        public static final double PIVOT_REVERSE_LIMIT_ROTATIONS = -0.02;

        public static final double ROLLER_STATOR_LIMIT_AMPS = 60.0;
        public static final double ROLLER_SUPPLY_LIMIT_AMPS = 40.0;
        public static final double PIVOT_STATOR_LIMIT_AMPS = 40.0;
        public static final double PIVOT_SUPPLY_LIMIT_AMPS = 30.0;

        // Roller velocity loop (units: volts per rotation/sec)
        public static final double ROLLER_KP = 0.1;
        public static final double ROLLER_KV = 0.12;

        // Pivot position loop (PositionVoltage; units: volts per rotation, volts per rotation/sec)
        public static final double PIVOT_KP = 30.0;
        public static final double PIVOT_KD = 0.5;

        /** Pivot positions in mechanism rotations; must sit inside the soft limits above. */
        public static final double PIVOT_STOWED_ROTATIONS = 0.0;
        public static final double PIVOT_DEPLOYED_ROTATIONS = 0.25;
        public static final double PIVOT_TOLERANCE_ROTATIONS = 0.02;

        /** Roller speeds in rotations/sec; negative ejects. */
        public static final double ROLLER_INTAKE_RPS = 50.0;
        public static final double ROLLER_EJECT_RPS = -50.0;
    }

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
}
