package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.constants.SubsystemConstants.IntakeConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
    private static final double LOOP_PERIOD_SECS = 0.02;

    private static final double ROLLER_MOI = 0.002;
    private static final double PIVOT_MOI = 0.05;

    private final DCMotorSim rollerSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(ROLLER_MOTORS, ROLLER_MOI, ROLLER_GEAR_RATIO), ROLLER_MOTORS);
    private final DCMotorSim pivotSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(PIVOT_MOTOR, PIVOT_MOI, PIVOT_GEAR_RATIO), PIVOT_MOTOR);

    private final PIDController rollerController = new PIDController(ROLLER_KP, 0.0, 0.0);
    private final PIDController pivotController = new PIDController(PIVOT_KP, 0.0, PIVOT_KD);

    private double rollerSetpointRps = 0.0;
    private double pivotSetpointRotations = 0.0;

    private double rollerAppliedVolts = 0.0;
    private double pivotAppliedVolts = 0.0;

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        double rollerRps = Units.radiansToRotations(rollerSim.getAngularVelocityRadPerSec());
        double pivotRotations = pivotSim.getAngularPosition().in(Rotations);
        double rollerVolts = ROLLER_KV * rollerSetpointRps + rollerController.calculate(rollerRps, rollerSetpointRps);
        double pivotVolts = pivotController.calculate(pivotRotations, pivotSetpointRotations);

        boolean disabled = DriverStation.isDisabled();
        rollerAppliedVolts = disabled ? 0.0 : MathUtil.clamp(rollerVolts, -12.0, 12.0);
        pivotAppliedVolts = disabled ? 0.0 : MathUtil.clamp(pivotVolts, -12.0, 12.0);

        rollerSim.setInputVoltage(rollerAppliedVolts);
        pivotSim.setInputVoltage(pivotAppliedVolts);
        rollerSim.update(LOOP_PERIOD_SECS);
        pivotSim.update(LOOP_PERIOD_SECS);
        enforcePivotLimits();

        double rollerPerMotorAmps = rollerSim.getCurrentDrawAmps() / ROLLER_MOTOR_COUNT;

        inputs.rollerConnected = true;
        inputs.rollerVelocityRadPerSec = rollerSim.getAngularVelocityRadPerSec();
        inputs.rollerAppliedVolts = rollerAppliedVolts;
        inputs.rollerStatorCurrentAmps = rollerPerMotorAmps;
        inputs.rollerSupplyCurrentAmps = rollerPerMotorAmps;
        inputs.rollerTempCelsius = 0.0;

        inputs.rollerFollowerConnected = true;
        inputs.rollerFollowerVelocityRadPerSec = rollerSim.getAngularVelocityRadPerSec();
        inputs.rollerFollowerAppliedVolts = rollerAppliedVolts;
        inputs.rollerFollowerStatorCurrentAmps = rollerPerMotorAmps;
        inputs.rollerFollowerSupplyCurrentAmps = rollerPerMotorAmps;
        inputs.rollerFollowerTempCelsius = 0.0;

        inputs.pivotConnected = true;
        inputs.pivotPosition = new Rotation2d(pivotSim.getAngularPosition());
        inputs.pivotVelocityRadPerSec = pivotSim.getAngularVelocityRadPerSec();
        inputs.pivotAppliedVolts = pivotAppliedVolts;
        inputs.pivotStatorCurrentAmps = pivotSim.getCurrentDrawAmps();
        inputs.pivotSupplyCurrentAmps = pivotSim.getCurrentDrawAmps();
        inputs.pivotTempCelsius = 0.0;
    }

    /** Mimics the TalonFX soft limits: stop the pivot at the limit and kill velocity pushing past it. */
    private void enforcePivotLimits() {
        double position = pivotSim.getAngularPosition().in(Rotations);
        double velocity = pivotSim.getAngularVelocityRadPerSec();
        double clamped = MathUtil.clamp(position, PIVOT_REVERSE_LIMIT_ROTATIONS, PIVOT_FORWARD_LIMIT_ROTATIONS);
        if (clamped != position) {
            boolean pushingPast = (position > clamped) == (velocity > 0.0);
            pivotSim.setState(Units.rotationsToRadians(clamped), pushingPast ? 0.0 : velocity);
        }
    }

    @Override
    public void setRollerVelocity(double rotationsPerSec) {
        rollerSetpointRps = rotationsPerSec;
    }

    @Override
    public void setPivotPosition(double rotations) {
        pivotSetpointRotations = rotations;
    }

    @Override
    public void resetPivotPosition(Rotation2d position) {
        pivotSim.setState(position.getRadians(), pivotSim.getAngularVelocityRadPerSec());
    }

    @Override
    public void stop() {
        rollerSetpointRps = 0.0;
    }
}
