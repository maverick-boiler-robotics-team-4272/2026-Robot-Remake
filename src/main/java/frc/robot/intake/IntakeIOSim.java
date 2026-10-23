
package frc.robot.intake;

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

    private final DCMotorSim rollerSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(ROLLER_MOTORS, ROLLER_MOI, ROLLER_GEAR_RATIO), ROLLER_MOTORS);

    private final PIDController rollerController = new PIDController(ROLLER_KP, 0.0, 0.0);

    private double rollerSetpointRps = 0.0;

    private double rollerAppliedVolts = 0.0;

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        double rollerRps = Units.radiansToRotations(rollerSim.getAngularVelocityRadPerSec());
        double rollerVolts = ROLLER_KV * rollerSetpointRps + rollerController.calculate(rollerRps, rollerSetpointRps);

        boolean disabled = DriverStation.isDisabled();
        rollerAppliedVolts = disabled ? 0.0 : MathUtil.clamp(rollerVolts, -12.0, 12.0);

        rollerSim.setInputVoltage(rollerAppliedVolts);
        rollerSim.update(LOOP_PERIOD_SECS);

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
    }
   
    @Override
    public void defaultState() {
        rollerSetpointRps = 0.0;
    }
}
