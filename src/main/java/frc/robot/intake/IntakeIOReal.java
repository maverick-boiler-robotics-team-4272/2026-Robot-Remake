
package frc.robot.intake;

import static frc.robot.constants.SubsystemConstants.IntakeConstants.*;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.constants.SubsystemConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class IntakeIOReal implements IntakeIO {
    private final TalonFX rollerLeader = new TalonFX(ROLLER_LEADER_ID, CAN_BUS);
    private final TalonFX rollerFollower = new TalonFX(ROLLER_FOLLOWER_ID, CAN_BUS);

    private final VelocityVoltage rollerVelocityRequest = new VelocityVoltage(0.0).withEnableFOC(true);
    private final PositionVoltage pivotPositionRequest = new PositionVoltage(0.0).withEnableFOC(true);
    private final NeutralOut neutralRequest = new NeutralOut();

    private final StatusSignal<AngularVelocity> rollerVelocity = rollerLeader.getVelocity();
    private final StatusSignal<Voltage> rollerAppliedVolts = rollerLeader.getMotorVoltage();
    private final StatusSignal<Current> rollerStatorCurrent = rollerLeader.getStatorCurrent();
    private final StatusSignal<Current> rollerSupplyCurrent = rollerLeader.getSupplyCurrent();
    private final StatusSignal<Temperature> rollerTemp = rollerLeader.getDeviceTemp();

    private final StatusSignal<AngularVelocity> followerVelocity = rollerFollower.getVelocity();
    private final StatusSignal<Voltage> followerAppliedVolts = rollerFollower.getMotorVoltage();
    private final StatusSignal<Current> followerStatorCurrent = rollerFollower.getStatorCurrent();
    private final StatusSignal<Current> followerSupplyCurrent = rollerFollower.getSupplyCurrent();
    private final StatusSignal<Temperature> followerTemp = rollerFollower.getDeviceTemp();

    private final BaseStatusSignal[] rollerSignals = {
        rollerVelocity, rollerAppliedVolts, rollerStatorCurrent, rollerSupplyCurrent, rollerTemp
    };
    private final BaseStatusSignal[] followerSignals = {
        followerVelocity, followerAppliedVolts, followerStatorCurrent, followerSupplyCurrent, followerTemp
    };

    //control requests
    private final VelocityVoltage control = new VelocityVoltage(0);
    private final VoltageOut voltControl = new VoltageOut(0);

    public IntakeIOReal() {
        var rollerConfig = new TalonFXConfiguration();
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rollerConfig.MotorOutput.Inverted =
                ROLLER_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        rollerConfig.Feedback.SensorToMechanismRatio = ROLLER_GEAR_RATIO;
        rollerConfig.CurrentLimits.StatorCurrentLimit = ROLLER_STATOR_LIMIT_AMPS;
        rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rollerConfig.CurrentLimits.SupplyCurrentLimit = ROLLER_SUPPLY_LIMIT_AMPS;
        rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rollerConfig.Slot0.kP = ROLLER_KP;
        rollerConfig.Slot0.kV = ROLLER_KV;

        PhoenixUtil.tryUntilOk(5, () -> rollerLeader.getConfigurator().apply(rollerConfig));
      
        PhoenixUtil.tryUntilOk(5, () -> rollerFollower.getConfigurator().apply(rollerConfig));

        rollerFollower.setControl(new Follower(
                ROLLER_LEADER_ID,
                MotorAlignmentValue.Opposed));

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, rollerSignals);
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, followerSignals);
        rollerLeader.optimizeBusUtilization();
        rollerFollower.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.rollerConnected = BaseStatusSignal.refreshAll(rollerSignals).isOK();
        inputs.rollerVelocityRadPerSec = Units.rotationsToRadians(rollerVelocity.getValueAsDouble());
        inputs.rollerAppliedVolts = rollerAppliedVolts.getValueAsDouble();
        inputs.rollerStatorCurrentAmps = rollerStatorCurrent.getValueAsDouble();
        inputs.rollerSupplyCurrentAmps = rollerSupplyCurrent.getValueAsDouble();
        inputs.rollerTempCelsius = rollerTemp.getValueAsDouble();

        inputs.rollerFollowerConnected = BaseStatusSignal.refreshAll(followerSignals).isOK();
        inputs.rollerFollowerVelocityRadPerSec = Units.rotationsToRadians(followerVelocity.getValueAsDouble());
        inputs.rollerFollowerAppliedVolts = followerAppliedVolts.getValueAsDouble();
        inputs.rollerFollowerStatorCurrentAmps = followerStatorCurrent.getValueAsDouble();
        inputs.rollerFollowerSupplyCurrentAmps = followerSupplyCurrent.getValueAsDouble();
        inputs.rollerFollowerTempCelsius = followerTemp.getValueAsDouble();
    }

    @Override
    public void setIntakeState(double rps) {
        rollerLeader.setControl(control.withVelocity(rps));
    }

    @Override
    public void defaultState() {
        rollerLeader.setControl(neutralRequest);
    }
}
