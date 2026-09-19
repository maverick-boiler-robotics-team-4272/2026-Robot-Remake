package frc.robot.subsystems;

import static frc.robot.constants.SubsystemConstants.IntakeConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
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
    private final TalonFX pivot = new TalonFX(PIVOT_ID, CAN_BUS);

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

    private final StatusSignal<Angle> pivotPosition = pivot.getPosition();
    private final StatusSignal<AngularVelocity> pivotVelocity = pivot.getVelocity();
    private final StatusSignal<Voltage> pivotAppliedVolts = pivot.getMotorVoltage();
    private final StatusSignal<Current> pivotStatorCurrent = pivot.getStatorCurrent();
    private final StatusSignal<Current> pivotSupplyCurrent = pivot.getSupplyCurrent();
    private final StatusSignal<Temperature> pivotTemp = pivot.getDeviceTemp();

    private final BaseStatusSignal[] rollerSignals = {
        rollerVelocity, rollerAppliedVolts, rollerStatorCurrent, rollerSupplyCurrent, rollerTemp
    };
    private final BaseStatusSignal[] followerSignals = {
        followerVelocity, followerAppliedVolts, followerStatorCurrent, followerSupplyCurrent, followerTemp
    };
    private final BaseStatusSignal[] pivotSignals = {
        pivotPosition, pivotVelocity, pivotAppliedVolts, pivotStatorCurrent, pivotSupplyCurrent, pivotTemp
    };

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

        var pivotConfig = new TalonFXConfiguration();
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfig.MotorOutput.Inverted =
                PIVOT_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        pivotConfig.Feedback.SensorToMechanismRatio = PIVOT_GEAR_RATIO;
        pivotConfig.CurrentLimits.StatorCurrentLimit = PIVOT_STATOR_LIMIT_AMPS;
        pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        pivotConfig.CurrentLimits.SupplyCurrentLimit = PIVOT_SUPPLY_LIMIT_AMPS;
        pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = PIVOT_FORWARD_LIMIT_ROTATIONS;
        pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = PIVOT_REVERSE_LIMIT_ROTATIONS;
        pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        pivotConfig.Slot0.kP = PIVOT_KP;
        pivotConfig.Slot0.kD = PIVOT_KD;

        PhoenixUtil.tryUntilOk(5, () -> rollerLeader.getConfigurator().apply(rollerConfig));
      
        PhoenixUtil.tryUntilOk(5, () -> rollerFollower.getConfigurator().apply(rollerConfig));
        PhoenixUtil.tryUntilOk(5, () -> pivot.getConfigurator().apply(pivotConfig));

        rollerFollower.setControl(new Follower(
                ROLLER_LEADER_ID,
                ROLLER_FOLLOWER_OPPOSED ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, rollerSignals);
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, followerSignals);
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, pivotSignals);
        rollerLeader.optimizeBusUtilization();
        rollerFollower.optimizeBusUtilization();
        pivot.optimizeBusUtilization();
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

        inputs.pivotConnected = BaseStatusSignal.refreshAll(pivotSignals).isOK();
        inputs.pivotPosition = Rotation2d.fromRotations(pivotPosition.getValueAsDouble());
        inputs.pivotVelocityRadPerSec = Units.rotationsToRadians(pivotVelocity.getValueAsDouble());
        inputs.pivotAppliedVolts = pivotAppliedVolts.getValueAsDouble();
        inputs.pivotStatorCurrentAmps = pivotStatorCurrent.getValueAsDouble();
        inputs.pivotSupplyCurrentAmps = pivotSupplyCurrent.getValueAsDouble();
        inputs.pivotTempCelsius = pivotTemp.getValueAsDouble();
    }

    @Override
    public void setRollerVelocity(double rotationsPerSec) {
        rollerLeader.setControl(rollerVelocityRequest.withVelocity(rotationsPerSec));
    }

    @Override
    public void setPivotPosition(double rotations) {
        pivot.setControl(pivotPositionRequest.withPosition(rotations));
    }

    @Override
    public void resetPivotPosition(Rotation2d position) {
        pivot.setPosition(position.getRotations());
    }

    @Override
    public void stop() {
        rollerLeader.setControl(neutralRequest);
        pivot.setControl(neutralRequest);
    }
}
