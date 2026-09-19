package frc.robot.drum;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.*;

import static frc.robot.constants.SubsystemConstants.DrumConstants.*;

public class DrumIOReal implements DrumIO {
    private static final int motorHoodID = 0;
    private static final int motorLID = 0;
    private static final int motorRID = 0;

    protected final TalonFX motorL;
    protected final TalonFX motorR;
    protected final TalonFX motorHood;

    //control request

    protected final StatusSignal<Angle> motorHoodPosition;
    protected final StatusSignal<Current> motorHoodStatorCurrent;
    protected final StatusSignal<Current> motorHoodSupplyCurrent;
    protected final StatusSignal<AngularVelocity> motorHoodVelocityRPS;
    protected final StatusSignal<Voltage> motorHoodSupplyVoltage;
    protected final StatusSignal<Voltage> motorHoodOutputVoltage;

    protected final StatusSignal<Current> motorLStatorCurrent;
    protected final StatusSignal<Current> motorLSupplyCurrent;
    protected final StatusSignal<AngularVelocity> motorLVelocityRPS;
    protected final StatusSignal<Voltage> motorLSupplyVoltage;
    protected final StatusSignal<Voltage> motorLOutputVoltage;

    protected final StatusSignal<Current> motorRStatorCurrent;
    protected final StatusSignal<Current> motorRSupplyCurrent;
    protected final StatusSignal<AngularVelocity> motorRVelocityRPS;
    protected final StatusSignal<Voltage> motorRSupplyVoltage;
    protected final StatusSignal<Voltage> motorROutputVoltage;

    private final Debouncer motorLIsConnected = new Debouncer(0.5);
    private final Debouncer motorRIsConnected = new Debouncer(0.5);
    private final Debouncer motorHoodIsConnected = new Debouncer(0.5);

    public DrumIOReal() {
        motorR = new TalonFX(motorRID);
        motorL = new TalonFX(motorLID);
        motorHood = new TalonFX(motorHoodID);

        Slot0Configs drumSlot0Configs = new Slot0Configs()
            .withKP(DRUM_KP)
            .withKD(DRUM_KD)
            .withKV(DRUM_KV)
            .withKS(DRUM_KS);

        Slot0Configs hoodSlot0Configs = new Slot0Configs()
            .withKP(HOOD_KP)
            .withKD(HOOD_KD)
            .withKV(HOOD_KV)
            .withKS(HOOD_KS);

        var drumConfig = new TalonFXConfiguration();
            drumConfig.withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(60)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(60)
                .withSupplyCurrentLimitEnable(true));
            drumConfig.withSlot0(drumSlot0Configs);
            drumConfig.withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(DRUM_GEARING));
            drumConfig.withMotorOutput(new MotorOutputConfigs()
                .withInverted(DRUM_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive));
            drumConfig.withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast));

        var hoodConfig = new TalonFXConfiguration();
            hoodConfig.withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(30)
                .withSupplyCurrentLimitEnable(true));
                hoodConfig.withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(HOOD_GEARING));
            hoodConfig.withSlot0(hoodSlot0Configs);
            hoodConfig.withMotorOutput(new MotorOutputConfigs()
                .withInverted(HOOD_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.Clockwise_Positive));
            hoodConfig.withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast));

        motorL.getConfigurator().apply(drumConfig);
        motorR.getConfigurator().apply(drumConfig);
        motorHood.getConfigurator().apply(hoodConfig);
        motorR.setControl(new Follower(motorLID, MotorAlignmentValue.Opposed));
        motorHood.setPosition(0);

        motorHoodPosition = motorHood.getPosition();
        motorHoodStatorCurrent = motorHood.getStatorCurrent();
        motorHoodSupplyCurrent = motorHood.getSupplyCurrent();
        motorHoodVelocityRPS = motorHood.getVelocity();
        motorHoodSupplyVoltage = motorHood.getSupplyVoltage();
        motorHoodOutputVoltage = motorHood.getMotorVoltage();
        
        motorLStatorCurrent = motorL.getStatorCurrent();
        motorLSupplyCurrent = motorL.getSupplyCurrent();
        motorLVelocityRPS = motorL.getVelocity();
        motorLSupplyVoltage = motorL.getSupplyVoltage();
        motorLOutputVoltage = motorL.getMotorVoltage();

        motorRStatorCurrent = motorR.getStatorCurrent();
        motorRSupplyCurrent = motorR.getSupplyCurrent();
        motorRVelocityRPS = motorR.getVelocity();
        motorRSupplyVoltage = motorR.getSupplyVoltage();
        motorROutputVoltage = motorR.getMotorVoltage();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        motorHoodPosition,
        motorHoodStatorCurrent,
        motorHoodSupplyCurrent,
        motorHoodVelocityRPS,
        motorHoodSupplyVoltage,
        motorHoodOutputVoltage,
        motorLStatorCurrent,
        motorLSupplyCurrent,
        motorLVelocityRPS,
        motorLSupplyVoltage,
        motorLOutputVoltage,
        motorRStatorCurrent,
        motorRSupplyCurrent,
        motorRVelocityRPS,
        motorRSupplyVoltage,
        motorROutputVoltage);
    }
    
    public void updateInputs(DrumIOInputs inputs) {
        StatusCode motorLStatus = BaseStatusSignal.refreshAll(
        motorLStatorCurrent,
        motorLSupplyCurrent,
        motorLVelocityRPS,
        motorLSupplyVoltage,
        motorLOutputVoltage);

        StatusCode motorRStatus = BaseStatusSignal.refreshAll(
        motorRStatorCurrent,
        motorRSupplyCurrent,
        motorRVelocityRPS,
        motorRSupplyVoltage,
        motorROutputVoltage);

        StatusCode motorHoodStatus = BaseStatusSignal.refreshAll(
        motorHoodPosition,
        motorHoodStatorCurrent,
        motorHoodSupplyCurrent,
        motorHoodVelocityRPS,
        motorHoodSupplyVoltage,
        motorHoodOutputVoltage);

        inputs.motorLIsConnected = motorLIsConnected.calculate(motorLStatus.isOK());
        inputs.motorLStatorCurrent = motorLStatorCurrent.getValueAsDouble();
        inputs.motorLSupplyCurrent  = motorLSupplyCurrent.getValueAsDouble();
        inputs.motorLVelocityRPS = motorLVelocityRPS.getValueAsDouble();
        inputs.motorLSupplyVoltage = motorLSupplyVoltage.getValueAsDouble();
        inputs.motorLOutputVoltage = motorLOutputVoltage.getValueAsDouble();

        inputs.motorRIsConnected = motorRIsConnected.calculate(motorRStatus.isOK());
        inputs.motorRStatorCurrent = motorRStatorCurrent.getValueAsDouble();
        inputs.motorRSupplyCurrent  = motorRSupplyCurrent.getValueAsDouble();
        inputs.motorRVelocityRPS = motorRVelocityRPS.getValueAsDouble();
        inputs.motorRSupplyVoltage = motorRSupplyVoltage.getValueAsDouble();
        inputs.motorROutputVoltage = motorROutputVoltage.getValueAsDouble();

        inputs.motorHoodIsConnected = motorHoodIsConnected.calculate(motorHoodStatus.isOK());
        inputs.motorHoodPosition = motorHoodPosition.getValueAsDouble();
        inputs.motorHoodStatorCurrent = motorHoodStatorCurrent.getValueAsDouble();
        inputs.motorHoodSupplyCurrent  = motorHoodSupplyCurrent.getValueAsDouble();
        inputs.motorHoodVelocityRPS = motorHoodVelocityRPS.getValueAsDouble();
        inputs.motorHoodSupplyVoltage = motorHoodSupplyVoltage.getValueAsDouble();
        inputs.motorHoodOutputVoltage = motorHoodOutputVoltage.getValueAsDouble();
    }

}