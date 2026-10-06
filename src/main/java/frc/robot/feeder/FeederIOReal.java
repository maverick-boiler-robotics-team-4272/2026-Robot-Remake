package frc.robot.feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import static frc.robot.constants.SubsystemConstants.FeederConstants.*;


public class FeederIOReal implements FeederIO {
    private static final int motorLID = 40;
    private static final int motorRID = 41;

    protected final TalonFX motorL;
    protected final TalonFX motorR;

    //control requests
    private final VelocityVoltage control = new VelocityVoltage(0);
    private final VoltageOut voltControl = new VoltageOut(0);

    protected final StatusSignal<Current> motorLStatorCurrent;
    protected final StatusSignal<Current> motorLSupplyCurrent;
    protected final StatusSignal<AngularVelocity> motorLVelocityRPS;
    protected final StatusSignal<Voltage> motorLSupplyVoltage;
    protected final StatusSignal<Voltage> motorLOutputVolts;

    protected final StatusSignal<Current> motorRStatorCurrent;
    protected final StatusSignal<Current> motorRSupplyCurrent;
    protected final StatusSignal<AngularVelocity> motorRVelocityRPS;
    protected final StatusSignal<Voltage> motorRSupplyVoltage;
    protected final StatusSignal<Voltage> motorROutputVolts;

    
    private final Debouncer motorLIsConnected = new Debouncer(0.5);
    private final Debouncer motorRIsConnected = new Debouncer(0.5);

    public FeederIOReal() {
        
        motorR = new TalonFX(motorRID);
        motorL = new TalonFX(motorLID);

         Slot0Configs feedSlot0Configs = new Slot0Configs()
            .withKP(FEED_KP)
            .withKD(FEED_KD)
            .withKV(FEED_KV)
            .withKS(FEED_KS);

            var feederConfig = new TalonFXConfiguration();
            feederConfig.withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(30)
                .withSupplyCurrentLimitEnable(true));
                feederConfig.withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(FEED_GEARING));
            feederConfig.withSlot0(feedSlot0Configs);
            feederConfig.withMotorOutput(new MotorOutputConfigs()
                .withInverted(FEED_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.Clockwise_Positive));
            feederConfig.withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast));

                
        motorL.getConfigurator().apply(feederConfig);
        motorR.getConfigurator().apply(feederConfig);

        
        motorLStatorCurrent = motorL.getStatorCurrent();
        motorLSupplyCurrent = motorL.getSupplyCurrent();
        motorLVelocityRPS = motorL.getVelocity();
        motorLSupplyVoltage = motorL.getSupplyVoltage();
        motorLOutputVolts = motorL.getMotorVoltage();
       
        motorRStatorCurrent = motorR.getStatorCurrent();
        motorRSupplyCurrent = motorR.getSupplyCurrent();
        motorRVelocityRPS = motorR.getVelocity();
        motorRSupplyVoltage = motorR.getSupplyVoltage();
        motorROutputVolts = motorR.getMotorVoltage();

        BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        motorLStatorCurrent,
        motorLSupplyCurrent,
        motorLVelocityRPS,
        motorLSupplyVoltage,
        motorLOutputVolts,
        motorRStatorCurrent,
        motorRSupplyCurrent,
        motorRVelocityRPS,
        motorRSupplyVoltage,
        motorROutputVolts,
        motorLStatorCurrent,
        motorLSupplyCurrent,
        motorLVelocityRPS,
        motorLSupplyVoltage,
        motorLOutputVolts);

        motorR.setControl(new Follower(motorL.getDeviceID(), MotorAlignmentValue.Opposed));

    }
    public void updateInputs(FeederIOInputs inputs) {
        
        StatusCode motorLStatus = BaseStatusSignal.refreshAll(
        motorLStatorCurrent,
        motorLSupplyCurrent,
        motorLVelocityRPS,
        motorLSupplyVoltage);

        StatusCode motorRStatus = BaseStatusSignal.refreshAll(
        motorRStatorCurrent,
        motorRSupplyCurrent,
        motorRVelocityRPS,
        motorRSupplyVoltage);

        
        inputs.motorLIsConnected = motorLIsConnected.calculate(motorLStatus.isOK());
        inputs.motorLStatorCurrent = motorLStatorCurrent.getValueAsDouble();
        inputs.motorLSupplyCurrent  = motorLSupplyCurrent.getValueAsDouble();
        inputs.motorLVelocityRPS = motorLVelocityRPS.getValueAsDouble();
        inputs.motorLSupplyVoltage = motorLSupplyVoltage.getValueAsDouble();

        inputs.motorRIsConnected = motorRIsConnected.calculate(motorRStatus.isOK());
        inputs.motorRStatorCurrent = motorRStatorCurrent.getValueAsDouble();
        inputs.motorRSupplyCurrent  = motorRSupplyCurrent.getValueAsDouble();
        inputs.motorRVelocityRPS = motorRVelocityRPS.getValueAsDouble();
        inputs.motorRSupplyVoltage = motorRSupplyVoltage.getValueAsDouble();

    }

    @Override
    public void setFeederState(double spin) {
        motorL.setControl(control.withVelocity(spin));
    }

    @Override
    public void defaultState() {
        motorL.setControl(voltControl);
    }

}
