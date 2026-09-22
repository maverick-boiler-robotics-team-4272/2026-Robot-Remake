package frc.robot.hopper;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.Follower;

import static frc.robot.constants.SubsystemConstants.HoppahConstants.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class HoppahIOReal implements HoppahIO {
   private static final int motorLID = 0;
   private static final int motorRID = 0;
   private static final int motorBLID = 0;  
   private static final int motorBRID = 0;


   protected final TalonFX motorL;
   protected final TalonFX motorR;
   protected final TalonFX motorBL;
   protected final TalonFX motorBR;

   //control requests

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

    protected final StatusSignal<Current> motorBLStatorCurrent;
    protected final StatusSignal<Current> motorBLSupplyCurrent;
    protected final StatusSignal<AngularVelocity> motorBLVelocityRPS;
    protected final StatusSignal<Voltage> motorBLSupplyVoltage;
    protected final StatusSignal<Voltage> motorBLOutputVolts;

    protected final StatusSignal<Current> motorBRStatorCurrent;
    protected final StatusSignal<Current> motorBRSupplyCurrent;
    protected final StatusSignal<AngularVelocity> motorBRVelocityRPS;
    protected final StatusSignal<Voltage> motorBRSupplyVoltage;
    protected final StatusSignal<Voltage> motorBROutputVolts;

    private final Debouncer motorLIsConnected = new Debouncer(0.5);
    private final Debouncer motorRIsConnected = new Debouncer(0.5);
    private final Debouncer motorBLIsConnected = new Debouncer(0.5);
    private final Debouncer motorBRIsConnected = new Debouncer(0.5);

    public HoppahIOReal() {
        motorL = new TalonFX(motorLID);
        motorR = new TalonFX(motorRID);
        motorBL = new TalonFX(motorBLID);
        motorBR = new TalonFX(motorBRID);

         Slot0Configs beltSlot0Configs = new Slot0Configs()
            .withKP(BELT_KP)
            .withKD(BELT_KD)
            .withKV(BELT_KV)
            .withKS(BELT_KS);

        Slot0Configs feederSlot0Configs = new Slot0Configs()
            .withKP(FEEDER_KP)
            .withKD(FEEDER_KD)
            .withKV(FEEDER_KV)
            .withKS(FEEDER_KS);

        var beltConfig = new TalonFXConfiguration();
            beltConfig.withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(60)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(60)
                .withSupplyCurrentLimitEnable(true));
            beltConfig.withSlot0(beltSlot0Configs);
            beltConfig.withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(BELT_GEARING));
            beltConfig.withMotorOutput(new MotorOutputConfigs()
                .withInverted(BELT_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive));
            beltConfig.withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast));

        var feederConfig = new TalonFXConfiguration();
            feederConfig.withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(30)
                .withSupplyCurrentLimitEnable(true));
                feederConfig.withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(FEEDER_GEARING));
            feederConfig.withSlot0(feederSlot0Configs);
            feederConfig.withMotorOutput(new MotorOutputConfigs()
                .withInverted(FEEDER_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.Clockwise_Positive));
            feederConfig.withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast));

        motorL.getConfigurator().apply(beltConfig);
        motorR.getConfigurator().apply(beltConfig);
        motorBL.getConfigurator().apply(feederConfig);
        motorBR.getConfigurator().apply(feederConfig);

        motorBLStatorCurrent = motorBL.getStatorCurrent();
        motorBLSupplyCurrent = motorBL.getSupplyCurrent();
        motorBLVelocityRPS = motorBL.getVelocity();
        motorBLSupplyVoltage = motorBL.getSupplyVoltage();
       
        motorBRStatorCurrent = motorBR.getStatorCurrent();
        motorBRSupplyCurrent = motorBR.getSupplyCurrent();
        motorBRVelocityRPS = motorBR.getVelocity();
        motorBRSupplyVoltage = motorBR.getSupplyVoltage();
        
        motorLStatorCurrent = motorL.getStatorCurrent();
        motorLSupplyCurrent = motorL.getSupplyCurrent();
        motorLVelocityRPS = motorL.getVelocity();
        motorLSupplyVoltage = motorL.getSupplyVoltage();
        
        motorRStatorCurrent = motorR.getStatorCurrent();
        motorRSupplyCurrent = motorR.getSupplyCurrent();
        motorRVelocityRPS = motorR.getVelocity();
        motorRSupplyVoltage = motorR.getSupplyVoltage();

        BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        motorBLStatorCurrent,
        motorBLSupplyCurrent,
        motorBLVelocityRPS,
        motorBLSupplyVoltage,
        motorBRStatorCurrent,
        motorBRSupplyCurrent,
        motorBRVelocityRPS,
        motorBRSupplyVoltage,
        motorLStatorCurrent,
        motorLSupplyCurrent,
        motorLVelocityRPS,
        motorLSupplyVoltage,
        motorRStatorCurrent,
        motorRSupplyCurrent,
        motorRVelocityRPS,
        motorRSupplyVoltage);

        
        final DutyCycleOut m_request = new DutyCycleOut(0);

        motorR.setControl(new Follower(motorL.getDeviceID(), MotorAlignmentValue.Aligned));
        motorBR.setControl(new Follower(motorBL.getDeviceID(), MotorAlignmentValue.Aligned));
    }
    
    public void updateInputs(HoppahIOInputs inputs) {
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

        StatusCode motorBLStatus = BaseStatusSignal.refreshAll(
        motorBLStatorCurrent,
        motorBLSupplyCurrent,
        motorBLVelocityRPS,
        motorBLSupplyVoltage);

        StatusCode motorBRStatus = BaseStatusSignal.refreshAll(
        motorBRStatorCurrent,
        motorBRSupplyCurrent,
        motorBRVelocityRPS,
        motorBRSupplyVoltage);


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
        
        inputs.motorBLIsConnected = motorBLIsConnected.calculate(motorBLStatus.isOK());
        inputs.motorBLStatorCurrent = motorBLStatorCurrent.getValueAsDouble();
        inputs.motorBLSupplyCurrent  = motorBLSupplyCurrent.getValueAsDouble();
        inputs.motorBLVelocityRPS = motorBLVelocityRPS.getValueAsDouble();
        inputs.motorBLSupplyVoltage = motorBLSupplyVoltage.getValueAsDouble();

        inputs.motorBRIsConnected = motorBRIsConnected.calculate(motorBRStatus.isOK());
        inputs.motorBRStatorCurrent = motorBRStatorCurrent.getValueAsDouble();
        inputs.motorBRSupplyCurrent  = motorBRSupplyCurrent.getValueAsDouble();
        inputs.motorBRVelocityRPS = motorBRVelocityRPS.getValueAsDouble();
        inputs.motorBRSupplyVoltage = motorBRSupplyVoltage.getValueAsDouble();
    }
    
    public void setHoppahState(double bottomSpin, double topSpin) {
        motorBL.setVoltage(bottomSpin);
        motorL.setVoltage(topSpin);
    }
}
   