package frc.robot.hopper;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import static frc.robot.constants.SubsystemConstants.HoppahConstants.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class HoppahIOReal implements HoppahIO {
   private static final int motorLID = 30;
   private static final int motorBLID = 31;  
   private static final int motorBRID = 32;


   protected final TalonFX motorL;
   protected final TalonFX motorBL;
   protected final TalonFX motorBR;

   //control requests
   private final VelocityVoltage control = new VelocityVoltage(0);
   private final VoltageOut voltControl = new VoltageOut(0);

    protected final StatusSignal<Current> motorLStatorCurrent;
    protected final StatusSignal<Current> motorLSupplyCurrent;
    protected final StatusSignal<AngularVelocity> motorLVelocityRPS;
    protected final StatusSignal<Voltage> motorLSupplyVoltage;
    protected final StatusSignal<Voltage> motorLOutputVolts;

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
    private final Debouncer motorBLIsConnected = new Debouncer(0.5);
    private final Debouncer motorBRIsConnected = new Debouncer(0.5);

    public HoppahIOReal() {
        motorL = new TalonFX(motorLID);
        motorBL = new TalonFX(motorBLID);
        motorBR = new TalonFX(motorBRID);

         Slot0Configs beltSlot0Configs = new Slot0Configs()
            .withKP(BELT_KP)
            .withKD(BELT_KD)
            .withKV(BELT_KV)
            .withKS(BELT_KS);

        Slot0Configs feederSlot0Configs = new Slot0Configs()
            .withKP(TOP_BELT_KP)
            .withKD(TOP_BELT_KD)
            .withKV(TOP_BELT_KV)
            .withKS(TOP_BELT_KS);

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

        var lowerBeltConfig = new TalonFXConfiguration();
            lowerBeltConfig.withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(30)
                .withSupplyCurrentLimitEnable(true));
                lowerBeltConfig.withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(TOP_BELT_GEARING));
            lowerBeltConfig.withSlot0(feederSlot0Configs);
            lowerBeltConfig.withMotorOutput(new MotorOutputConfigs()
                .withInverted(TOP_BELT_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.Clockwise_Positive));
            lowerBeltConfig.withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast));

        motorL.getConfigurator().apply(beltConfig);
        motorBL.getConfigurator().apply(lowerBeltConfig);
        motorBR.getConfigurator().apply(lowerBeltConfig);

        motorBLStatorCurrent = motorBL.getStatorCurrent();
        motorBLSupplyCurrent = motorBL.getSupplyCurrent();
        motorBLVelocityRPS = motorBL.getVelocity();
        motorBLSupplyVoltage = motorBL.getSupplyVoltage();
        motorBLOutputVolts = motorBL.getMotorVoltage();
       
        motorBRStatorCurrent = motorBR.getStatorCurrent();
        motorBRSupplyCurrent = motorBR.getSupplyCurrent();
        motorBRVelocityRPS = motorBR.getVelocity();
        motorBRSupplyVoltage = motorBR.getSupplyVoltage();
        motorBROutputVolts = motorBR.getMotorVoltage();
        
        motorLStatorCurrent = motorL.getStatorCurrent();
        motorLSupplyCurrent = motorL.getSupplyCurrent();
        motorLVelocityRPS = motorL.getVelocity();
        motorLSupplyVoltage = motorL.getSupplyVoltage();
        motorLOutputVolts = motorL.getMotorVoltage();

        BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        motorBLStatorCurrent,
        motorBLSupplyCurrent,
        motorBLVelocityRPS,
        motorBLSupplyVoltage,
        motorBLOutputVolts,
        motorBRStatorCurrent,
        motorBRSupplyCurrent,
        motorBRVelocityRPS,
        motorBRSupplyVoltage,
        motorBROutputVolts,
        motorLStatorCurrent,
        motorLSupplyCurrent,
        motorLVelocityRPS,
        motorLSupplyVoltage,
        motorLOutputVolts);

        motorBR.setControl(new Follower(motorBL.getDeviceID(), MotorAlignmentValue.Aligned));
    }
    
    public void updateInputs(HoppahIOInputs inputs) {
        StatusCode motorLStatus = BaseStatusSignal.refreshAll(
        motorLStatorCurrent,
        motorLSupplyCurrent,
        motorLVelocityRPS,
        motorLSupplyVoltage);

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
    
    @Override
    public void setHoppahState(double bottomSpin, double topSpin) {
        motorBL.setControl(control.withVelocity(bottomSpin));
        motorL.setControl(control.withVelocity(topSpin));
    }

    @Override
    public void defaultState() {
        motorBL.setControl(voltControl);
        motorL.setControl(voltControl);
    }
}