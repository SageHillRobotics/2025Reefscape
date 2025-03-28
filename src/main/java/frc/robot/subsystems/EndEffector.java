package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffector extends SubsystemBase{
    private final TalonFX verticalRoller;
    private final TalonFX horizontalRoller;

    private final SparkMax wristMotor;

    private final RelativeEncoder wristEncoder;
    private final SparkClosedLoopController wristController;

    private final double startingOffset = 129.5/360.0;

    private final int WRIST_MOTOR_CAN_ID = 31;
    private final int HORIZONTAL_ROLLER_CAN_ID = 32;
    private final int VERTICAL_ROLLER_CAN_ID = 10;

    private final double kP = 4;
    private final double kI = 0;
    private final double kD = 0;
    private final double maxVelocity = 0.5;
    private final double maxAcceleration = 0.5;

    private final DigitalInput beamBreak;
    private final int BEAM_BREAK_ID = 9;
    

    private final double VERTICAL_ROLLER_GROUND_VOLTAGE = 0.5 * -12; //60% output
    private final double HORIZONTAL_ROLLER_GROUND_VOLTAGE = 0.5 * -12;

    private final double VERTICAL_ROLLER_STATION_VOLTAGE = 0.3 * -12;
    private final double HORIZONTAL_ROLLER_STATION_VOLTAGE = 0.3 * -12;

    // private final double HOLD_SPEED = 0.01 * 12; //1% output
    private final double EJECT_SPEED = 0.5 * -12; //100% output

    private final int ENCODER_COUNTS_PER_REV = 8192;
    private final double ENCODER_TO_WRIST_RATIO = 1.0;
    private final double WRIST_CONVERSION_FACTOR = 1.0/ENCODER_TO_WRIST_RATIO;
    
    private final double POSITION_TOLERANCE = 10/360.0;

    private double setpoint;

    public EndEffector(){
        verticalRoller = new TalonFX(VERTICAL_ROLLER_CAN_ID);
        horizontalRoller = new TalonFX(HORIZONTAL_ROLLER_CAN_ID);

        wristMotor = new SparkMax(WRIST_MOTOR_CAN_ID, MotorType.kBrushless);
        wristMotor.configure(configureWrist(new SparkMaxConfig()), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        setpoint = -1;

        wristEncoder = wristMotor.getAlternateEncoder();
        wristEncoder.setPosition(startingOffset);

        wristController = wristMotor.getClosedLoopController();
        
        beamBreak = new DigitalInput(BEAM_BREAK_ID);
    }

    public SparkMaxConfig configureWrist(SparkMaxConfig config){
        config.inverted(true);
        config.idleMode(IdleMode.kCoast);
        config.alternateEncoder.positionConversionFactor(WRIST_CONVERSION_FACTOR);
        config.alternateEncoder.countsPerRevolution(ENCODER_COUNTS_PER_REV);
        config.alternateEncoder.inverted(false);
        config.closedLoop.feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder);
        config.closedLoop.pid(kP, kI, kD);
        config.closedLoop.maxMotion
            .maxVelocity(maxVelocity)
            .maxAcceleration(maxAcceleration)
            .allowedClosedLoopError(POSITION_TOLERANCE);

        return config;
    }

    public TalonFXConfiguration configureVerticalRoller(TalonFXConfiguration config){
        config.withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(80));
        return config;
    }
    

    public void setEjectSpeed(){
        verticalRoller.setControl(new VoltageOut(EJECT_SPEED));
    }

    public void setGroundIntakeSpeed(){
        verticalRoller.setControl(new VoltageOut(VERTICAL_ROLLER_GROUND_VOLTAGE));
        horizontalRoller.setControl(new VoltageOut(HORIZONTAL_ROLLER_GROUND_VOLTAGE));
    }

    public void setStationIntakeSpeed(){
        verticalRoller.setControl(new VoltageOut(VERTICAL_ROLLER_STATION_VOLTAGE));
        horizontalRoller.setControl(new VoltageOut(HORIZONTAL_ROLLER_STATION_VOLTAGE));
    }

    public void indexBrake(){
        verticalRoller.setControl(new StaticBrake());
    }

    public void setHoldSpeed(){
        // indexMotor.setControl(new VoltageOut(HOLD_SPEED));
        horizontalRoller.set(0);
        verticalRoller.setControl(new StaticBrake());
    }

    public boolean getBeamBreakValue(){
        return beamBreak.get();
    }

    public double getStatorCurrent(){
        StatusSignal<Current> statorCurrentSignal = verticalRoller.getStatorCurrent();
        return statorCurrentSignal.getValue().in(Amps);
    }

    public double getSupplyCurrent(){
        StatusSignal<Current> supplyCurrentSignal = verticalRoller.getSupplyCurrent();
        return supplyCurrentSignal.getValue().in(Amps);
    }

    public void wristToAngle(double setpointDegrees){
        double setpointRotations = Units.degreesToRotations(setpointDegrees);
        setpoint = setpointRotations;
        wristController.setReference(setpointRotations, SparkBase.ControlType.kPosition);
    }

    public void clearWristReference(){
        wristMotor.set(0);
    }

    public boolean atSetpoint(){
        double curPos = wristEncoder.getPosition();

        return Math.abs(curPos - setpoint) < POSITION_TOLERANCE;
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Beam Break Value", getBeamBreakValue());
        SmartDashboard.putNumber("Wrist Encoder Rotations", wristEncoder.getPosition());
        SmartDashboard.putNumber("Internal Wrist Rotations", wristMotor.getEncoder().getPosition());
    }

}
