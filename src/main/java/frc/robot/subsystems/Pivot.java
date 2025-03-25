package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase{
    private final TalonFX leftPivot;
    private final TalonFX rightPivot;
    private final CANcoder pivotEncoder;

    private final int LEFT_PIVOT_CAN_ID = 7;
    private final int RIGHT_PIVOT_CAN_ID = 6;
    private final int PIVOT_ENCODER_CAN_ID = 15;

    private final double GEAR_REDUCTION = 180/1.0;

    private final double ENCODER_ZERO_OFFSET = 0.556640;

    // private final double RELATIVE_OFFSET = 90/360.0;

    private final double kS = 0.14;
    private final double kV = 0.1; 
    private final double kG = 0.02;
    private final double kA = 0.08;
    private final double kP = 100;
    private final double kI = 0;
    private final double kD = 0;

    private final double kCruiseVelocity = 80;
    private final double kAcceleration = 160;
    private final double kJerk = 1600;

    private final double CURRENT_LIMIT = 80;

    private double setpoint;
    private final double POSITION_TOLERANCE = 3; //1 degree

    public Pivot(){
        rightPivot = new TalonFX(RIGHT_PIVOT_CAN_ID);
        leftPivot = new TalonFX(LEFT_PIVOT_CAN_ID);
        pivotEncoder = new CANcoder(PIVOT_ENCODER_CAN_ID);

        rightPivot.getConfigurator().apply(configureRight(new TalonFXConfiguration()));
        leftPivot.getConfigurator().apply(configureLeft(new TalonFXConfiguration()));

        pivotEncoder.getConfigurator().apply(configureCANCoder(new CANcoderConfiguration()));

        // rightPivot.setPosition(ENCODER_OFFSET);
        // rightPivot.setPosition(RELATIVE_OFFSET);
    
        // rightPivot.setControl(new MotionMagicVoltage(0));
        leftPivot.setControl(new Follower(RIGHT_PIVOT_CAN_ID, true));
    }

    private CANcoderConfiguration configureCANCoder(CANcoderConfiguration config){

        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.6;
        config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        // config.MagnetSensor.withMagnetOffset(-(ENCODER_OFFSET - START_OFFSET));
        config.MagnetSensor.withMagnetOffset(-ENCODER_ZERO_OFFSET);

        return config;

    }

    private TalonFXConfiguration configureRight(TalonFXConfiguration config){

        config.withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(CURRENT_LIMIT));

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Feedback.FeedbackRemoteSensorID = pivotEncoder.getDeviceID();
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        
        config.Feedback.SensorToMechanismRatio = 1.0;
        config.Feedback.RotorToSensorRatio = GEAR_REDUCTION;

        // config.Feedback.SensorToMechanismRatio = GEAR_REDUCTION;

        setpoint = 0;

        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;
        config.Slot0.kG = kG;
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        config.MotionMagic.MotionMagicCruiseVelocity = kCruiseVelocity;
        config.MotionMagic.MotionMagicAcceleration = kAcceleration;
        config.MotionMagic.MotionMagicJerk = kJerk;

        return config;
    }
    private TalonFXConfiguration configureLeft(TalonFXConfiguration config){

        config.withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(CURRENT_LIMIT));
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // config.Feedback.withFeedbackRotorOffset(GEAR_REDUCTION);
        // config.Feedback.FeedbackRotorOffset = ENCODER_OFFSET;
        // config.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        return config;
    }

    public void movetoAngle(double degrees){
        MotionMagicVoltage request = new MotionMagicVoltage(Units.degreesToRotations(degrees)).withEnableFOC(true);
        rightPivot.setControl(request);
        leftPivot.setControl(new Follower(RIGHT_PIVOT_CAN_ID, true));
        setpoint = degrees;
    }

    public double getAngleDegrees(){
        StatusSignal<Angle> positionSignal = pivotEncoder.getPosition();
        // StatusSignal<Angle> positionSignal = rightPivot.getPosition();
        return positionSignal.getValue().in(Degrees);
    }

    public boolean atSetpoint(){
        // StatusSignal<Angle> posSignal = rightPivot.getPosition();
        StatusSignal<Angle> posSignal = pivotEncoder.getPosition();
        double curPos = posSignal.getValue().in(Degrees);

        return Math.abs(curPos - setpoint) < POSITION_TOLERANCE;
    }
}
