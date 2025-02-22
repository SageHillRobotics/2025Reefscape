package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase{
    private final TalonFX leftPivot;
    private final TalonFX rightPivot;
    private final CANcoder pivotEncoder;

    private final int LEFT_PIVOT_CAN_ID = 5;
    private final int RIGHT_PIVOT_CAN_ID = 6;
    private final int PIVOT_ENCODER_CAN_ID = 7;

    private final double GEAR_REDUCTION = 150/1.0;
    private final double ENCODER_OFFSET = 0.4;

    private final double kS = 0.25;
    private final double kV = 1; 
    private final double kG = 0.1;
    private final double kA = 0.01;
    private final double kP = 12;
    private final double kI = 0;
    private final double kD = 0.1;

    private final double kCruiseVelocity = 20;
    private final double kAcceleration = 10;
    private final double kJerk = 1600;


    public Pivot(){
        leftPivot = new TalonFX(LEFT_PIVOT_CAN_ID);
        rightPivot = new TalonFX(RIGHT_PIVOT_CAN_ID);
        pivotEncoder = new CANcoder(PIVOT_ENCODER_CAN_ID);

        leftPivot.getConfigurator().apply(configureLeft(new TalonFXConfiguration()));
        rightPivot.getConfigurator().apply(configureRight(new TalonFXConfiguration()));
        pivotEncoder.getConfigurator().apply(configureCANCoder(new CANcoderConfiguration()));

        // leftPivot.setControl(new MotionMagicVoltage(0));
        rightPivot.setControl(new Follower(LEFT_PIVOT_CAN_ID, true));
    }

    private CANcoderConfiguration configureCANCoder(CANcoderConfiguration config){
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        config.MagnetSensor.withMagnetOffset(-1 * ENCODER_OFFSET);
        return config;
    }

    private TalonFXConfiguration configureLeft(TalonFXConfiguration config){
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        config.Feedback.SensorToMechanismRatio = 1.0;
        config.Feedback.RotorToSensorRatio = GEAR_REDUCTION;

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
    private TalonFXConfiguration configureRight(TalonFXConfiguration config){
        config.Feedback.SensorToMechanismRatio = GEAR_REDUCTION;
        return config;
    }

    private void movetoAngle(double degrees){
        MotionMagicVoltage request = new MotionMagicVoltage(Units.degreesToRotations(degrees));
        rightPivot.setControl(request);
    }

    private double getAngleDegrees(){
        StatusSignal<Angle> positionSignal = pivotEncoder.getPosition();
        return positionSignal.getValue().in(Degrees);
    }
    

}
