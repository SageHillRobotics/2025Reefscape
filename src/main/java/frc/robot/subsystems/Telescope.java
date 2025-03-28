package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Telescope extends SubsystemBase{

    private final TalonFX frontTelescope;
    private final TalonFX backTelescope;

    private final int FRONT_TELESCOPE_CAN_ID = 9;
    private final int BACK_TELESCOPE_CAN_ID = 8;

    private final double CURRENT_LIMIT = 80;
    private final double POSITION_TOLERANCE = 2;
    private double setpoint;

    // private final double GEAR_RATIO = 4.0/1.0;

    private final double kS = 0.25; 
    private final double kV = 0.08;    
    private final double kG = 0.5;
    private final double kA = 0.02;
    private final double kP = 0.7;
    private final double kI = 0;
    private final double kD = 0;

    private final double kCruiseVelocity = 60;
    private final double kAcceleration = 100;
    // private final double kJerk = 100;

    public Telescope(){
        frontTelescope = new TalonFX(FRONT_TELESCOPE_CAN_ID);
        backTelescope = new TalonFX(BACK_TELESCOPE_CAN_ID);

        frontTelescope.getConfigurator().apply(configureFront(new TalonFXConfiguration()));
        backTelescope.getConfigurator().apply(configureBack(new TalonFXConfiguration()));

        setpoint = 0;

        backTelescope.setControl(new Follower(FRONT_TELESCOPE_CAN_ID, false));
    }

    private TalonFXConfiguration configureFront(TalonFXConfiguration config){
        // config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        // config.Feedback.SensorToMechanismRatio = 1.0;
        // config.Feedback.RotorToSensorRatio = GEAR_REDUCTION;
        config.withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(CURRENT_LIMIT));
        // config.Feedback.SensorToMechanismRatio = GEAR_RATIO;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;
        config.Slot0.kG = kG;
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        config.MotionMagic.MotionMagicCruiseVelocity = kCruiseVelocity;
        config.MotionMagic.MotionMagicAcceleration = kAcceleration;
        // config.MotionMagic.MotionMagicJerk = kJerk;

        return config;
    }

    private TalonFXConfiguration configureBack(TalonFXConfiguration config){
        config.withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(CURRENT_LIMIT));
        // config.Feedback.withFeedbackRotorOffset(GEAR_RATIO);
        // config.Feedback.FeedbackRotorOffset = ENCODER_OFFSET;

        return config;
    }

    public void movetoPosition(double rotations){
        MotionMagicVoltage request = new MotionMagicVoltage(rotations);
        frontTelescope.setControl(request);
        backTelescope.setControl(new Follower(FRONT_TELESCOPE_CAN_ID, false));
        setpoint = rotations;
    }
    
    public double getPosition(){
        StatusSignal<Angle> posSignal = frontTelescope.getPosition();
        double curPos = posSignal.getValue().in(Rotations);
        return curPos;
    }

    public boolean atSetpoint(){
        StatusSignal<Angle> posSignal = frontTelescope.getPosition();
        double curPos = posSignal.getValue().in(Rotations);

        return Math.abs(curPos - setpoint) < POSITION_TOLERANCE;
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Telescope at setpoint", atSetpoint());
    }
}
