package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffector extends SubsystemBase{
    private final TalonFX indexMotor;
    private final int INDEX_MOTOR_CAN_ID = 10;

    private final DigitalInput beamBreak;
    private final int BEAM_BREAK_ID = 0;
    

    private final double INTAKE_SPEED = 0.60; //60% output
    private final double HOLD_SPEED = 0.01; //1% outpput

    public EndEffector(){
        indexMotor = new TalonFX(INDEX_MOTOR_CAN_ID);
        beamBreak = new DigitalInput(BEAM_BREAK_ID);
    }

    public void setIntakeSpeed(){
        indexMotor.setControl(new DutyCycleOut(INTAKE_SPEED));
    }

    public void indexBrake(){
        indexMotor.setControl(new StaticBrake());
    }

    public void setHoldSpeed(){
        indexMotor.setControl(new DutyCycleOut(HOLD_SPEED));
    }

    public boolean getBeamBreakValue(){
        return beamBreak.get();
    }

}
