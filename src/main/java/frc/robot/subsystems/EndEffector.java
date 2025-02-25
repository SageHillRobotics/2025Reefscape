package frc.robot.subsystems;

import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffector extends SubsystemBase{
    private final TalonFX indexMotor;
    private final int INDEX_MOTOR_CAN_ID = 10;

    private final DigitalInput beamBreak;
    private final int BEAM_BREAK_ID = 0;
    

    private final double INTAKE_SPEED = 0.60 * 12; //60% output
    private final double HOLD_SPEED = 0.01 * 12; //1% output
    private final double EJECT_SPEED = 1.0 * 12; //100% output

    public EndEffector(){
        indexMotor = new TalonFX(INDEX_MOTOR_CAN_ID);
        beamBreak = new DigitalInput(BEAM_BREAK_ID);
    }

    public void setEjectSpeed(){
        indexMotor.setControl(new VoltageOut(EJECT_SPEED));
    }

    public void setIntakeSpeed(){
        indexMotor.setControl(new VoltageOut(INTAKE_SPEED));
    }

    public void indexBrake(){
        indexMotor.setControl(new StaticBrake());
    }

    public void setHoldSpeed(){
        indexMotor.setControl(new VoltageOut(HOLD_SPEED));
    }

    public boolean getBeamBreakValue(){
        return beamBreak.get();
    }

}
