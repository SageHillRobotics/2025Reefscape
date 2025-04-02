package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.EndEffector;

public class MoveWristToL4 extends Command{
    private final EndEffector m_endEffector;
    public MoveWristToL4(EndEffector m_endEffector){
        this.m_endEffector   = m_endEffector;
        addRequirements(m_endEffector);
    }
    @Override
    public void initialize(){
        m_endEffector.wristToAngle(Constants.WristConstants.L4AngleDegrees);
    }

    @Override
    public boolean isFinished(){
        return m_endEffector.atSetpoint();
    }
}
