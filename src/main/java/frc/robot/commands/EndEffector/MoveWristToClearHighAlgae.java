package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.EndEffector;

public class MoveWristToClearHighAlgae extends Command{
    private final EndEffector m_endEffector;
    public MoveWristToClearHighAlgae(EndEffector m_endEffector){
        this.m_endEffector   = m_endEffector;
        addRequirements(m_endEffector);
    }
    @Override
    public void initialize(){
        m_endEffector.wristToAngle(Constants.WristConstants.clearHighAlgaeAngleDegrees);
    }

    @Override
    public boolean isFinished(){
        return m_endEffector.atSetpoint();
    }
}
