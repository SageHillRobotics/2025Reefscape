package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;

public class IndexCoralStageTwo extends Command{
    private final EndEffector m_endEffector;
    public IndexCoralStageTwo(EndEffector m_endEffector){
        this.m_endEffector = m_endEffector;
        addRequirements(m_endEffector);
    }
    @Override
    public void execute(){
        m_endEffector.feedUp();
    }
    @Override
    public boolean isFinished(){
        return !m_endEffector.getBackBeamBreakValue() && !m_endEffector.getFrontBeamBreakValue();
    }
    @Override
    public void end(boolean interrupted){
        m_endEffector.setHoldSpeed();
    }
}
