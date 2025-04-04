package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;

public class DropCoralAuto extends Command{
    
    private final EndEffector m_endEffector;
    private double startingTime;

    public DropCoralAuto(EndEffector m_endEffector){
        this.m_endEffector = m_endEffector;
        addRequirements(m_endEffector);
    }
    @Override
    public void initialize(){
        startingTime = System.currentTimeMillis();
        m_endEffector.setEjectSpeed();
    }
    @Override
    public boolean isFinished(){
        return (System.currentTimeMillis() - startingTime > 800);
    }
    @Override
    public void end(boolean interrupted){
        m_endEffector.indexBrake();
    }
}
