package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;

public class DropCoral extends Command{
    private final EndEffector m_EndEffector;
    public DropCoral(EndEffector m_EndEffector){
        this.m_EndEffector = m_EndEffector;
        addRequirements(m_EndEffector);
    }
    @Override
    public void initialize(){
        m_EndEffector.setEjectSpeed();
    }
    @Override
    public boolean isFinished(){
        Debouncer m_debouncer = new Debouncer(0.2, Debouncer.DebounceType.kBoth);
        if (m_debouncer.calculate(m_EndEffector.getBeamBreakValue() == false)){
            return true;
        }
        return false;
    }
}
