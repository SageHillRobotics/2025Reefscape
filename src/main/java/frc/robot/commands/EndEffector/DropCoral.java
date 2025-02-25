package frc.robot.commands.EndEffector;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;

public class DropCoral extends Command{
    private final EndEffector m_endEffector;
    public DropCoral(EndEffector m_endEffector){
        this.m_endEffector = m_endEffector;
        addRequirements(m_endEffector);
    }
    @Override
    public void initialize(){
        m_endEffector.setEjectSpeed();
    }
    @Override
    public boolean isFinished(){
        Debouncer m_debouncer = new Debouncer(0.2, Debouncer.DebounceType.kBoth);
        if (m_debouncer.calculate(m_endEffector.getBeamBreakValue())){
            return true;
        }
        return false;
    }
}
