package frc.robot.commands.EndEffector;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;

public class IntakeCoral extends Command{
    private final EndEffector m_EndEffector;

    public IntakeCoral(EndEffector m_EndEffector){
        this.m_EndEffector = m_EndEffector;
        addRequirements(m_EndEffector);
    }
    @Override
    public void initialize(){
        m_EndEffector.setIntakeSpeed();
    }
    @Override
    public boolean isFinished(){
        Debouncer m_debouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);
        if (m_debouncer.calculate(m_EndEffector.getBeamBreakValue())){
            return true;
        }
        return false;
    }
    @Override
    public void end(boolean interrupted){
        m_EndEffector.setHoldSpeed();
    }
}
