package frc.robot.commands.EndEffector;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;

public class IntakeCoral extends Command{
    private final EndEffector m_endEffector;

    public IntakeCoral(EndEffector m_endEffector){
        this.m_endEffector = m_endEffector;
        addRequirements(m_endEffector);
    }
    @Override
    public void initialize(){
        m_endEffector.setIntakeSpeed();
    }
    @Override
    public boolean isFinished(){
        Debouncer m_debouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);
        if (m_debouncer.calculate(m_endEffector.getBeamBreakValue() == false)){
            return true;
        }
        return false;
    }
    @Override
    public void end(boolean interrupted){
        m_endEffector.setHoldSpeed();
    }
}
