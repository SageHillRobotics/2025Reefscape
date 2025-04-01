package frc.robot.commands.EndEffector;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;

public class GroundIntake extends Command{
    private final EndEffector m_endEffector;
    private final Debouncer m_debouncer = new Debouncer(0.35, Debouncer.DebounceType.kBoth);
    private final Debouncer m_debouncer_back = new Debouncer(0.1, Debouncer.DebounceType.kBoth);


    public GroundIntake(EndEffector m_endEffector){
        this.m_endEffector = m_endEffector;
        addRequirements(m_endEffector);
    }
    @Override
    public void initialize(){
        m_endEffector.setGroundIntakeSpeed();
    }
    @Override
    public boolean isFinished(){
        if (m_debouncer.calculate(m_endEffector.getFrontBeamBreakValue() == false) || m_debouncer_back.calculate(!m_endEffector.getBackBeamBreakValue())){
            return true;
        }
        return false;
        // return !(m_endEffector.getFrontBeamBreakValue());
        // if (m_debouncer.calculate(m_endEffector.getStatorCurrent() > 75)){
        //     return true;
        // }
        // return false;
    }
    @Override
    public void end(boolean interrupted){
        m_endEffector.setHoldSpeed();
    }
}
