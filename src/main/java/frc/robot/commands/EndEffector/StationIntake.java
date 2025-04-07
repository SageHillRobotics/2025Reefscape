package frc.robot.commands.EndEffector;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;

public class StationIntake extends Command{
    private final EndEffector m_endEffector;
    private final Debouncer m_debouncer = new Debouncer(0.25, Debouncer.DebounceType.kBoth);
    // private final Debouncer m_debouncer_back = new Debouncer(0.01, Debouncer.DebounceType.kBoth);


    public StationIntake(EndEffector m_endEffector){
        this.m_endEffector = m_endEffector;
        addRequirements(m_endEffector);
    }
    @Override
    public void initialize(){
        m_endEffector.setStationIntakeSpeed();
    }
    @Override
    public boolean isFinished(){
        if (!(m_endEffector.getFrontBeamBreakValue()) || (!m_endEffector.getBackBeamBreakValue())){
            return true;
        }
        return false;
        // return m_debouncer.calculate(!(m_endEffector.getFrontBeamBreakValue()));
        //CHANGE BACK
        // return !m_endEffector.getBackBeamBreakValue();
    }
    @Override
    public void end(boolean interrupted){
        m_endEffector.setHoldSpeed();
    }
}
