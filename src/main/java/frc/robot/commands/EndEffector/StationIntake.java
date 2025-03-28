package frc.robot.commands.EndEffector;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;

public class StationIntake extends Command{
    private final EndEffector m_endEffector;
    private final Debouncer m_debouncer = new Debouncer(0.05, Debouncer.DebounceType.kBoth);

    public StationIntake(EndEffector m_endEffector){
        this.m_endEffector = m_endEffector;
        addRequirements(m_endEffector);
    }
    @Override
    public void initialize(){
        m_endEffector.setGroundIntakeSpeed();
    }
    @Override
    public boolean isFinished(){
        // if (m_debouncer.calculate(m_endEffector.getBeamBreakValue() == false)){
        //     return true;
        // }
        // return false;
        return !(m_endEffector.getBeamBreakValue());
    }
    @Override
    public void end(boolean interrupted){
        m_endEffector.setHoldSpeed();
    }
}
