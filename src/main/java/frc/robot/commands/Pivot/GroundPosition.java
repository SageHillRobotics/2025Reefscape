package frc.robot.commands.Pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Pivot;

public class GroundPosition extends Command{
    private final Pivot m_pivot;
    public GroundPosition(Pivot m_pivot){
        this.m_pivot = m_pivot;
        addRequirements(m_pivot);
    }
    @Override
    public void initialize(){
        m_pivot.movetoAngle(Constants.PivotConstants.groundIntakeAngleDegrees);
    }
    @Override
    public boolean isFinished(){
        return m_pivot.atSetpoint();
    }
}