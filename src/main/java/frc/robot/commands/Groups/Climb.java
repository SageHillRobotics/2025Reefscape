package frc.robot.commands.Groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Pivot;
import frc.robot.commands.Pivot.ClimbPosition;
import frc.robot.commands.Telescope.MoveToL2;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Telescope;

public class Climb extends SequentialCommandGroup{
    public Climb(EndEffector m_endEffector, Pivot m_pivot, Telescope m_telescope, LED m_led){
        addRequirements(m_endEffector, m_pivot, m_telescope);
        
        addCommands(new MoveToL2(m_telescope));
        addCommands(new ClimbPosition(m_pivot));
    }
}