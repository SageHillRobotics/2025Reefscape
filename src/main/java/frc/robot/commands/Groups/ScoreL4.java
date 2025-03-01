package frc.robot.commands.Groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Pivot;
import frc.robot.commands.Pivot.ReefPosition;
import frc.robot.commands.Telescope.MoveToL4;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Telescope;

public class ScoreL4 extends SequentialCommandGroup{
    public ScoreL4(EndEffector m_endEffector, Pivot m_pivot, Telescope m_telescope, LED m_led){
        addRequirements(m_endEffector, m_pivot, m_telescope);
        
        addCommands(new ReefPosition(m_pivot));
        addCommands(new MoveToL4(m_telescope));
    }
}