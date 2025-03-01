package frc.robot.commands.Groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Pivot;
import frc.robot.commands.Pivot.ReefPosition;
import frc.robot.commands.Telescope.MoveToL3;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Telescope;

public class ScoreL3 extends SequentialCommandGroup{
    public ScoreL3(EndEffector m_endEffector, Pivot m_pivot, Telescope m_telescope, LED m_led){
        addRequirements(m_endEffector, m_pivot, m_telescope);
        
        addCommands(new ReefPosition(m_pivot));
        addCommands(new MoveToL3(m_telescope));
    }
}