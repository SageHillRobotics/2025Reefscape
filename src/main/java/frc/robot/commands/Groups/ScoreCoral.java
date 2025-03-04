package frc.robot.commands.Groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.EndEffector.DropCoral;
import frc.robot.commands.Pivot.StationPosition;
import frc.robot.commands.Telescope.MoveToStation;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Telescope;

public class ScoreCoral extends SequentialCommandGroup{
    public ScoreCoral(EndEffector m_endEffector, Pivot m_pivot, Telescope m_telescope, LED m_led){
        addRequirements(m_endEffector, m_pivot, m_telescope);

        addCommands(new DropCoral(m_endEffector));
        addCommands(new MoveToStation(m_telescope));
        addCommands(new StationPosition(m_pivot));

    }
}
