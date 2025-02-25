package frc.robot.commands.Groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Pivot;
import frc.robot.commands.EndEffector.DropCoral;
import frc.robot.commands.Pivot.ReefPosition;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Telescope;

public class ScoreL3 extends SequentialCommandGroup{
    public ScoreL3(EndEffector m_endEffector, Pivot m_pivot, Telescope m_telescope){
        addRequirements(m_endEffector, m_pivot, m_telescope);

        addCommands(m_pivot.runOnce(() -> new ReefPosition(m_pivot)));
        addCommands(new WaitUntilCommand(m_pivot::atSetpoint));
        addCommands(m_telescope.runOnce(() -> new ScoreL3(m_endEffector, m_pivot, m_telescope)));
        addCommands(new WaitUntilCommand(m_telescope::atSetpoint));
        addCommands(new DropCoral(m_endEffector));
    }
}