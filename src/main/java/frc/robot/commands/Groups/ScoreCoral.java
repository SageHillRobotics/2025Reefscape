package frc.robot.commands.Groups;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.EndEffector.DropCoral;
import frc.robot.commands.Pivot.ReefIntermediatePosition;
import frc.robot.commands.Pivot.StationPosition;
import frc.robot.commands.Telescope.MoveToStation;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Telescope;

public class ScoreCoral extends SequentialCommandGroup{
    public ScoreCoral(EndEffector m_endEffector, Pivot m_pivot, Telescope m_telescope, LED m_led){
        addRequirements(m_endEffector, m_pivot, m_telescope);

        addCommands(new ParallelDeadlineGroup(new DropCoral(m_endEffector), m_led.blinkGreen()));
        addCommands(new ReefIntermediatePosition(m_pivot));
        // addCommands(new ConditionalCommand(new ReefIntermediatePosition(m_pivot), new ReefPosition(m_pivot), (() -> m_telescope.getPosition() > 20)));
        addCommands(new ParallelDeadlineGroup(new MoveToStation(m_telescope), m_led.solidGreen()));
        addCommands(new StationPosition(m_pivot));

    }
}
