package frc.robot.commands.Groups;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.EndEffector.DropCoralAuto;
// import frc.robot.commands.EndEffector.MoveWristToGround;
import frc.robot.commands.EndEffector.MoveWristToIntermediate;
// import frc.robot.commands.Pivot.ReefIntermediatePosition;
// import frc.robot.commands.Pivot.StationPosition;
import frc.robot.commands.Telescope.MoveToStation;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Telescope;

public class ScoreCoralAuto extends SequentialCommandGroup{
    public ScoreCoralAuto(EndEffector m_endEffector, Pivot m_pivot, Telescope m_telescope, LED m_led){
        addRequirements(m_endEffector, m_pivot, m_telescope);

        addCommands(new ParallelDeadlineGroup(new DropCoralAuto(m_endEffector), m_led.blinkGreen()));
        // addCommands(new ReefIntermediatePosition(m_pivot));
        addCommands(new MoveWristToIntermediate(m_endEffector));
        // addCommands(new ConditionalCommand(new ReefIntermediatePosition(m_pivot), new ReefPosition(m_pivot), (() -> m_telescope.getPosition() > 20)));
        addCommands(new ParallelDeadlineGroup(new MoveToStation(m_telescope), m_led.solidGreen()));
        // addCommands(new StationPosition(m_pivot));
        addCommands(new Stow(m_endEffector, m_pivot, m_telescope, m_led));

    }
}
