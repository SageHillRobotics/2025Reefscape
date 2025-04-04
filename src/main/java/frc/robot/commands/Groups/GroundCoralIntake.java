package frc.robot.commands.Groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.EndEffector.GroundIntake;
import frc.robot.commands.EndEffector.IndexCoralStageOne;
import frc.robot.commands.EndEffector.IndexCoralStageTwo;
import frc.robot.commands.EndEffector.MoveWristToGround;
import frc.robot.commands.EndEffector.WristJam;
import frc.robot.commands.Pivot.GroundPosition;
import frc.robot.commands.Telescope.MoveToGround;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Telescope;

public class GroundCoralIntake extends SequentialCommandGroup{
    public GroundCoralIntake(EndEffector m_endEffector, LED m_led, Pivot m_pivot, Telescope m_telescope){
        addRequirements(m_endEffector, m_led, m_pivot);
        
        addCommands(new ParallelCommandGroup(new MoveWristToGround(m_endEffector), new MoveToGround(m_telescope)));
        addCommands(new ParallelDeadlineGroup(new GroundIntake(m_endEffector), new GroundPosition(m_pivot), m_led.blinkPurple()));
        addCommands(new WristJam(m_endEffector));
        addCommands(new ParallelDeadlineGroup(new SequentialCommandGroup(new IndexCoralStageOne(m_endEffector), new IndexCoralStageTwo(m_endEffector)), m_led.blinkRed()));
        addCommands(new ParallelDeadlineGroup(new Stow(m_endEffector, m_pivot, m_telescope, m_led), m_led.blinkGreen()));
    }
}
