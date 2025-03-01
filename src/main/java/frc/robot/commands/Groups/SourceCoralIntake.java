package frc.robot.commands.Groups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.EndEffector.IntakeCoral;
import frc.robot.commands.Pivot.StationPosition;
import frc.robot.commands.Telescope.MoveToStation;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Telescope;

public class SourceCoralIntake extends SequentialCommandGroup{
    // private final EndEffector m_endEffector;
    // private final LED m_led;
    // private final Pivot m_pivot;
    public SourceCoralIntake(EndEffector m_endEffector, LED m_led, Pivot m_pivot, Telescope m_telescope){
        // this.m_endEffector = m_endEffector;
        // this.m_led = m_led;
        // this.m_pivot = m_pivot;

        addRequirements(m_endEffector, m_led, m_pivot);
        
        addCommands(new MoveToStation(m_telescope));
        addCommands(new ParallelDeadlineGroup(new IntakeCoral(m_endEffector), new StationPosition(m_pivot)), m_led.blinkPurple());
        addCommands(new ParallelCommandGroup(m_led.solidGreen(), new InstantCommand(() -> m_pivot.movetoAngle(0))));    
    }
}
