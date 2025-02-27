package frc.robot.commands.Groups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.EndEffector.IntakeCoral;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Pivot;

public class SourceCoralIntake extends SequentialCommandGroup{
    // private final EndEffector m_endEffector;
    // private final LED m_led;
    // private final Pivot m_pivot;
    public SourceCoralIntake(EndEffector m_endEffector, LED m_led, Pivot m_pivot){
        // this.m_endEffector = m_endEffector;
        // this.m_led = m_led;
        // this.m_pivot = m_pivot;

        addRequirements(m_endEffector, m_led, m_pivot);
        
        addCommands(new ParallelDeadlineGroup(new IntakeCoral(m_endEffector), new InstantCommand(() -> m_pivot.movetoAngle(30))), m_led.blinkPurple());
        addCommands(new ParallelCommandGroup(m_led.blinkGreen(), new InstantCommand(() -> m_pivot.movetoAngle(0))));    }
}
