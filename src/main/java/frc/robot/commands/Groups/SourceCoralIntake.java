package frc.robot.commands.Groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.EndEffector.MoveWristToStation;
import frc.robot.commands.EndEffector.StationIntake;
import frc.robot.commands.Pivot.ReefPosition;
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
        
        addCommands(new ParallelCommandGroup(new MoveWristToStation(m_endEffector), new MoveToStation(m_telescope)));
        addCommands(new ParallelDeadlineGroup(new StationIntake(m_endEffector), new StationPosition(m_pivot), m_led.blinkPurple()));
        addCommands(new ParallelDeadlineGroup(new ReefPosition(m_pivot), m_led.blinkGreen()));
        // addCommands(new ParallelCommandGroup(m_led.blinkGreen(), new InstantCommand(() -> m_pivot.movetoAngle(0))));    
    }
}
