package frc.robot.commands.Groups;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Pivot;
import frc.robot.commands.EndEffector.MoveWristToGround;
import frc.robot.commands.EndEffector.MoveWristToL3;
import frc.robot.commands.Pivot.ReefPosition;
import frc.robot.commands.Telescope.MoveToL2;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Telescope;

public class ScoreL2 extends SequentialCommandGroup{
    public ScoreL2(EndEffector m_endEffector, Pivot m_pivot, Telescope m_telescope, LED m_led){
        addRequirements(m_endEffector, m_pivot, m_telescope);
        
        addCommands(new MoveWristToGround(m_endEffector));
        addCommands(new ReefPosition(m_pivot));
        addCommands(new ParallelDeadlineGroup(new WaitCommand(5), new MoveToL2(m_telescope)));
        addCommands(new MoveWristToL3(m_endEffector));
    }
}