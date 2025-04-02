package frc.robot.commands.Groups;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Pivot;
import frc.robot.commands.EndEffector.MoveWristToStow;
import frc.robot.commands.EndEffector.MoveWristToStowWithCoral;
import frc.robot.commands.Pivot.StowPosition;
import frc.robot.commands.Telescope.MoveToStow;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Telescope;

public class Stow extends SequentialCommandGroup{
    public Stow(EndEffector m_endEffector, Pivot m_pivot, Telescope m_telescope, LED m_led){
        addRequirements(m_endEffector, m_pivot, m_telescope);
        

        addCommands(new MoveToStow(m_telescope));
        addCommands(new ParallelCommandGroup(new StowPosition(m_pivot), 
                                            new ConditionalCommand(new MoveWristToStowWithCoral(m_endEffector), new MoveWristToStow(m_endEffector), 
                                            () -> m_endEffector.hasCoral())));
    }
}