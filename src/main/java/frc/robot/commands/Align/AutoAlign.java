package frc.robot.commands.Align;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoAlign extends SequentialCommandGroup{
    private final SetpointManager setpointManager;
    public AutoAlign(CommandSwerveDrivetrain drivetrain, Character position){

        this.setpointManager = new SetpointManager();
        addRequirements(drivetrain);
        addCommands(new RoughAlign(drivetrain, setpointManager, position));
        addCommands(new PreciseAlign(drivetrain, setpointManager, position));
    }
}





