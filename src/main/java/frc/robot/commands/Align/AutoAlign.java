package frc.robot.commands.Align;


import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoAlign extends SequentialCommandGroup{
    private final SetpointManager setpointManager;
    public AutoAlign(CommandSwerveDrivetrain drivetrain, String position){

        this.setpointManager = new SetpointManager();
        addRequirements(drivetrain);
        addCommands(new RoughAlign(drivetrain, setpointManager, position + "1"));
        addCommands(new PreciseAlign(drivetrain, setpointManager, position));
        addCommands(new InstantCommand(() -> drivetrain.applyRequest(() -> new SwerveRequest.SwerveDriveBrake())));
    }
}





