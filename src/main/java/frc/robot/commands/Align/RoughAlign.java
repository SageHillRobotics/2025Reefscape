package frc.robot.commands.Align;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RoughAlign extends Command{
    private final CommandSwerveDrivetrain drivetrain;
    private final SetpointManager setpointManager;
    private final String targetKey;
    private Pose2d targetPose;
    private Command driveCommand;

    public RoughAlign(CommandSwerveDrivetrain drivetrain, SetpointManager setpointManager, String targetKey){
        this.drivetrain = drivetrain;
        this.setpointManager = setpointManager;
        this.targetKey = targetKey;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize(){
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        targetPose = setpointManager.getSetpoint(targetKey, alliance);
        driveCommand = drivetrain.driveToPose(targetPose);
        driveCommand.initialize();
    }

    @Override
    public void execute(){
        driveCommand.execute();
    }

    @Override
    public boolean isFinished(){
        return (driveCommand.isFinished() || distance(drivetrain.getState().Pose) < 1);
    }

    @Override
    public void end(boolean interrupted){
        driveCommand.end(interrupted);
    }

    public double distance(Pose2d pose){
        double curX = pose.getX();
        double curY = pose.getY();

        double targetX = targetPose.getX();
        double targetY = targetPose.getY();

        return Math.sqrt((Math.pow((curX - targetX), 2) + Math.pow((curY - targetY), 2)));

    }

}
