package frc.robot.commands.Align;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;


public class PreciseAlign extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    // private final Pose2d targetPose;
    private final SetpointManager setpointManager;
    private String targetKey;
    private Pose2d targetPose;

    private final PIDController xPID;
    private final PIDController yPID;
    private final PIDController thetaPID;

    private static final double kXTolerance = 0.01;      // in meters
    private static final double kYTolerance = 0.01;      // in meters
    private static final double kThetaTolerance = Math.toRadians(2);  // in radians

    /**
     * @param drivetrain The swerve drivetrain subsystem.
     * @param targetPose The desired target pose.
     */
    public PreciseAlign(CommandSwerveDrivetrain drivetrain, SetpointManager setpointManager, String targetKey) {
        this.drivetrain = drivetrain;
        this.setpointManager = setpointManager;
        this.targetKey = targetKey;

        xPID = new PIDController(4.0, 0.0, 0.1);
        yPID = new PIDController(4.0, 0.0, 0.1);
        thetaPID = new PIDController(4.0, 0.0, 0.1);

        xPID.setTolerance(kXTolerance);
        yPID.setTolerance(kYTolerance);
        thetaPID.setTolerance(kThetaTolerance);
        thetaPID.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivetrain);
}

    @Override
    public void initialize() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        targetPose = setpointManager.getSetpoint(targetKey, alliance);

        xPID.reset();
        yPID.reset();
        thetaPID.reset();

    }

    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getState().Pose;

        double xSpeed = xPID.calculate(currentPose.getTranslation().getX(), targetPose.getTranslation().getX());
        double ySpeed = yPID.calculate(currentPose.getTranslation().getY(), targetPose.getTranslation().getY());
        double thetaSpeed = thetaPID.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

        drivetrain.applyRequest(() ->
            drive.withVelocityX(xSpeed)
                .withVelocityY(ySpeed)
                .withRotationalRate(thetaSpeed)
        );
    }
    @Override
    public boolean isFinished() {
        return xPID.atSetpoint() && yPID.atSetpoint() && thetaPID.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        new InstantCommand(() -> drivetrain.applyRequest(() -> new SwerveRequest.SwerveDriveBrake())).schedule();
    }
}
