// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;


import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.Align.AutoAlign;
import frc.robot.commands.EndEffector.DropCoral;
import frc.robot.commands.EndEffector.IntakeCoral;
import frc.robot.commands.Groups.ScoreL2;
import frc.robot.commands.Groups.ScoreL3;
import frc.robot.commands.Groups.ScoreL4;
import frc.robot.commands.Groups.SourceCoralIntake;
import frc.robot.commands.Groups.Stow;
import frc.robot.commands.Pivot.ReefPosition;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Telescope;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandJoystick driver = new CommandJoystick(0);
    private final CommandGenericHID buttonBoard = new CommandGenericHID(1);
    private final CommandGenericHID auxButtonBoard = new CommandGenericHID(2);


    private final int strafeAxis = 0;
    private final int translationAxis = 1;

    private final Trigger cwButton = driver.button(6);
    private final Trigger ccwButton = driver.button(5);
    private final Trigger zeroGyro = driver.button(2);

    private final Trigger pivotReefPosition = driver.button(7);
    private final Trigger L2Position = driver.button(9);
    private final Trigger L3Position = driver.button(10);
    private final Trigger L4Position = driver.button(12);

    private final Trigger intakeCoral = driver.button(3);
    private final Trigger score = driver.button(1);
    private final Trigger sourceCoralIntake = driver.button(4);
    private final Trigger stow = driver.button(11);

    private final Trigger sourceLeftLineUp = auxButtonBoard.button(1);
    private final Trigger sourceRightLineUp = auxButtonBoard.button(2);

    private final Trigger aLineUp = buttonBoard.button(1);
    private final Trigger bLineUp = buttonBoard.button(2);
    private final Trigger cLineUp = buttonBoard.button(3);
    private final Trigger dLineUp = buttonBoard.button(4);
    private final Trigger eLineUp = buttonBoard.button(5);
    private final Trigger fLineUp = buttonBoard.button(6);
    private final Trigger gLineUp = buttonBoard.button(7);
    private final Trigger hLineUp = buttonBoard.button(8);
    private final Trigger iLineUp = buttonBoard.button(9);
    private final Trigger jLineUp = buttonBoard.button(10);


    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Pivot m_pivot = new Pivot();
    public final Telescope m_telescope = new Telescope();
    public final EndEffector m_endEffector = new EndEffector();
    public final LED m_led = new LED();

  	private final SendableChooser<Command> autoChooser;


    public RobotContainer() {


        NamedCommands.registerCommand("ScoreL3", new ScoreL3(m_endEffector, m_pivot, m_telescope, m_led));
        NamedCommands.registerCommand("ScoreL2", new ScoreL2(m_endEffector, m_pivot, m_telescope, m_led));
        NamedCommands.registerCommand("ScoreL4", new ScoreL4(m_endEffector, m_pivot, m_telescope, m_led));

        
        NamedCommands.registerCommand("ReefPosition", new ReefPosition(m_pivot));
        NamedCommands.registerCommand("IntakeCoral", new IntakeCoral(m_endEffector));
        NamedCommands.registerCommand("DropCoral", new DropCoral(m_endEffector));

        NamedCommands.registerCommand("AlignSourceRight", new AutoAlign(drivetrain, "sourceRight"));
        NamedCommands.registerCommand("AlignSourceLeft", new AutoAlign(drivetrain, "sourceLeft"));
        NamedCommands.registerCommand("AlignI", new AutoAlign(drivetrain, "I"));
        NamedCommands.registerCommand("AlignJ", new AutoAlign(drivetrain, "J"));
        NamedCommands.registerCommand("AlignK", new AutoAlign(drivetrain, "K"));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getRawAxis(translationAxis) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getRawAxis(strafeAxis) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        cwButton.getAsBoolean() && !ccwButton.getAsBoolean() ? -0.75 * MaxAngularRate :
                        (ccwButton.getAsBoolean() && !cwButton.getAsBoolean() ? 0.75 * MaxAngularRate : 0)
                    ) // Drive counterclockwise with negative X (left)
            )
        );

        // lineUp.toggleOnTrue(drivetrain.pathFind());
        sourceLeftLineUp.whileTrue(new AutoAlign(drivetrain, "sourceLeft"));
        sourceRightLineUp.whileTrue(new AutoAlign(drivetrain, "sourceRight"));

        aLineUp.whileTrue(new AutoAlign(drivetrain, "A"));
        bLineUp.whileTrue(new AutoAlign(drivetrain, "B"));
        cLineUp.whileTrue(new AutoAlign(drivetrain, "C"));
        dLineUp.whileTrue(new AutoAlign(drivetrain, "D"));
        eLineUp.whileTrue(new AutoAlign(drivetrain, "E"));
        fLineUp.whileTrue(new AutoAlign(drivetrain, "F"));
        gLineUp.whileTrue(new AutoAlign(drivetrain, "G"));
        hLineUp.whileTrue(new AutoAlign(drivetrain, "H"));
        iLineUp.whileTrue(new AutoAlign(drivetrain, "I"));
        jLineUp.whileTrue(new AutoAlign(drivetrain, "J"));


        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        zeroGyro.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        pivotReefPosition.toggleOnTrue(new ReefPosition(m_pivot));
        L2Position.onTrue(new ScoreL2(m_endEffector, m_pivot, m_telescope, m_led));
        L3Position.onTrue(new ScoreL3(m_endEffector, m_pivot, m_telescope, m_led));
        L4Position.onTrue(new ScoreL4(m_endEffector, m_pivot, m_telescope, m_led));

        intakeCoral.onTrue(new IntakeCoral(m_endEffector));
        score.onTrue(new DropCoral(m_endEffector));
        sourceCoralIntake.onTrue(new SourceCoralIntake(m_endEffector, m_led, m_pivot, m_telescope));
        stow.onTrue(new Stow(m_endEffector, m_pivot, m_telescope, m_led));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
