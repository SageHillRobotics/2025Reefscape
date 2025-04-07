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
import frc.robot.commands.Groups.ClearHighAlgae;
import frc.robot.commands.Groups.Climb;
import frc.robot.commands.Groups.GroundCoralIntake;
import frc.robot.commands.Groups.ScoreCoral;
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
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.07) // Add a 10% deadband
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

    private final Trigger groundIntakeCoral = driver.button(3);
    private final Trigger score = driver.button(1);
    private final Trigger sourceCoralIntake = driver.button(4);
    private final Trigger stow = driver.button(11);

    private final Trigger clearHighAlgae = driver.button(8);

    private final Trigger sourceLeftLineUp = auxButtonBoard.button(1);
    private final Trigger sourceRightLineUp = auxButtonBoard.button(2);

    private final Trigger aTest = driver.povLeft();
    private final Trigger bTest = driver.povRight();

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
    private final Trigger kLineUp = buttonBoard.button(11);
    private final Trigger lLineUp = buttonBoard.button(12);



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
        NamedCommands.registerCommand("IntakeCoral", new SourceCoralIntake(m_endEffector, m_led, m_pivot, m_telescope));
        NamedCommands.registerCommand("DropCoral", new DropCoral(m_endEffector));
        NamedCommands.registerCommand("ScoreCoral", new ScoreCoral(m_endEffector, m_pivot, m_telescope, m_led));

        NamedCommands.registerCommand("AlignSourceRight", new AutoAlign(drivetrain, "sourceRight"));
        NamedCommands.registerCommand("AlignSourceLeft", new AutoAlign(drivetrain, "sourceLeft"));

        NamedCommands.registerCommand("AlignA", new AutoAlign(drivetrain, "A"));
        NamedCommands.registerCommand("AlignB", new AutoAlign(drivetrain, "B"));
        NamedCommands.registerCommand("AlignC", new AutoAlign(drivetrain, "C"));
        NamedCommands.registerCommand("AlignD", new AutoAlign(drivetrain, "D"));
        NamedCommands.registerCommand("AlignE", new AutoAlign(drivetrain, "E"));
        NamedCommands.registerCommand("AlignF", new AutoAlign(drivetrain, "F"));
        NamedCommands.registerCommand("AlignG", new AutoAlign(drivetrain, "G"));
        NamedCommands.registerCommand("AlignH", new AutoAlign(drivetrain, "H"));
        NamedCommands.registerCommand("AlignI", new AutoAlign(drivetrain, "I"));
        NamedCommands.registerCommand("AlignJ", new AutoAlign(drivetrain, "J"));
        NamedCommands.registerCommand("AlignK", new AutoAlign(drivetrain, "K"));
        NamedCommands.registerCommand("AlignL", new AutoAlign(drivetrain, "L"));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        
        configureBindings();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getRawAxis(translationAxis) * MaxSpeed)
                    .withVelocityY(-driver.getRawAxis(strafeAxis) * MaxSpeed)
                    .withRotationalRate(
                        cwButton.getAsBoolean() && !ccwButton.getAsBoolean() ? -0.75 * MaxAngularRate :
                        (ccwButton.getAsBoolean() && !cwButton.getAsBoolean() ? 0.75 * MaxAngularRate : 0)
                    )
            )
        );

        // lineUp.toggleOnTrue(drivetrain.pathFind());
        sourceLeftLineUp.onTrue(new AutoAlign(drivetrain, "sourceLeft"));
        sourceRightLineUp.onTrue(new AutoAlign(drivetrain, "sourceRight"));

        aTest.onTrue(new AutoAlign(drivetrain, "A"));
        bTest.onTrue(new AutoAlign(drivetrain, "B"));

        aLineUp.onTrue(new AutoAlign(drivetrain, "A"));
        bLineUp.onTrue(new AutoAlign(drivetrain, "B"));
        cLineUp.onTrue(new AutoAlign(drivetrain, "C"));
        dLineUp.onTrue(new AutoAlign(drivetrain, "D"));
        eLineUp.onTrue(new AutoAlign(drivetrain, "E"));
        fLineUp.onTrue(new AutoAlign(drivetrain, "F"));
        gLineUp.onTrue(new AutoAlign(drivetrain, "G"));
        hLineUp.onTrue(new AutoAlign(drivetrain, "H"));
        iLineUp.onTrue(new AutoAlign(drivetrain, "I"));
        jLineUp.onTrue(new AutoAlign(drivetrain, "J"));
        kLineUp.onTrue(new AutoAlign(drivetrain, "K"));
        lLineUp.onTrue(new AutoAlign(drivetrain, "L"));

        zeroGyro.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        pivotReefPosition.toggleOnTrue(new ReefPosition(m_pivot));
        L2Position.onTrue(new ScoreL2(m_endEffector, m_pivot, m_telescope, m_led));
        L3Position.onTrue(new ScoreL3(m_endEffector, m_pivot, m_telescope, m_led));
        L4Position.onTrue(new ScoreL4(m_endEffector, m_pivot, m_telescope, m_led));

        score.onTrue(new ScoreCoral(m_endEffector, m_pivot, m_telescope, m_led));

        sourceCoralIntake.toggleOnTrue(new SourceCoralIntake(m_endEffector, m_led, m_pivot, m_telescope));
        groundIntakeCoral.toggleOnTrue(new GroundCoralIntake(m_endEffector, m_led, m_pivot, m_telescope));

        stow.onTrue(new Stow(m_endEffector, m_pivot, m_telescope, m_led));

        clearHighAlgae.onTrue(new ClearHighAlgae(m_endEffector, m_led, m_pivot, m_telescope));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
