package frc.robot.commands.Pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;

public class ReefPosition extends Command{
    private final Pivot pivot;
    public ReefPosition(Pivot pivot){
        this.pivot = pivot;
        addRequirements(pivot);
    }
    @Override
    public void initialize(){
        pivot.movetoAngle(93);
    }
    @Override
    public void end(boolean interrupted){
        pivot.movetoAngle((0));
    }
}
