package frc.robot.commands.Pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Pivot;

public class ReefPosition extends Command{
    private final Pivot pivot;
    public ReefPosition(Pivot pivot){
        this.pivot = pivot;
        addRequirements(pivot);
    }
    @Override
    public void initialize(){
        pivot.movetoAngle(Constants.PivotConstants.reefAngleDegrees);
    }
    @Override
    public void end(boolean interrupted){
        pivot.movetoAngle((Constants.PivotConstants.stowAngleDegrees));
    }
}
