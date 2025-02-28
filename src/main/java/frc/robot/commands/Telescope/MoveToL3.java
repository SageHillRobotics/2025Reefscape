package frc.robot.commands.Telescope;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Telescope;

public class MoveToL3 extends Command{
    private final Telescope m_telescope;
    public MoveToL3(Telescope m_telescope){
        this.m_telescope = m_telescope;
        addRequirements(m_telescope);
    }
    @Override
    public void initialize(){
        m_telescope.movetoPosition(4);
    }

    @Override
    public boolean isFinished(){
        return m_telescope.atSetpoint();
    }
    // @Override
    // public void end(boolean interrupted){
    //     m_telescope.movetoPosition(1);
    // }
}
