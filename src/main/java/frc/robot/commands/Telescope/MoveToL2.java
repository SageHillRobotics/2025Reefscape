package frc.robot.commands.Telescope;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Telescope;

public class MoveToL2 extends Command{
    private final Telescope m_telescope;
    public MoveToL2(Telescope m_telescope){
        this.m_telescope = m_telescope;
        addRequirements(m_telescope);
    }
    @Override
    public void initialize(){
        m_telescope.movetoPosition(Constants.TelescopeConstants.L2Rotations);
    }

    @Override
    public boolean isFinished(){
        return m_telescope.atSetpoint();
    }
}
