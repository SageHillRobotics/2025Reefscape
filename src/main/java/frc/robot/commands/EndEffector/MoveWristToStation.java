package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.EndEffector;

public class MoveWristToStation extends Command{
    private final EndEffector m_endEffector;
    public MoveWristToStation(EndEffector m_endEffector){
        this.m_endEffector   = m_endEffector;
        addRequirements(m_endEffector);
    }
    @Override
    public void initialize(){
        m_endEffector.wristToAngle(Constants.WristConstants.stationAngleDegrees);
    }

    @Override
    public boolean isFinished(){
        return m_endEffector.atSetpoint();
    }
}
