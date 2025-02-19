package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

public class Pivot extends SubsystemBase{
    private final TalonFX leftPivot;
    private final int LEFT_PIVOT_CAN_ID = 5;
    private final TalonFX rightPivot;
    private final int RIGHT_PIVOT_CAN_ID = 6;


    public Pivot(){
        leftPivot = new TalonFX(LEFT_PIVOT_CAN_ID);
        rightPivot = new TalonFX(RIGHT_PIVOT_CAN_ID);

        TalonFXConfigurator leaderConfig = leftPivot.getConfigurator();
    }
}
