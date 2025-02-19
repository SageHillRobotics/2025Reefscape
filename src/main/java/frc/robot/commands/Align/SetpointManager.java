package frc.robot.commands.Align;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class SetpointManager {
    private final Map<String, Pose2d> setpoints = new HashMap<>();
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    private final NetworkTable driveStateTable = inst.getTable("DriveState");
    private final StructPublisher<Pose2d> setpointPublisher = driveStateTable.getStructTopic("Pose Setpoint", Pose2d.struct).publish();

    public SetpointManager() {
        //Pose setpoints on the Blue alliance side, units in meters, meters, degrees
        setpoints.put("Source_2", new Pose2d(1.05, .95, Rotation2d.fromRadians(.95)));

        setpoints.put("A", new Pose2d(3.2, 4.2, Rotation2d.fromDegrees(0)));
        setpoints.put("B", new Pose2d(3.2, 3.85, Rotation2d.fromDegrees(0)));

        //fill in later
        // setpoints.put('C', new Setpoint(0, 0, 0));
        // setpoints.put('D', new Setpoint(0, 0, 0));
        // setpoints.put('E', new Setpoint(0, 0, 0));
        // setpoints.put('F', new Setpoint(5.33, 3.95, 120));
    }

    public Pose2d getSetpoint(String key, Alliance alliance){
        Pose2d output = setpoints.get(key);

        if (alliance == Alliance.Red){
            output = FlippingUtil.flipFieldPose(output);
        }
        setpointPublisher.set(output);

        return output;
    }
}
