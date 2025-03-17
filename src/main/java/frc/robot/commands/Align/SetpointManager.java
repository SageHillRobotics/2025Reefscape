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
        
        setpoints.put("sourceLeft1", new Pose2d(1.25, 6.9, Rotation2d.fromDegrees(126)));
        setpoints.put("sourceRigh1t", new Pose2d(1.25, .95, Rotation2d.fromDegrees(234)));

        setpoints.put("sourceLeft", new Pose2d(1.05, 7.1, Rotation2d.fromDegrees(126)));
        setpoints.put("sourceRight", new Pose2d(1.05, .95, Rotation2d.fromDegrees(234)));

        setpoints.put("A", new Pose2d(3.208, 4.2, Rotation2d.fromDegrees(180)));
        setpoints.put("B", new Pose2d(3.4, 3.85, Rotation2d.fromDegrees(180)));
        setpoints.put("C", new Pose2d(3.81, 3.18, Rotation2d.fromDegrees(240)));
        setpoints.put("D", new Pose2d(4.09, 3.00, Rotation2d.fromDegrees(240)));
        setpoints.put("E", new Pose2d(4.88, 3.01, Rotation2d.fromDegrees(300)));
        setpoints.put("F", new Pose2d(5.17, 3.17, Rotation2d.fromDegrees(300)));
        setpoints.put("G", new Pose2d(5.757, 3.85, Rotation2d.fromDegrees(0)));
        setpoints.put("H", new Pose2d(5.69, 4.18, Rotation2d.fromDegrees(0)));
        setpoints.put("I", new Pose2d(5.26, 5.047, Rotation2d.fromDegrees(60)));
        setpoints.put("J", new Pose2d(4.937, 5.145, Rotation2d.fromDegrees(60)));
        setpoints.put("K", new Pose2d(3.97, 5.19, Rotation2d.fromDegrees(120)));
        setpoints.put("L", new Pose2d(3.8, 5.02, Rotation2d.fromDegrees(120)));

        setpoints.put("A1", new Pose2d(3.4, 4.2, Rotation2d.fromDegrees(180)));
        setpoints.put("B1", new Pose2d(3.4, 3.85, Rotation2d.fromDegrees(180)));
        setpoints.put("C1", new Pose2d(3.62, 2.79, Rotation2d.fromDegrees(240)));
        setpoints.put("D1", new Pose2d(3.91, 2.65, Rotation2d.fromDegrees(240)));
        setpoints.put("E1", new Pose2d(5.06, 2.61, Rotation2d.fromDegrees(300)));
        setpoints.put("F1", new Pose2d(5.37, 2.76, Rotation2d.fromDegrees(300)));
        setpoints.put("G1", new Pose2d(6.01, 3.86, Rotation2d.fromDegrees(0)));
        setpoints.put("H1", new Pose2d(6.01, 4.18, Rotation2d.fromDegrees(0)));
        setpoints.put("I1", new Pose2d(5.35, 5.27, Rotation2d.fromDegrees(60)));
        setpoints.put("J1", new Pose2d(5.06, 5.427, Rotation2d.fromDegrees(60)));
        setpoints.put("K1", new Pose2d(3.93, 5.427, Rotation2d.fromDegrees(120)));
        setpoints.put("L1", new Pose2d(3.64, 5.26, Rotation2d.fromDegrees(120)));


        setpoints.put("barge", new Pose2d(8, 6, Rotation2d.fromDegrees(0)));
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
