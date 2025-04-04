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
        
        // setpoints.put("sourceLeft1", new Pose2d(1.25, 6.9, Rotation2d.fromDegrees(126)));
        // setpoints.put("sourceRight1", new Pose2d(1.25, .95, Rotation2d.fromDegrees(234)));

        setpoints.put("sourceLeft", new Pose2d(1.05, 7.1, Rotation2d.fromDegrees(126)));
        setpoints.put("sourceRight", new Pose2d(1.087, .89, Rotation2d.fromDegrees(234)));

        // setpoints.put("A", new Pose2d(3.4, 4.2, Rotation2d.fromDegrees(180))); //3.225x, 4.214y
        // setpoints.put("B", new Pose2d(3.4, 3.85, Rotation2d.fromDegrees(180))); //3.234x, 3.839y, -3.116
        // setpoints.put("C", new Pose2d(3.81, 3.18, Rotation2d.fromDegrees(240))); //3.683x, 3.031y, -2.121r
        // setpoints.put("D", new Pose2d(4.037, 2.868, Rotation2d.fromDegrees(240))); //4.023x, 2.844y, -2.049r

        setpoints.put("A", new Pose2d(3.225, 4.214, Rotation2d.fromDegrees(180))); //3.225x, 4.214y
        setpoints.put("B", new Pose2d(3.234, 3.839, Rotation2d.fromDegrees(180))); //3.234x, 3.839y, -3.116
        setpoints.put("C", new Pose2d(3.683, 3.031, Rotation2d.fromDegrees(240))); //3.683x, 3.031y, -2.121r
        setpoints.put("D", new Pose2d(4.023, 2.844, Rotation2d.fromDegrees(240))); //4.023x, 2.844y, -2.049r

        setpoints.put("E", new Pose2d(4.957, 2.86, Rotation2d.fromDegrees(300)));
        setpoints.put("F", new Pose2d(5.266, 3.024, Rotation2d.fromDegrees(300)));
        setpoints.put("G", new Pose2d(5.747, 3.863, Rotation2d.fromDegrees(0)));
        setpoints.put("H", new Pose2d(5.748, 4.195, Rotation2d.fromDegrees(0)));
        setpoints.put("I", new Pose2d(5.26, 5.037, Rotation2d.fromDegrees(60)));
        setpoints.put("J", new Pose2d(4.966, 5.204, Rotation2d.fromDegrees(60)));
        setpoints.put("K", new Pose2d(4.01, 5.196, Rotation2d.fromDegrees(120)));
        // setpoints.put("L", new Pose2d(3.8, 4.9, Rotation2d.fromDegrees(120))); //3.699x, 5.017y, 2.121r 
        setpoints.put("L", new Pose2d(3.699, 5.017, Rotation2d.fromDegrees(120)));


        // setpoints.put("A", new Pose2d(3.208, 4.2, Rotation2d.fromDegrees(180)));
        // setpoints.put("B", new Pose2d(3.4, 3.85, Rotation2d.fromDegrees(180)));
        // setpoints.put("C", new Pose2d(3.81, 3.18, Rotation2d.fromDegrees(240)));
        // setpoints.put("D", new Pose2d(4.09, 3.00, Rotation2d.fromDegrees(240)));
        // setpoints.put("E", new Pose2d(4.88, 3.01, Rotation2d.fromDegrees(300)));
        // setpoints.put("F", new Pose2d(5.17, 3.17, Rotation2d.fromDegrees(300)));
        // setpoints.put("G", new Pose2d(5.757, 3.85, Rotation2d.fromDegrees(0)));
        // setpoints.put("H", new Pose2d(5.69, 4.18, Rotation2d.fromDegrees(0)));
        // setpoints.put("I", new Pose2d(5.26, 5.047, Rotation2d.fromDegrees(60)));
        // setpoints.put("J", new Pose2d(4.937, 5.145, Rotation2d.fromDegrees(60)));
        // setpoints.put("K", new Pose2d(3.97, 5.19, Rotation2d.fromDegrees(120)));
        // setpoints.put("L", new Pose2d(3.8, 5.02, Rotation2d.fromDegrees(120)));

        // setpoints.put("A1", new Pose2d(3.4, 4.2, Rotation2d.fromDegrees(180)));
        // setpoints.put("B1", new Pose2d(3.4, 3.85, Rotation2d.fromDegrees(180)));
        // setpoints.put("C1", new Pose2d(3.62, 2.79, Rotation2d.fromDegrees(240)));
        // setpoints.put("D1", new Pose2d(3.91, 2.65, Rotation2d.fromDegrees(240)));
        // setpoints.put("E1", new Pose2d(5.06, 2.61, Rotation2d.fromDegrees(300)));
        // setpoints.put("F1", new Pose2d(5.37, 2.76, Rotation2d.fromDegrees(300)));
        // setpoints.put("G1", new Pose2d(6.01, 3.86, Rotation2d.fromDegrees(0)));
        // setpoints.put("H1", new Pose2d(6.01, 4.18, Rotation2d.fromDegrees(0)));
        // setpoints.put("I1", new Pose2d(5.35, 5.27, Rotation2d.fromDegrees(60)));
        // setpoints.put("J1", new Pose2d(5.06, 5.427, Rotation2d.fromDegrees(60)));
        // setpoints.put("K1", new Pose2d(3.93, 5.427, Rotation2d.fromDegrees(120)));
        // setpoints.put("L1", new Pose2d(3.64, 5.26, Rotation2d.fromDegrees(120)));


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
