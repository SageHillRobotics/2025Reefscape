package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;


public class Constants {
    public static final class VisionConstants{
        public static final Translation3d LEFT_CAM_TRANSLATION = new Translation3d(Units.inchesToMeters(-10.497), 
                                                                        Units.inchesToMeters(11.878), 
                                                                        Units.inchesToMeters(9.104));

    public static final Rotation3d LEFT_CAM_ROTATION =  new Rotation3d(0,   
                                                                        Units.degreesToRadians(18.7), 
                                                                        Units.degreesToRadians(-160));
    
    public static final Translation3d RIGHT_CAM_TRANSLATION = new Translation3d(Units.inchesToMeters(-10.497), 
                                                                        Units.inchesToMeters(-11.878),  
                                                                        Units.inchesToMeters(9.104));

    public static final Rotation3d RIGHT_CAM_ROTATION = new Rotation3d(0, 
                                                                        Units.degreesToRadians(18.7), 
                                                                        Units.degreesToRadians(160));
                                                                        
    public static final Transform3d LEFT_CAM_TRANSFORM = new Transform3d(LEFT_CAM_TRANSLATION, LEFT_CAM_ROTATION);
    public static final Transform3d RIGHT_CAM_TRANSFORM = new Transform3d(RIGHT_CAM_TRANSLATION, RIGHT_CAM_ROTATION);


    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }
}
