package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;


public class Constants {
    public static final class VisionConstants{
        public static final Translation3d CAM_TRANSLATION = new Translation3d(Units.inchesToMeters(2.985), 
                                                                        Units.inchesToMeters(5), 
                                                                        Units.inchesToMeters(14.725));

    public static final Rotation3d CAM_ROTATION =  new Rotation3d(0, Units.degreesToRadians(-19.18), Units.degreesToRadians(-17.38));
    
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }
}
