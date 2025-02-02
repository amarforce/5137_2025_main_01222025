package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class VisionConstants {
    public static final Transform3d robotToFrontCamera = new Transform3d(new Translation3d(0.35, 0, 0.2), new Rotation3d(0, -Math.PI/6, 0));
    public static final Transform3d robotToLeftCamera = new Transform3d(new Translation3d(0, 0.35, 0.2), new Rotation3d(0, -Math.PI/6, Math.PI/2));
    public static final Transform3d robotToFrontObjectCamera = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
}
