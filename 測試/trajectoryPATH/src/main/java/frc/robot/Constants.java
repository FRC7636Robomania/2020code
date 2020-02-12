package frc.robot;

// import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Units;

public class Constants {
  public static double rotationPerPulse = 2048;
  public static double distantsPerPulse = 2 * Math.PI * Units.inchesToMeters(3) / rotationPerPulse;
    //圓周長　／　分辨率（解析度）（一圈的脈衝數）
      
  public static final double kS = 1.07;

  public static final double kV = 0.365;

  public static final double kA = 0.008;

  public static final double kP = 5.4;

  public static final String path = "paths/hi.wpilib.json";
   
}