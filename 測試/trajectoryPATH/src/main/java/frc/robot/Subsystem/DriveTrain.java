
package frc.robot.Subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
//new pid
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

  VictorSPX rightmotor = new VictorSPX(4);
  VictorSPX rightmotorS = new VictorSPX(5);

  VictorSPX leftmotor = new VictorSPX(1);
  VictorSPX leftmotorS = new VictorSPX(0);

  AHRS ahrs = new AHRS(SPI.Port.kMXP);

  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.7);      // 輪子寬度
  DifferentialDriveOdometry odmetry = new DifferentialDriveOdometry(getHeading());

  Encoder lencoder = new Encoder(0, 1);
  Encoder rencoder = new Encoder(2, 3, true);

  Pose2d pose;

  SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(Constants.kS, Constants.kV, Constants.kA);

  PIDController lpidcontroller = new PIDController(
      Constants.kP, 0, 0);
  PIDController rpidcontroller = new PIDController(
      Constants.kP, 0, 0);

  public DriveTrain(){
    ahrs.reset();
    rencoder.reset();
    lencoder.reset();
    rencoder.setDistancePerPulse(Constants.distantsPerPulse);
    lencoder.setDistancePerPulse(Constants.distantsPerPulse);
    rightmotor.setInverted(true);
    rightmotorS.setInverted(true);
    rightmotorS.follow(rightmotor);
    leftmotorS.follow(leftmotor);
  }
 
  public void reset(){
    ahrs.reset();
  }
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-ahrs.getAngle());
  }

  public DifferentialDriveWheelSpeeds getSpeed() {
    SmartDashboard.putNumber("leftRate", lencoder.getRate());
    SmartDashboard.putNumber("rightRate", rencoder.getRate());
    return new DifferentialDriveWheelSpeeds(
      lencoder.getRate(), 
      rencoder.getRate()
      );
  
  }

  public  SimpleMotorFeedforward getFeedforward() {
    return feedForward;
  }

  public PIDController getlpidcontroller() {
    return lpidcontroller;
  }

  public PIDController getrpidcontroller() {
    return rpidcontroller;
  }

  public  DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public  void setOutput(double leftVolts, double rightVolts) {
    leftVolts /= 12;
    rightVolts /= 12;
    leftmotor.set(ControlMode.PercentOutput, leftVolts);
    rightmotor.set(ControlMode.PercentOutput, rightVolts);
    SmartDashboard.putNumber("leftOutput ", leftVolts );
    SmartDashboard.putNumber("rightOutput", rightVolts);
    //leftVolts / 12          -rightVolts / 12
  }
  
  public Pose2d getpose2d(){
    return pose;
  }
  public double getX(){
    return odmetry.getPoseMeters().getTranslation().getX();
  }
  public double getY(){
    return odmetry.getPoseMeters().getTranslation().getY();
  }
 
  @Override
  public void periodic() {
    pose = odmetry.update(getHeading(), lencoder.getDistance(), rencoder.getDistance());
    SmartDashboard.putNumber("x", odmetry.getPoseMeters().getTranslation().getX());
    SmartDashboard.putNumber("Y", getY());
    SmartDashboard.putNumber("perpulse", lencoder.getDistancePerPulse());
    SmartDashboard.putNumber("rightperpulse", rencoder.getDistancePerPulse());
    //distants
    SmartDashboard.putNumber("leftDistants", lencoder.getDistance());
    SmartDashboard.putNumber("rightDistants", rencoder.getDistance());
    SmartDashboard.putNumber("Yaw", ahrs.getYaw());
  }
//
}