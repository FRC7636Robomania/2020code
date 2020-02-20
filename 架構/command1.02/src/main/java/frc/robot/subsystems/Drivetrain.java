/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants.DrCon;
import frc.robot.Setmotor;

public class Drivetrain extends SubsystemBase {
  

  PIDController lpidcontroller = new PIDController(
      DrCon.kP, 0, 0);
  PIDController rpidcontroller = new PIDController(
      DrCon.kP, 0, 0);

  SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(DrCon.kS, DrCon.kV, DrCon.kA);
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.7);      // 輪子寬度


  DifferentialDriveOdometry odometry;
  Pose2d pose;
  SupplyCurrentLimitConfiguration supplyCurrentLimitConfiguration = new SupplyCurrentLimitConfiguration(true, 40, 50, 1);
  Setmotor setmotor = new Setmotor();
  WPI_TalonSRX leftmas = new WPI_TalonSRX(DrCon.LeftmasterID);
  WPI_VictorSPX leftfol= new WPI_VictorSPX(DrCon.LeftfollowerID);
  WPI_TalonSRX rightmas = new WPI_TalonSRX(DrCon.RightmasterID);
  WPI_VictorSPX rightfol = new WPI_VictorSPX(DrCon.RightfollowerID);
  AHRS ahrs = new AHRS(SPI.Port.kMXP);
  double m_quickStopAccumulator = 0,leftout=0,rightout=0;

  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain() {
    try {

      ahrs = new AHRS(SPI.Port.kMXP);
      
    } catch (Exception e) {
      //TODO: handle exception
    }
    ahrs.reset();
    
    odometry = new DifferentialDriveOdometry(getHeading());
    
    setmotor.setmotor(leftmas, supplyCurrentLimitConfiguration, DrCon.kP, DrCon.kF, InvertType.None, DrCon.pidsolt, DrCon.Ramptime, DrCon.timeoutMs);
    setmotor.setmotor(rightmas, supplyCurrentLimitConfiguration, DrCon.kP, DrCon.kF, InvertType.InvertMotorOutput, DrCon.pidsolt, DrCon.Ramptime,DrCon.timeoutMs);
    setmotor.setmotorfol(leftfol, supplyCurrentLimitConfiguration, InvertType.FollowMaster,DrCon.timeoutMs);
    setmotor.setmotorfol(rightfol, supplyCurrentLimitConfiguration, InvertType.FollowMaster,DrCon.timeoutMs);
  }
  public void curvaturedrive(double xSpeed,double zRotation,boolean isQuickTurn){
    
    curvatureDrive(xSpeed, zRotation, isQuickTurn);
    rightmas.set(ControlMode.PercentOutput, rightout);
    leftmas.set(ControlMode.PercentOutput,leftout);
  }
  
  /**
   * Curvature drive method for differential drive platform.
   *
   * <p>The rotation argument controls the curvature of the robot's path rather than its rate of
   * heading change. This makes the robot more controllable at high speeds. Also handles the
   * robot's quick turn functionality - "quick turn" overrides constant-curvature turning for
   * turn-in-place maneuvers.
   *
   * @param xSpeed      The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
   * @param zRotation   The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is
   *                    positive.
   * @param isQuickTurn If set, overrides constant-curvature turning for
   *                    turn-in-place maneuvers.
   */
  @SuppressWarnings({"ParameterName", "PMD.CyclomaticgitComplexity"})
  public void curvatureDrive(double xSpeed, double zRotation, boolean isQuickTurn) {

    xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
    if(Math.abs(zRotation)<0.1){
      zRotation =0;
    }
    zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);

    double angularPower;
    boolean overPower;
    double m_quickStopAlpha =0.1;
    if (isQuickTurn) {
      if (Math.abs(xSpeed) < 0.1) {
        m_quickStopAccumulator = (1 - 0.1) * m_quickStopAccumulator
            + m_quickStopAlpha * MathUtil.clamp(zRotation, -1.0, 1.0) * 2;
      }
      
      overPower = true;
      angularPower = zRotation;
    } else {
      overPower = false;
      angularPower = Math.abs(xSpeed) * zRotation - m_quickStopAccumulator;

      if (m_quickStopAccumulator > 1) {
        m_quickStopAccumulator -= 1;
      } else if (m_quickStopAccumulator < -1) {
        m_quickStopAccumulator += 1;
      } else {
        m_quickStopAccumulator = 0.0;
      }
    }

    double leftMotorOutput = xSpeed + angularPower;
    double rightMotorOutput = xSpeed - angularPower;

    // If rotation is overpowered, reduce both outputs to within acceptable range
    if (overPower) {
      if (leftMotorOutput > 1.0) {
        rightMotorOutput -= leftMotorOutput - 1.0;
        leftMotorOutput = 1.0;
      } else if (rightMotorOutput > 1.0) {
        leftMotorOutput -= rightMotorOutput - 1.0;
        rightMotorOutput = 1.0;
      } else if (leftMotorOutput < -1.0) {
        rightMotorOutput -= leftMotorOutput + 1.0;
        leftMotorOutput = -1.0;
      } else if (rightMotorOutput < -1.0) {
        leftMotorOutput -= rightMotorOutput + 1.0;
        rightMotorOutput = -1.0;
      }
    }

    // Normalize the wheel speeds
    double maxMagnitude = Math.max(Math.abs(leftMotorOutput), Math.abs(rightMotorOutput));
    if (maxMagnitude > 1.0) {
      leftMotorOutput /= maxMagnitude;
      rightMotorOutput /= maxMagnitude;
    }

    leftout=  leftMotorOutput;
    rightout = rightMotorOutput;
    
  }
  

  



  @Override
  public void periodic() {

    pose = odometry.update(getHeading(), lencoder.getDistance(), rencoder.getDistance());

    Message();
    
  }

  public  void setOutput(double leftVolts, double rightVolts) {
    leftVolts /= 12;
    rightVolts /= 12;
    leftmas.set(ControlMode.PercentOutput, leftVolts);
    rightmas.set(ControlMode.PercentOutput, rightVolts);
    SmartDashboard.putNumber("leftOutput ", leftVolts );
    SmartDashboard.putNumber("rightOutput", rightVolts);
    //leftVolts / 12          -rightVolts / 12
  }

  public DifferentialDriveWheelSpeeds getSpeed() {
    SmartDashboard.putNumber("leftRate", leftmas.getSelectedSensorVelocity()*0.15);
    SmartDashboard.putNumber("rightRate", rencoder.getRate());
    return new DifferentialDriveWheelSpeeds(
      lencoder.getRate(), 
      rencoder.getRate()
      );
  
  }

  public double getMotorDistance(WPI_TalonSRX motor) {

    return motor.getSelectedSensorPosition()*DrCon.distantsPerPulse;
  }

  public double getMotorVelocity(WPI_TalonSRX motor) {

    return motor.getSelectedSensorVelocity()*DrCon.distantsPerPulse*10;
  }


  public void Message() {
    
    SmartDashboard.putNumber("x", odometry.getPoseMeters().getTranslation().getX());
    SmartDashboard.putNumber("Y", getY());
    SmartDashboard.putNumber("perpulse", lencoder.getDistancePerPulse());
    SmartDashboard.putNumber("rightperpulse", rencoder.getDistancePerPulse());
    //distants
    SmartDashboard.putNumber("leftDistants", lencoder.getDistance());
    SmartDashboard.putNumber("rightDistants", rencoder.getDistance());
    SmartDashboard.putNumber("Yaw", ahrs.getYaw());

  }


  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-ahrs.getAngle());
  }

  public PIDController getlpidcontroller() {
    return lpidcontroller;
  }

  public PIDController getrpidcontroller() {
    return rpidcontroller;
  }

  public  SimpleMotorFeedforward getFeedforward() {
    return feedForward;
  }
  
  public  DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  
  public Pose2d getpose2d(){
    return pose;
  }
  public double getX(){
    return odometry.getPoseMeters().getTranslation().getX();
  }
  public double getY(){
    return odometry.getPoseMeters().getTranslation().getY();
  }

}