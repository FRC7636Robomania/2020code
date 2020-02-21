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
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants.DrCon;
import frc.robot.Setmotor;

public class Drivetrain extends SubsystemBase {
  SupplyCurrentLimitConfiguration supplyCurrentLimitConfiguration = new SupplyCurrentLimitConfiguration(true, 40, 50, 1);
  Setmotor setmotor = new Setmotor();
  WPI_TalonFX leftmas = new WPI_TalonFX(DrCon.LeftmasterID);
  WPI_TalonFX leftfol= new WPI_TalonFX(DrCon.LeftfollowerID);
  WPI_TalonFX rightmas = new WPI_TalonFX(DrCon.RightmasterID);
  WPI_TalonFX rightfol = new WPI_TalonFX(DrCon.RightfollowerID);
  AHRS ahrs = new AHRS(SPI.Port.kMXP);
  double disterr;
  double m_quickStopAccumulator = 0,leftout=0,rightout=0;

  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain() {
    
    setmotor.setmotor(leftmas, supplyCurrentLimitConfiguration, DrCon.kP, DrCon.kF, InvertType.None, DrCon.pidsolt, DrCon.Ramptime, DrCon.timeoutMs);
    setmotor.setmotor(rightmas, supplyCurrentLimitConfiguration, DrCon.kP, DrCon.kF, InvertType.InvertMotorOutput, DrCon.pidsolt, DrCon.Ramptime,DrCon.timeoutMs);
    setmotor.setmotorfol(leftfol, supplyCurrentLimitConfiguration, InvertType.FollowMaster,DrCon.timeoutMs);
    setmotor.setmotorfol(rightfol, supplyCurrentLimitConfiguration, InvertType.FollowMaster,DrCon.timeoutMs);
  }
  public void distaim(double distpoint){
    disterr = distpoint;

  }
  public void curvaturedrive(double xSpeed,double zRotation,boolean isQuickTurn){
    
    curvatureDrive(xSpeed+disterr, zRotation, isQuickTurn);
    rightmas.set(ControlMode.PercentOutput, rightout);
    leftmas.set(ControlMode.PercentOutput,leftout);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
}