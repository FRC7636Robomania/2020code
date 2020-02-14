/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Constants.Drivetrainconstants;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {  
  WPI_TalonSRX Leftmaster    = new WPI_TalonSRX(Constants.Drivetrainconstants.LeftmasterID);
  WPI_TalonSRX Rightmaster   = new WPI_TalonSRX(Constants.Drivetrainconstants.RightmasterID);
  WPI_VictorSPX Leftfollower  = new WPI_VictorSPX(Constants.Drivetrainconstants.LeftfollowerID);
  WPI_VictorSPX Rightfollower = new WPI_VictorSPX(Constants.Drivetrainconstants.RightfollowerID);

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    
    Leftmaster.configFactoryDefault();
    Rightmaster.configFactoryDefault();
    Leftfollower.configFactoryDefault();
    Rightfollower.configFactoryDefault();
    
    Leftmaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0,Constants.Drivetrainconstants.timeoutMs );
    Rightmaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, Constants.Drivetrainconstants.timeoutMs);

    Leftmaster.setInverted(false);
    Rightmaster.setInverted(true);
    Leftfollower.setInverted(InvertType.FollowMaster);
    Rightfollower.setInverted(InvertType.FollowMaster);

    Leftfollower.follow(Leftmaster);
    Rightfollower.follow(Rightmaster);

    Leftmaster.configMotionCruiseVelocity(Drivetrainconstants.MaxSpeed,10);
    Rightmaster.configMotionCruiseVelocity(Drivetrainconstants.MaxSpeed,10);
    Leftmaster.configMotionAcceleration(Drivetrainconstants.MaxAcc, 10);
    Rightmaster.configMotionAcceleration(Drivetrainconstants.MaxAcc,10);

    Rightmaster.enableCurrentLimit(true);
    Leftmaster.enableCurrentLimit(true);
    Rightmaster.configContinuousCurrentLimit(Constants.Drivetrainconstants.MaxAmp);
    
    Rightmaster.enableVoltageCompensation(true);
    Leftmaster.enableVoltageCompensation(true);

    Rightmaster.configVoltageCompSaturation(11.3);
    Leftmaster.configVoltageCompSaturation(11.3);

    Leftmaster.configAllowableClosedloopError(0, 10, 10);
    Rightmaster.configAllowableClosedloopError(0, 10, 10);
    
    Leftmaster.config_kP(Drivetrainconstants.pidsolt, Drivetrainconstants.kPdrive,Drivetrainconstants.timeoutMs);
    Rightmaster.config_kP(Drivetrainconstants.pidsolt, Drivetrainconstants.kPdrive,Drivetrainconstants.timeoutMs);
    Leftmaster.config_kF(Drivetrainconstants.pidsolt, Drivetrainconstants.kFdrive,Drivetrainconstants.timeoutMs);
    Rightmaster.config_kF(Drivetrainconstants.pidsolt, Drivetrainconstants.kFdrive,Drivetrainconstants.timeoutMs);
    Leftmaster.config_kD(Drivetrainconstants.pidsolt, Drivetrainconstants.kDdrive,Drivetrainconstants.timeoutMs);
    Rightmaster.config_kD(Drivetrainconstants.pidsolt, Drivetrainconstants.kDdrive,Drivetrainconstants.timeoutMs);
    Leftmaster.config_kI(Drivetrainconstants.pidsolt, Drivetrainconstants.kIdrive,Drivetrainconstants.timeoutMs);
    Rightmaster.config_kI(Drivetrainconstants.pidsolt, Drivetrainconstants.kIdrive,Drivetrainconstants.timeoutMs);

    Rightmaster.configNeutralDeadband(Drivetrainconstants.deadband);
    Leftmaster.configNeutralDeadband(Drivetrainconstants.deadband);

    Leftmaster.setNeutralMode(NeutralMode.Brake);
    Rightmaster.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
