/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Drivetrainconstants;
import com.ctre.phoenix.CANifier.LEDChannel;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
//import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.*;

public class Drivetrain extends SubsystemBase {
  WPI_TalonSRX Leftmaster    = new WPI_TalonSRX(Constants.Drivetrainconstants.LeftmasterID);
  WPI_TalonSRX Rightmaster   = new WPI_TalonSRX(Constants.Drivetrainconstants.RightmasterID);
  WPI_VictorSPX Leftfollower  = new WPI_VictorSPX(Constants.Drivetrainconstants.LeftfollowerID);
  WPI_VictorSPX Rightfollower = new WPI_VictorSPX(Constants.Drivetrainconstants.RightfollowerID);

  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain() {
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
/*
    Rightmaster.configRemoteFeedbackFilter(Leftmaster.getDeviceID(),					// Device ID of Source
												RemoteSensorSource.TalonSRX_SelectedSensor,	// Remote Feedback Source
												Drivetrainconstants.remotesensorID0,							// Source number [0, 1]
												Drivetrainconstants.timeoutMs);						// Configuration Timeout
		*/
    



  }
  
  

  @Override
  public void periodic() {


  }
}
