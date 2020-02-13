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

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
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






  }
  
  
  public void CurvatureDrive(double Joystick){
    SmartDashboard.putNumber("runsubmethodcura",Joystick);
    

  }
  public void AutoDrive(){

  }
  public void AuxiliaryDrive(){

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Right", Rightmaster.getSelectedSensorPosition());
    SmartDashboard.putNumber("Left", Leftmaster.getSelectedSensorPosition());
    

    // This method will be called once per scheduler run
  }
}
