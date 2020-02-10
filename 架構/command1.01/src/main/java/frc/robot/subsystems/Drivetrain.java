/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  int mode=0;
  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain() {


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
    

    // This method will be called once per scheduler run
  }
}
