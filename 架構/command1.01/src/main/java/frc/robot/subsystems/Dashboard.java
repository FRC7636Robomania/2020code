/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.sql.Driver;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Dashboard extends SubsystemBase {
  String drivemode;
  /**
   * 
   * Creates a new Dashboard.
   */
  public Dashboard() {

  }
  public void Drivemode(int mode){
    switch (mode) {
      case 0:
        drivemode = "CurvatureDrive"; 
        break;
      case 1:
        drivemode = "Auto"; 
        break;
      case 2:
        drivemode = "Auxiliary"; 
        break;
      default:
        break;
    }
    SmartDashboard.putString("DriveMode", drivemode);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
