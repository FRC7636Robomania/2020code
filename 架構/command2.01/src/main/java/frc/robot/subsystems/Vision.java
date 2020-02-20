/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisCon;


public class Vision extends SubsystemBase {

  /**
   * Creates a new Vision.
   */
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-unicorn");
  private double x,y,area,dist;
  public Vision() {
    

  }
  public double getx(){
    return x;
  }

  public double getDist(){
    dist = (VisCon.targetheight-VisCon.limeheight)/Math.tan(Math.toRadians(VisCon.limeangle
    +y));
    return dist;
  }
  public double getarea(){
    return area;
  }

  @Override
  public void periodic() {
    
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    x =  tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);

    // This method will be called once per scheduler run
  }
}
