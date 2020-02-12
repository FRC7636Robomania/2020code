/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
public class Robot extends TimedRobot {
  RobotContainer container = new RobotContainer();

  @Override
  public void robotInit() {
  }

  @Override
  public void autonomousInit() {
    try {
      container.getAutonomousCommand().schedule();
    } catch (Exception e) {
      System.out.println("error");
    }
    
  }

  @Override
  public void autonomousPeriodic() {
    
    CommandScheduler.getInstance().run();
    
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
   
  }

  @Override
  public void testPeriodic() {
  }
}