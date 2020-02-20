/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Powercell;
import frc.robot.subsystems.TrajectorySystem;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public final static  Drivetrain m_drivetrain= new Drivetrain();
  private final Powercell m_powercell = new Powercell();
  Joystick joystick = new Joystick(0);
  public static TrajectorySystem trajectory;
  Joystick Driver = new Joystick(2);
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  JoystickButton turnButton = new JoystickButton(joystick,1);

  private final TrajectorySystem m_tTrajectory = new TrajectorySystem();

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_drivetrain.setDefaultCommand(new RunCommand(()->
      m_drivetrain.curvaturedrive(-joystick.getY(), joystick.getZ(),joystick.getRawButton(1)),
      m_drivetrain));
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(joystick,5).whenPressed(new InstantCommand(m_powercell::spinup,m_powercell));
    new JoystickButton(joystick, 6).whenPressed(new InstantCommand(m_powercell::flystop,m_powercell));
    new JoystickButton(joystick, 2).whenPressed(new InstantCommand(m_powercell::intake,m_powercell));
    new JoystickButton(joystick, 2).whenReleased(new InstantCommand(m_powercell::stopintake,m_powercell));
    new JoystickButton(joystick, 3).whenPressed(new InstantCommand(m_powercell::feed,m_powercell));
    new JoystickButton(joystick, 3).whenReleased(new InstantCommand(m_powercell::stopfeed,m_powercell));

  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }

  public void AutonomousInit() {

    m_tTrajectory.periodic();
    m_drivetrain.periodic();
  }
}
