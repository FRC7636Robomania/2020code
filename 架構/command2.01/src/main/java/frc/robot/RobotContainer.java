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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Button;
import frc.robot.commands.Aim;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Shoot;
import frc.robot.commands.Auto.Easyauto;
import frc.robot.commands.Auto.Easyauto1;
import frc.robot.commands.Auto.Easyauto2;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Powercell;
import frc.robot.subsystems.Vision;
/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Drivetrain       m_drivetrain       = new Drivetrain();
  private final Powercell        m_powercell        = new Powercell();
  private final Vision           m_vision           = new Vision();
  private final Joystick         joystick           = new Joystick(0);
  private final Joystick         drivestation       = new Joystick(2);
  private final ExampleCommand   m_autoCommand      = new ExampleCommand(m_exampleSubsystem);
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();
  private final Easyauto m_easyauto = new Easyauto(m_powercell,m_drivetrain,m_vision);
  private final Easyauto1 m_easyauto1 = new Easyauto1(m_powercell,m_drivetrain,m_vision);
  private final Easyauto2 m_easyauto2 = new Easyauto2(m_powercell,m_drivetrain,m_vision);
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_drivetrain.setDefaultCommand(new RunCommand(()->
    m_drivetrain.curvaturedrive(joystick.getY(), joystick.getZ(),joystick.getRawButton(1)),m_drivetrain));
    m_chooser.addOption("Simple AutoUP", m_easyauto);
    m_chooser.addOption("Simple AutoMID", m_easyauto1);
    m_chooser.addOption("Simple AutoDOWN", m_easyauto2);

    // Put the chooser on the dashboard
    Shuffleboard.getTab("Autonomous").add(m_chooser);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
    new JoystickButton(drivestation, Button.turrethoming).whenHeld(new InstantCommand(m_powercell::turrethoming,m_powercell));
    new JoystickButton(drivestation, Button.aim)         .whenHeld(new Aim(m_powercell, m_vision,m_drivetrain));
    new JoystickButton(drivestation, Button.shoot)       .whenHeld(new Shoot(m_powercell));
    new JoystickButton(drivestation, Button.wide)        .whenPressed(new InstantCommand(m_powercell::widein,m_powercell)).whenReleased(new InstantCommand(m_powercell::widestop,m_powercell));
    new JoystickButton(joystick,     Button.intake)      .whenPressed(new InstantCommand(m_powercell::intake,m_powercell)).whenReleased(new InstantCommand(m_powercell::intakestop,m_powercell));
    new JoystickButton(drivestation, Button.armup)       .whenPressed(new InstantCommand(m_powercell::armup,m_powercell)).whenReleased(new InstantCommand(m_powercell::armstop,m_powercell));
    new JoystickButton(drivestation, Button.armdown)     .whenPressed(new InstantCommand(m_powercell::armdown,m_powercell)).whenReleased(new InstantCommand(m_powercell::armstop,m_powercell));
    new JoystickButton(drivestation, Button.turretleft)  .whenPressed(new InstantCommand(m_powercell::turretleft,m_powercell)).whenReleased(new InstantCommand(m_powercell::turretstop,m_powercell));
    new JoystickButton(drivestation, Button.turretright) .whenPressed(new InstantCommand(m_powercell::turretright,m_powercell)).whenReleased(new InstantCommand(m_powercell::turretstop,m_powercell));
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
}
