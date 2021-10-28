// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DriveTrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  XboxController m_xbox = new XboxController(0);
  // The robot's subsystems and commands are defined here...

  /*
    Notice that those variables are private. This practice is called encapsulation. The idea's that the 
    classes internal details won't be available for outsiders to mess around with. However, in practice,
    they have to be exposed anyways, through getters and setters. See one below. 
  */
  private final Throttle m_throttle = new Throttle(); // create an instance of the throttle class. See explaination below
  private final Turn m_turn = new Turn();
  private final DriveTrain m_driveTrain = new DriveTrain(); 
  private final DriveCommand m_driveCommand = new DriveCommand(m_driveTrain, m_throttle, m_turn); 
  // Notice how the drivetrain is passed into the contructor for DriveCommand. Remember that commands act upon 
  // subsystems, so it must have access to the subsystem.

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  /*
   * The purpose of the Throttle and Turn classes are to calculate the turn amount or throttle speed given 
   * an xbox controller. Notice that they are declared *inside* another class. That means they are nested classes
   * and thus can access the parent classes attributes (in this case, m_xbox).
   * By extending DoubleSupplier, an instance of Throttle or Turn can be passed into any method (or class contructor), 
   * as long as they expect an DoubleSupplier. That means that the receiving class/contructor can ignore *how*
   * the value is calculated (it could be from an controller, or maybe it's randomly generated, or something else...),
   * just that it will be a Double.
   */
  private class Throttle implements DoubleSupplier{
    @Override
    public double getAsDouble() {
      return m_xbox.getY(Hand.kLeft);
    }
  }

  private class Turn implements DoubleSupplier{
    @Override
    public double getAsDouble() {
      return m_xbox.getX(Hand.kLeft);
    }
  }

  /* The getter for m_driveCommand. Notice that it's public, meaning that outsiders can access it. */
  public DriveCommand getDriveCommand(){
    return(m_driveCommand);
  }
}


