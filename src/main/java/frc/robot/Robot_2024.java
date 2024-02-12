// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot_2024 extends TimedRobot {
   
  private final XboxController m_joystick_arm = new XboxController(0);
    private final XboxController m_joystick_drive = new XboxController(1);
    private final PWMSparkMax m_rightWheel0 = new PWMSparkMax(0);
    private final PWMSparkMax m_rightWheel1 = new PWMSparkMax(1);
    private final PWMSparkMax m_leftWheel0 = new PWMSparkMax(2);
    private final PWMSparkMax m_leftWheel1 = new PWMSparkMax(3);
private final MotorControllerGroup m_leftDrive = new MotorControllerGroup(m_leftWheel0, m_leftWheel1);
    private final MotorControllerGroup m_rightDrive = new MotorControllerGroup(m_rightWheel0, m_rightWheel1);
    private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftDrive, m_rightDrive)
    private final PWMSparkMax m_lift = new PWMSparkMax(7);
 
    /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
  
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double speed = .7;

    // this is for the lift
if (m_joystick_arm.getLeftY() > .25)
m_lift.set(m_joystick_arm.getLeftY()); 

if (m_joystick_arm.getLeftY()< -.25)
m_lift.set(m_joystick_arm.getLeftY());
  //Initiates Tank Drive
  double left_stick = m_joystick_drive.getLeftY() * speed; 
  double right_stick = m_joystick_drive.getRightY() * speed;

// this is for the drive
  m_robotDrive.tankDrive(left_stick,right_stick);
 // need to finish drive code. 

  }




  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
