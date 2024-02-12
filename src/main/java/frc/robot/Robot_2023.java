package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

import javax.print.attribute.standard.PrinterMessageFromOperator;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.Filter;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

//import edu.wpi.first.wpilibj.;
// import edu.wpi.first.wpilibj.drive.arcadeDrive;

public class Robot_2023 extends TimedRobot {

    // constants
    private static final double TURBO_SPEED = 1;
    private static final double SPEED = 0.7;
    private static final double AUTONAMOUS_SPEED = 0.3;
    private static final double CLAW_SPEED = 0.5;

    private static final double LIFT_SPEED = 1.0;
    private static final double ARM_SPEED = 0.5;
    private static final double TRIGGER_PRESS_THRESHOLD = 0.2;
    private static final double BRAKE_TRIGGER_THRESHOLD = 0.5;
    private static final String kSides = "s";
    private static final String kCenter = "c";
    private static final String kOff = "off";


    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();
    SlewRateLimiter slewLimiter = new SlewRateLimiter(.5);
    SlewRateLimiter slewLimiter2 = new SlewRateLimiter(.5);


    // instantiate variables
    private final XboxController m_joystick_arm = new XboxController(0);
    private final XboxController m_joystick_drive = new XboxController(1);
    private final PWMSparkMax m_rightWheel0 = new PWMSparkMax(0);
    private final PWMSparkMax m_rightWheel1 = new PWMSparkMax(1);
    private final PWMSparkMax m_leftWheel0 = new PWMSparkMax(2);
    private final PWMSparkMax m_leftWheel1 = new PWMSparkMax(3);


    
    private final PWMSparkMax m_lift = new PWMSparkMax(7);
    private final PWMSparkMax m_arm = new PWMSparkMax(5);
    private final PWMSparkMax m_claw = new PWMSparkMax(4);
    private final PWMSparkMax m_brake = new PWMSparkMax(6);

    private final Timer m_timer = new Timer();

    private final MotorControllerGroup m_leftDrive = new MotorControllerGroup(m_leftWheel0, m_leftWheel1);
    private final MotorControllerGroup m_rightDrive = new MotorControllerGroup(m_rightWheel0, m_rightWheel1);
    
    private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftDrive, m_rightDrive);

    private final DigitalInput m_expandLimitSwitch = new DigitalInput(0);    
    private final DigitalInput m_contractLimitSwitch = new DigitalInput(1);
    //Default state for turbo is off
    private boolean turbo_mode = false;
    
    //Default state for brake mode is off
    private boolean brake_mode = false;

    private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();

    private double prevXAccelFilter = 0;
    private double prevYAccelFilter = 0;
    private double prevZAccelFilter = 0;

    // Claw Limit Switch variables
    private boolean clawFullyClosed = false;
    private boolean clawFullyOpened = false;

    // PID Constants
    private static final double kP = 0.05;
    private static final double kI = 0.0;
    private static final double kD = 0.005;

 
    // PID controller
    private PIDController pidController = new PIDController(kP, kI, kD);

    
    // Create a LinearDigitalFilter that will calculate a moving average of the measured X acceleration over the past 10 iterations of the main loop
    
    // LinearDigitalFilter xAccelFilter = LinearDigitalFilter.movingAverage(10);

    @Override
    public void robotInit() {
        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        m_leftDrive.setInverted(true);

       //Camera start
       CameraServer.startAutomaticCapture(0).setResolution(480, 270);
       CameraServer.startAutomaticCapture(1).setResolution(480, 270);

    
       
       m_chooser.setDefaultOption("Center Auto", kCenter);
       m_chooser.addOption("Side Auto", kSides);
       m_chooser.addOption("Disable Autonamous", kOff);
       
       SmartDashboard.putData("Custom", m_chooser);
        // Places a compass indicator for the gyro heading on the dashboard
        // Shuffleboard.getTab("Example tab").add(gyro);
        // SmartDashboard.putNumber("Example tab", gyro.getAngle());

    }

    @Override
    public void robotPeriodic() {

        SmartDashboard.putBoolean("kexpanded", m_expandLimitSwitch.get());
        SmartDashboard.putBoolean("kcontracted", m_contractLimitSwitch.get()); 
        SmartDashboard.putString("Gryo: ", gyro.getAngle() + "");
    }
  
    @Override
    public void teleopInit() {
         
    }
  
    /** This function is called periodically during teleoperated mode. */
    @Override
    public void teleopPeriodic() { // onUpdate()

        //Turbo driving
        turbo_mode = m_joystick_drive.getLeftTriggerAxis() >= BRAKE_TRIGGER_THRESHOLD;
        
        double speed;
        if (turbo_mode == true){
            speed = TURBO_SPEED;
        } else {
            speed = SPEED;
        }

        //Brake mode
        if (m_joystick_drive.getBButtonPressed()){
            m_brake.set(1);
        }
        else if (m_joystick_drive.getYButtonPressed()){
            m_brake.set(-1);
    

        }
        else {
            m_brake.set(0);
        }
            // System.out.print("back button presses");
            // brake_mode = !brake_mode;

        //     if (brake_mode == true){
        //         m_brake.set(1);
        //     } else if (brake_mode == false){
        //         m_brake.set(-1);
        //     }
        //     else {
        //         brake_mode = false;
        //         m_brake.set(-1);
        //     }
          

        
        //Initiates Tank Drive
        double left_stick = m_joystick_drive.getLeftY() * speed; 
        double right_stick = m_joystick_drive.getRightY() * speed;

        //When left_stick and right_stick, pressing up makes robot go backwards
        //Arguments may need to be flipped to make more sense
        m_robotDrive.tankDrive(left_stick, right_stick); 

        m_arm.set(m_joystick_arm.getLeftY() * ARM_SPEED);

        if (m_contractLimitSwitch.get()){
            clawFullyOpened = true;
        }
        else{
            clawFullyOpened = false;
        }

        if (m_expandLimitSwitch.get()){
            clawFullyClosed = true;
        }
        else{
            clawFullyClosed = false;
        }
        
        // Code to control claw mechanism
        if (m_joystick_arm.getRightTriggerAxis() >= TRIGGER_PRESS_THRESHOLD && m_contractLimitSwitch.get()) {
            m_claw.set(m_joystick_arm.getRightTriggerAxis() * -CLAW_SPEED);
        }
        else if (m_joystick_arm.getLeftTriggerAxis() >= TRIGGER_PRESS_THRESHOLD  && m_expandLimitSwitch.get()) {
            m_claw.set(m_joystick_arm.getLeftTriggerAxis() * CLAW_SPEED);
        }
        else {
            m_claw.stopMotor();
        }
        
        // Code to control the lift mechanism 
        if (m_joystick_arm.getRightY() < -0.1 ) {
            m_lift.set(LIFT_SPEED *  -m_joystick_arm.getRightY());  //  add negative to ensure up raises and down lowers
        } 
        else if (m_joystick_arm.getRightY() > 0.1) {
            m_lift.set(LIFT_SPEED * -m_joystick_arm.getRightY());  //
        }
        else {
            m_lift.set(0);
        }
    }

    @Override
    public void autonomousInit() {
        // TODO Auto-generated method stub
        m_autoSelected = m_chooser.getSelected();


        m_timer.reset();
        m_timer.start();


    }
    boolean isPIDenabled = false;
    double currentTime = 0;
    @Override
    public void autonomousPeriodic() {
        if (m_timer.get() < .5) {
            m_lift.set(-LIFT_SPEED);
        } else {
        m_lift.set(0);

        if (m_expandLimitSwitch.get()) {
            m_claw.set(CLAW_SPEED);
        } else {
            m_claw.set(0);


            if (currentTime == 0){
                currentTime = m_timer.get();
            }

            // if(m_timer.get() > currentTime && m_timer.get() < currentTime + 1) {
            //     m_robotDrive.tankDrive(0, 0); // drive forward
            // } else if (m_timer.get() > currentTime + 1 && m_timer.get() < currentTime + 5){
            //     m_robotDrive.tankDrive(slewLimiter.calculate  (0.5), slewLimiter2.calculate  (0.55)); // drive forward
            // } else {
            //     m_robotDrive.tankDrive(0, 0); // drive forward

            // }
        }
    
    }
        
        
    
        

        // if (m_timer.get() > 10) return;
        // SmartDashboard.putString("Autonomous Timer: ", m_timer.get());
    //     SmartDashboard.putString("Autonomous Mode Activated: ", m_autoSelected);
    //   switch(m_autoSelected){
    //     case kCenter:        
    //         if(isPIDenabled){
    //             double pidO utput = pidController.calculate(gyro.getAngle(), 0);  
    //             pidOutput = clamp(pidOutput, -AUTONAMOUS_SPEED, AUTONAMOUS_SPEED);
          
    //             m_robotDrive.tankDrive(slewLimiter.calculate(pidOutput), slewLimiter.calculate(pidOutput));
    //             // m_robotDrive.tankDrive(pidOutput, pidOutput);
    //             SmartDashboard.putString("PID", pidOutput + ":" + slewLimiter.calculate(pidOutput));
    //         } else {
    //             if (Math.abs(gyro.getAngle()) > 5) {
    //                 isPIDenabled = true;
                    
    //             }
    //             else {
    //                 m_robotDrive.tankDrive(0.5, 0.5); // drive forwards 
    //             }

    //         }
    //     break;
    //     case kSides:
    //         // Drive for 5 seconds
    //         if (m_timer.get() < 5) {
    //             m_robotDrive.tankDrive(0.5, 0.475); // drive forwards 
    //         } else if (m_timer.get() >= 5 && m_timer.get() < 6.5) {
    //             m_robotDrive.tankDrive(0, 0);
    //         } else {
    //             m_robotDrive.tankDrive(0, 0);
    //         }
    //     break;
    //     case kOff:
    //     break;
    //   }
    }

    public double clamp (double value, double min, double max) {
        return Math.min(max, Math.max(value, min));
    }
}