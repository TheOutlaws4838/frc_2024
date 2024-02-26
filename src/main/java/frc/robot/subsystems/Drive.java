package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase{

    private int REPLACE_THIS_NUMBER;

    private WPI_VictorSPX frontLeftMotor;
    private WPI_VictorSPX frontRightMotor; 
    private WPI_VictorSPX backLeftMotor; 
    private WPI_VictorSPX backRightMotor; 

    private DifferentialDrive differentialDrive;

    public void robotInit(){
        frontLeftMotor = new WPI_VictorSPX(REPLACE_THIS_NUMBER);
        backLeftMotor = new WPI_VictorSPX(REPLACE_THIS_NUMBER);

        frontRightMotor= new WPI_VictorSPX(REPLACE_THIS_NUMBER);
        backRightMotor= new WPI_VictorSPX(REPLACE_THIS_NUMBER);

        differentialDrive = new DifferentialDrive(frontLeftMotor, frontRightMotor);

        backLeftMotor.follow(frontLeftMotor);

        backRightMotor.follow(frontRightMotor);
    
    }

    public void arcadeDrive(double forwardBackward, double rotation ) {
        differentialDrive.arcadeDrive(forwardBackward, rotation);
    
    }
}


