//Team 5459 Code
//Filip Kernan
package org.usfirst.frc.team5459.robot;

import java.security.PublicKey;


//import com.ni.vision.NIVision.DrawMode;
import com.ni.vision.NIVision.Image;
//import com.ni.vision.NIVision.ShapeMode;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// hi
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
    RobotDrive rook;//drive name
    Joystick stick1, stick2;//the joysticks
    Victor shoot2,treads;//victor controllers
    Talon arm,shoot1;
    ADXRS450_Gyro gyro;//gyro
    AnalogInput forwardSensor, sideSensor;
    DigitalInput isBallIn;
    CameraServer camera;
    Image frame;
    Integer noAuto, Auto, simpleAuto, shortAuto;
    SendableChooser autoChooser;
    double speedX, speedY, speedRote, gyroAngle, varSpeed, valueToMm = 3.63/* scale factor for analog ultrasonics*/, xDistance, yDistance;
    boolean armed = false,hasShot = false,countTick1 = false, countTick2 = false, countTick3 = false, xPosition, yPosition, autoRerun = false, armDown = true, ballIn = false;
    int tickCount1 = 0, tickCount2 = 0, currentTick = 0, tickCount3 = 0, session;
    
    
    
 /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
     rook = new RobotDrive(3, 2, 1, 0);
     
     rook.setInvertedMotor(MotorType.kRearLeft, true);
     rook.setInvertedMotor(MotorType.kFrontLeft,  true);//reverses left motors
     rook.setSafetyEnabled(true);
     rook.setExpiration(0.1);
     stick1 = new Joystick(0); 
     stick2 = new Joystick(1);

     shoot1 = new Talon(5);

     gyro = new ADXRS450_Gyro();
     noAuto = 1;
     Auto = 0;
     shortAuto = 3;
     simpleAuto = 2;
     gyro.calibrate();
     gyro.reset();
     forwardSensor = new AnalogInput(0);
     sideSensor = new AnalogInput(1);
     isBallIn = new DigitalInput(0);
     camera = CameraServer.getInstance();
     camera.setQuality(50);
     camera.startAutomaticCapture("cam0");
     autoChooser = new SendableChooser();
     autoChooser.addDefault("shorter auto run", shortAuto);
     autoChooser.addObject("Auto", Auto);
     autoChooser.addObject("simple auto", simpleAuto);
     SmartDashboard.putData("auto chooser", autoChooser);
    }
    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
     
    	 gyroAngle = gyro.getAngle();
    	 if (gyroAngle >= 360) {
    		 gyroAngle = gyroAngle - 360;
    	 }
	      	xDistance = distance(forwardSensor);
	      	yDistance = distance(sideSensor);//gets current position 
	      	//if (tickCount1 > 70 && xDistance > 1430 && tickCount1 < 100) {
	      		//if (yDistance > 914) {
	      			//rook.mecanumDrive_Cartesian(0.5, 0.5, 0, 0);
	      		//}else {
	      			//rook.mecanumDrive_Cartesian(0.5, 0, 0, 0);
	      			//}
	      	//}else {
	      		//if (yDistance > 914 && tickCount1 < 100) {
	      			//rook.mecanumDrive_Cartesian(0, 0.5, 0, 0);
	      		//}else {
	      			//rook.mecanumDrive_Cartesian(0, 0, 0, gyroAngle);
	      		//}
	      	//}//goes to ideal position

	      	if(xDistance <= 1430  && tickCount1 > 90){
	      		xPosition = true;
	      	}//checks if x is correct
	      
	      	if (yDistance <= 914 && tickCount1 > 90) {
	      		yPosition = true;
	      	} //checks if y is correct
	      	/*if (xPosition && yPosition) {//in ideal position
	      		currentTick = tickCount1;
	      		if (gyroAngle < 60 && gyroAngle > 60) {
	      			rook.mecanumDrive_Polar(0, 0.0, 0.5);
	      		}else {
	      			rook.mecanumDrive_Polar(0, 0.0, 0);
	      		}//turns to 60 degrees
	      		if (yDistance < 400.0) {
	      			rook.mecanumDrive_Polar(1, 0.0, 0.0);
	      		}
	    
	      		if (yDistance >= 300.0) {
	      			shoot1.set(1);
	      			
	      		}else {
	      			shoot1.set(0.0);
	      		
	      		}//shoots after in ideal shoot position
	      	}*/
	      	if (tickCount1 < 89) {

	      		rook.mecanumDrive_Cartesian(0, 1, 0, 0);//drives forward 
	      	} else {
	      		rook.mecanumDrive_Polar(0, 0, 0);
	      	}//drives forward for 4 sec

     	tickCount1++;//counts ticks; tick == 200msec
     	Timer.delay(0.005);

     
    	}

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    
    	
    	 SmartDashboard.putNumber("side sensor", distance(sideSensor));
	     SmartDashboard.putNumber("forward sensor", distance(forwardSensor));//smart dash board
	     if(!stick1.getRawButton(12)){
	    	 varSpeed = 0.5;
	     }else{
	    	 varSpeed = 0.7;
	     }
	     speedX = -stick1.getX() * varSpeed;
	     speedY = -stick1.getY() * varSpeed;
	     speedRote = -stick1.getZ() *varSpeed;
	     gyroAngle = -gyro.getAngle();
	     //if (gyroAngle >= 360) {
	    	// gyroAngle = gyroAngle - 360;
	     //}
	     SmartDashboard.putNumber("Gyro angle", gyroAngle);

	     if (stick1.getRawButton(2)) {
	    	rook.mecanumDrive_Cartesian(speedX, speedY, speedRote, gyroAngle );
	     }else {
	       rook.mecanumDrive_Cartesian(speedX, speedY, 0, gyroAngle);
	     }
	     if (stick1.getRawButton(1)) {
	    	shoot1.set(-1);
	     }else if (stick1.getRawButton(2) /*&& ballIn*/) {
		     shoot1.set(1);
		 }else {
		    shoot1.set(0.0);
		    
		 }
	     if (countTick3){
	    	 tickCount3++;
	     }
	     if (countTick2) {
	    	 tickCount2++;
	     }//counts ticks
	     Timer.delay(0.005);
	 
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
     
    }
    
    /*
     * 
     *
     */
    
     
    double distance(AnalogInput sensor){
     double dis;
     dis = sensor.getValue() * valueToMm;

     //dis = dis / Math.cos(gyroAngle);

     
     return dis;
    }
}
