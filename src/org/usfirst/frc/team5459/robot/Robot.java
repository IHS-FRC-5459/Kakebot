//Team 5459 Code
//Filip Kernan
package org.usfirst.frc.team5459.robot;

import java.security.PublicKey;

import com.ni.vision.NIVision;
//import com.ni.vision.NIVision.DrawMode;
import com.ni.vision.NIVision.Image;
//import com.ni.vision.NIVision.ShapeMode;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


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
    Victor shoot1,shoot2,treads;//victor controllers
    Talon leftRear,arm;
    ADXRS450_Gyro gyro;//gyro
    AnalogInput forwardSensor, sideSensor;
    CameraServer camera;
    Image frame;
    double speedX, speedY, speedRote, gyroAngle, throttle, valueToMm = 0.001041/* scale factor for analog ultrasonics*/, xDistance, yDistance;
    boolean armed = false,hasShot = false,countTick1 = false, countTick2 = false, xPosition, yPosition, autoRerun = false, armDown = true;
    int tickCount1 = 0, tickCount2 = 0, currentTick = 0, session;
    
    
    
	/**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	rook = new RobotDrive(4,7, 0, 2);
    	rook.setInvertedMotor(MotorType.kRearLeft, true);
    	rook.setInvertedMotor(MotorType.kFrontLeft,  true);
    	rook.setSafetyEnabled(true);
    	rook.setExpiration(0.1);
    	stick1 = new Joystick(0); 
    	stick2 = new Joystick(1);
    	shoot1 = new Victor(1);
    	shoot2 = new Victor(5);
    	treads = new Victor(3);
    	arm = new Talon(8);
    	arm.setSafetyEnabled(true);
    	arm.setExpiration(0.1);
    	gyro = new ADXRS450_Gyro();
    	gyro.calibrate();
    	gyro.reset();
    	forwardSensor = new AnalogInput(0);
    	sideSensor = new AnalogInput(1);
    	camera = CameraServer.getInstance();
    	camera.setQuality(50);
    	camera.startAutomaticCapture("cam0");
    	/*frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);

        // the camera name (ex "cam0") can be found through the roborio web interface
        session = NIVision.IMAQdxOpenCamera("cam0",
                NIVision.IMAQdxCameraControlMode.CameraControlModeController);
        NIVision.IMAQdxConfigureGrab(session);*/
    }//TODO make cross hairs for goal offset
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
    	if (tickCount1 < 200) {
			rook.mecanumDrive_Polar(0.6, 0, 0);//drives forward 
		} else {
			rook.mecanumDrive_Polar(0, 0, 0);
		}//drives forward for 4 sec 
    	if (tickCount1 > 200 && xDistance > 4308) {
    		if (yDistance > 914) {
    			rook.mecanumDrive_Cartesian(0.5, 0.5, 0, 0);
    		}else {
				rook.mecanumDrive_Cartesian(0.5, 0, 0, 0);
			}
		}else {
			if (yDistance > 914) {
				rook.mecanumDrive_Cartesian(0, 0.5, 0, 0);
			} 
		}
    	if(xDistance <= 4308 ){
    		xPosition = true;
    	}
    	if (tickCount1 > 500 && yDistance > 914) {
			rook.mecanumDrive_Cartesian(0, 0.5, 0, gyroAngle);
		}//drives to ideal y
    	if (yDistance <= 914 ) {
			yPosition = true;
		} 
    	if (xPosition && yPosition) {//in ideal position
			currentTick = tickCount1;
			if (gyroAngle < 60 && gyroAngle > 60) {
				rook.mecanumDrive_Polar(0, 0.0, 0.5);
			}else {
				rook.mecanumDrive_Polar(0, 0.0, 0);
			}//turns to 60 degrees
			if (yDistance < 400.0) {
				rook.mecanumDrive_Polar(0.75, 0.0, 0.0);
			}
			
			if (yDistance >= 400.0) {
				 shoot1.set(-0.25);
				 shoot2.set(0.25);
			}else {
				shoot1.set(0.0);
				shoot2.set(0.0);
			}//shoots after in ideal shoot position
		}
    	tickCount1++;//counts ticks; tick == 200msec
    	Timer.delay(0.005);
    	
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	//NIVision.IMAQdxStartAcquisition(session);
    	//NIVision.Rect rect = new NIVision.Rect(10, 10, 100, 100);
    	//NIVision.IMAQdxGrab(session, frame, 1);
        //NIVision.imaqDrawShapeOnImage(frame, frame, rect,
          //      DrawMode.DRAW_VALUE, ShapeMode.SHAPE_OVAL, 0.0f);
        
        //CameraServer.getInstance().setImage(frame);
    	SmartDashboard.putNumber("side sensor", sideSensor.getValue());
    	SmartDashboard.putNumber("forward sensor", forwardSensor.getValue());
    	
    	throttle = (stick1.getThrottle()/2);
    	speedX = -stick1.getX() * throttleEncode(stick1);
    	speedY = -stick1.getY() * throttleEncode(stick1);
    	speedRote = stick1.getZ() * throttleEncode(stick1);
    	gyroAngle = -gyro.getAngle();
    	if (gyroAngle >= 360) {

			gyroAngle = gyroAngle - 360;
		}
    	SmartDashboard.putNumber("Gyro angle", gyroAngle);
    	if(stick1.getRawButton(7)){
    		autoRerun = true;
    	}else {
			autoRerun = false;
		}
    	if(autoRerun == false){
	    	if (stick1.getRawButton(2)) {
				rook.mecanumDrive_Cartesian(speedX, speedY, speedRote, gyroAngle );//if angle starts freaking out then uncomment the above if statment 
	    	}else {
	    		rook.mecanumDrive_Cartesian(speedX, speedY, 0, gyroAngle);
			}//rotation toggle
			if(stick1.getRawButton(1)){
	    		treads.set(0.5); 
	    	}else {
	    		treads.set(0.0);
	    	}
			//tread toggle
			if (tickCount1 >= 4) {
				tickCount1 = 0;
				countTick1 = false;
			}
			if (stick2.getRawButton(1)) {
				countTick1 = true;
			}
	    	if (tickCount1 < 0 && tickCount1 >= 3) {
				shoot1.set(0.25);
				shoot2.set(-0.25);//this can be made higher
			}else {
				shoot1.set(0.0);
				shoot2.set(0.0);
			}//auto shoots the ball for 600 ms
	    	
	    	//TODO maybe ajuste when ball is in
	    	
	    	   
	    	if (stick2.getRawButton(6)) {
	    		shoot1.set(0.25);
	    		shoot2.set(-0.25);
			}else {
				shoot1.set(0.0);
				shoot2.set(0.0);
			}//draws in ball
	    	
	    	if (stick2.getRawButton(5) && armDown) {//arm up
				countTick2 = true;
				if (tickCount2>= 1 && tickCount2 <= 3) {
					arm.set(0.2);
				}
	    		
			}else {
				if (stick2.getRawButton(3)) {//arm down
					countTick2 = true;
					if (tickCount2>= 1 && tickCount2 <= 3) {
						arm.set(0.2);
					}
		    		
				}else {
					arm.set(0.0);
					countTick2 = false;
					tickCount2 = 0;
				}	
			}
	    	
	    	
	    }else {
	    	countTick1 = true;
	    	if (xDistance > 4308) {
	    		xPosition = false;
	    		if (yDistance > 914) {
	    			rook.mecanumDrive_Cartesian(0.5, 0.5, 0, 0);
	    			yPosition = false;
	    		}else {
					rook.mecanumDrive_Cartesian(0.5, 0, 0, 0);
				}
			}else {
				if (yDistance > 914) {
					yPosition = false;
					rook.mecanumDrive_Cartesian(0, 0.5, 0, 0);
				} 
			}
	    	if(xDistance <= 4308 ){
	    		xPosition = true;
	    	}
	    	if (yDistance > 914) {
				rook.mecanumDrive_Cartesian(0, 0.5, 0, gyroAngle);
			}//drives to ideal y
	    	if (yDistance <= 914 ) {
				yPosition = true;
			} 
	    	if (xPosition && yPosition) {//in ideal position
				currentTick = tickCount1;
				if (gyroAngle < 60 && gyroAngle > 60) {
					rook.mecanumDrive_Polar(0, 0.0, 0.5);
				}else {
					rook.mecanumDrive_Polar(0, 0.0, 0);
				}//turns to 60 degrees
				if (yDistance < 400.0) {
					rook.mecanumDrive_Polar(0.75, 0.0, 0.0);
				}
				
				if (yDistance >= 400.0) {
					 shoot1.set(-0.25);
					 shoot2.set(0.25);
				}else {
					shoot1.set(0.0);
					shoot2.set(0.0);
				}//shoots after in ideal shoot position
			}
		}
    	if (countTick1) {
			tickCount1++;
		}//counts ticks
    	Timer.delay(0.005);
    	if (countTick2) {
			tickCount2++;
		}
        //NIVision.IMAQdxStopAcquisition(session);

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
    //TODO change to buttons
     double throttleEncode(Joystick stick) {
		double[] encode ={1.0,0.825,0.65,0.475,0.3, 0.125};//values for var speed
    	if (stick.getThrottle()<= -0.6666) {
			return encode[0];
		}
		if (stick.getThrottle()>= 0.6666) {
			return encode[5];
		}
    	if (stick.getThrottle() > -0.6666 && stick.getThrottle() <= -0.3333) {
			return encode[1];
		}
    	if (stick.getThrottle() < 0.6666 && stick.getThrottle() >= 0.3333) {
			return encode[4];
		}
    	if (stick.getThrottle() > -0.3333 && stick.getThrottle() < 0) {
			return encode[2];
		}
    	if (stick.getThrottle() < 0.3333 && stick.getThrottle() >= 0) {
			return encode[3];
		}
    	return 0.0;
	}
    double distance(AnalogInput sensor){
    	double dis;
    	dis = sensor.getValue() * valueToMm;
    	dis = dis / Math.cos(gyroAngle);
    	if (dis < 0) {
			dis = dis * -1;
		}
    	return dis;
    }
}
