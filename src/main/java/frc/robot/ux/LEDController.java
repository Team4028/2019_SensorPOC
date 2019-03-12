package frc.robot.ux;

import edu.wpi.first.wpilibj.Spark;
import frc.robot.RobotMap;

/**
 * This class controls the onboard LEDs Lead Student: Parker Johnson
 */
public class LEDController {

	private Spark LEDstrip;
    private static final double REDZONE = 27.0; 
	private static final double YELLOWZONE = 10.0;
    private static final double GREENZONE = 2.0;
    
    //=====================================================================================
	// Define Singleton Pattern
	//=====================================================================================
	private static LEDController _instance = new LEDController();
	
	public static LEDController getInstance() {
		return _instance;
	}
	
    // private constructor for singleton pattern - this LED Controller is like a motor in regards to set speed, colors are on rev
    // robotics Blinkin LED User manual. 
	private LEDController() {	
		LEDstrip = new Spark(RobotMap.PWM_LED_PORT);
	}
	
    /* call this method to display the correct LED Color, the method checks to see if a target is aquired using vision, 
    then goes though a series of if/ else statements to determine the position of angle one. When the target is homed +/- two degrees 
    Izzy's distance sensor determines whether the target is within one foot, if it is, it lights up in partymode (yass), if not 
    it is a solid green color*/ 
	public void set_targetangle (double currentAngleInDegrees, boolean isTargetAcquired, double inchesOut){
        if(isTargetAcquired == true) {
            if(Math.abs(currentAngleInDegrees) > REDZONE) {
                whiteLights();
            }
            else if(Math.abs(currentAngleInDegrees) >= YELLOWZONE && Math.abs(currentAngleInDegrees)<= REDZONE) {
                redLEDstrip();
            }
            else if(Math.abs(currentAngleInDegrees) >= GREENZONE && Math.abs(currentAngleInDegrees)<= YELLOWZONE) {
                orangeLEDstrip();
            }
            else if(Math.abs(currentAngleInDegrees) <= GREENZONE && inchesOut >= 12) {
                greenLights();
            }
            else if(Math.abs(currentAngleInDegrees) <= GREENZONE && inchesOut < 12 ) {
                partyMode();
            }
        } else {
            redBlinkLights();
        }
    }
    public void mvrCompPrettyColors()
    {
        LEDstrip.set(-0.97);
    }
    //Different methods that set the LEDs to a certain color, the names are self-explainitory
	private void partyMode() {
		LEDstrip.set(-0.99);
    }
    
	private void whiteLights(){
		LEDstrip.set(0.99);
    }

    private void redBlinkLights(){
        LEDstrip.set(-0.11);
    }
    
	private void greenLights(){
        LEDstrip.set(0.75);
    }

    private void orangeLEDstrip(){
        LEDstrip.set(0.67);
    }

    private void redLEDstrip(){
        LEDstrip.set(0.61);
    }

    /*
    private void fireLEDS(){
        LEDstrip.set(0.57);
    }
    
    private void oceanicPaletteLEDs(){
        LEDstrip.set(-0.95);
    }
    */
}
