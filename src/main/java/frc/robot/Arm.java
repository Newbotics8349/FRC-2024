package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Class that represents the controller for the physical robot's arm.
 * Handles intake and shooting processes, arm pitching movements,
 * and the proximity sensor for ring detection.
 */
public class Arm {
    // functional motors
    public CANSparkMax armMotor1;
    public CANSparkMax armMotor2;
    private CANSparkMax intakeMotor;
    private CANSparkMax shooterMotor1;
    private CANSparkMax shooterMotor2;
    private DigitalInput proximitySensor;
    public double ampPosition;

    public double lastCheckingSpeedTime = 0;
    public double lastEncoderValue = 0;    
    public double lastShooterSpeed = 0;
    
    // constants
    final double sprocketToMotorRatio = 0.5; // TODO: Update value
    final double armAngleToEncoderPosition = 360/42 * sprocketToMotorRatio;
    private final double armInitialAngle = 0; // TODO: Update value, angle the note would be shot out at from the shooter
    public double maxArmAngle = 180; // TODO: Update safety value
    public double minArmAngle = 0; // TODO: Update safety value
    
    /**
     * Constructor that initializes the motor and sensor objects
     */
    public Arm() {
        // Functional motors
        armMotor1 = new CANSparkMax(1, MotorType.kBrushless); // left arm base
        armMotor2 = new CANSparkMax(2, MotorType.kBrushless); // right arm base
        intakeMotor = new CANSparkMax(3, MotorType.kBrushed); // pwm set
        shooterMotor1 = new CANSparkMax(4, MotorType.kBrushless); // left shooter
        shooterMotor2 = new CANSparkMax(9, MotorType.kBrushless); // right shooter

        // TODO: Invert motors running backwards to what is expected during testing
        // moveMotorIDX.setInverted(true); // wiring thing, motor is flipped, bad wiring

        // Proximity sensor
        proximitySensor = new DigitalInput(1); // Use the actual DIO port number
    }

    /**
     * Checks whether a note is detected in the intake by the proximity sensor.
     * 
     * @return true if nothing is detected
     */
    public boolean checkSensorandNotify() { // method to return whether a note is loaded or not
        boolean noNoteDetected = proximitySensor.get(); // This will return true if nothing is detected
        // Assuming the sensor output is HIGH when an object is detected
        if (noNoteDetected) { // When no note is detected, this if statement occurs
            SmartDashboard.putString("Alert", "No note in the intake.");
        }
        // turn LED colour
        else { // When note is detected, this else statement occurs
            SmartDashboard.putString("Alert", "Note is in the intake!");
        }
        return noNoteDetected;
    }

    /**
     * Determines the angle the shooter on the arm is pointed at.
     * Uses the encoder on one of the motors controlling the arm pitch, as well as a
     * predifined starting angle.
     * 
     * @return the angle the shooter is pointed at in degrees
     */
    public double getArmAngle() {
        double currentEncoderPosition = armMotor1.getEncoder().getPosition(); // TODO: Make sure when the motor is applied positive power the encoder value goes up
        double currentAngle = currentEncoderPosition * armAngleToEncoderPosition + armInitialAngle;
        return currentAngle;
    }


    public double getAcceleration()
    {
        // Determine the current shooter acceleration
        // We can shoot once the acceleration is less than a defined threshold
        // That indicates that the motors have settled at their max speed for the motor power specified
        double lastTime = lastCheckingSpeedTime; // Done before next speed calculation to not be overwritten
        double curShooterSpeed = calculateShooterSpeed();
        double curTime = lastCheckingSpeedTime; // Different from lastTime since lastChechingSpeedTime is updated by calculateShooterSpeed()
        double curAvgAcceleration = (curShooterSpeed-lastShooterSpeed)/(curTime-lastTime); 
        
        lastShooterSpeed = curShooterSpeed;
        
        return curAvgAcceleration;
    }

    /**
     * Shoots a ring at a specified power
     * 
     * @param power, the motor power between -1 and 1
     */
    public void shoot(double power) {
        // Start to run the shooter motors at the specified speed
        prepareToShoot(power);

        if (power != 0)
        {
            final double accelerationThreshold = 0.1; // TODO: Adjust as needed, this should be as low as possible while being timely and accurate
            double curAvgAcceleration = getAcceleration();

            // if shooter speed has leveled out, fire, otherwise wait
            if (lastShooterSpeed != 0 && curAvgAcceleration <= accelerationThreshold)
                sendNoteToShooter(power==0?0:1);
            else sendNoteToShooter(0);
        }
        else sendNoteToShooter(0);
    }

    public double getArmHeight(double armAngle)
    {
        // TODO: Relate the distance from the ground to the shooter to the angle of the arm
        // Get the mech team to do this, they'll be bored now
        return 0;
    }


    public double getArmDistanceFromCamera(double armAngle)
    {
        // TODO: Relate the horizontal distance between the camera and the shooter to the angle of the arm
        return 0;
    }

    /**
     * Calculates the current motor speed of the shooter motors.
     * Calculates the speed between two instances of this function being called
     * 
     * @return the velocity (positive or negative speed) in cm/s
     */
    public double calculateShooterSpeed() {
        double currentTime = Timer.getFPGATimestamp();
        double currentEncoder = shooterMotor1.getEncoder().getPosition();
        double rangeTime = currentTime - lastCheckingSpeedTime;
        if (rangeTime > 0.25) { // Ensures the last time this function was called was recent
            lastCheckingSpeedTime = currentTime;
            lastEncoderValue = currentEncoder;
            return 0;
        }
        double rangeEncoder = currentEncoder - lastEncoderValue;
        final double distancePerTick = Math.PI * 5.08 / 21; // PI*Radius/(Encoder ticks/pi rad)
        double velocity = rangeEncoder * distancePerTick / rangeTime;
        lastCheckingSpeedTime = currentTime;
        lastEncoderValue = currentEncoder;
        return velocity;
    }

    /**
     * Give power to the shooting motors.
     * 
     * Used to get the motors up to speed before firing
     * 
     * @param power
     */
    public void prepareToShoot(double power) {
        shooterMotor1.set(power);
        shooterMotor2.set(power);
    }

     /**
     * Give power to the intake motor, passing the note to the shooter motors.
     * 
     * Used to shoot the note after the shooter motors have reached their max speed.
     * 
     * @param power
     */
    public void sendNoteToShooter(double power) {
        intakeMotor.set(power);
    }

    // Runs the intake motor at the specified power , automatically shuts off if
    // note is detected
    /**
     * Runs the intake motor at the specified power.
     * 
     * Turns off automatically when a note is detected.
     * 
     * @param power
     */
    public void intake(double power) {
        // Limit maximum power being set
        // TODO: Determine safe max power
        final double maxPower = 0.5;
        if (power > maxPower) {
            power = maxPower;
        } else if (power < -maxPower) {
            power = -maxPower;
        }
            //no note detected
        if (checkSensorandNotify() == false)
            // once the note is in the intake the motor stops
            intakeMotor.set(0);
        else
            // if there is no note the intake motor runs
            intakeMotor.set(power);
    }

    /*
     * Runs the motors controlling the arm pitch at the specified motor power.
     */
    public void setMotorPower(double power) {
        // Do not run motors if already at boundary angles
        // TODO: Make sure positive power causes the arm angle to increase
        if (getArmAngle() >= maxArmAngle && power > 1 || getArmAngle() <= minArmAngle && power < 1) 
            power = 0;

        // Limit maximum power being set
        // TODO: Determine safe max power
        final double maxPower = 0.5;
        // Cap power to positive and negative limits
        if (power > maxPower) {
            power = maxPower;
        } else if (power < -maxPower) {
            power = -maxPower;
        }
        armMotor1.set(power);
        armMotor2.set(power);
    }

    /*
     * Sets the shooting angle to the angle specified.
     */
    public void moveToPosition(double angle) {
        if (angle > maxArmAngle) {
            angle = maxArmAngle;
        } else if (angle < minArmAngle) {
            angle = minArmAngle;
        }
        final double Kp = 0.05; // TODO: adjust during testing, make this as large as possible without regularly overshooting the target angle
        double error = angle - getArmAngle(); // current position (after converting)*****
        double power = Kp * error;
        setMotorPower(power);
    }
}
