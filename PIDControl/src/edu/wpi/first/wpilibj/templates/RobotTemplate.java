/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class RobotTemplate extends IterativeRobot {

    public class source implements PIDOutput {

        Talon m1;
        Talon m2;

        source(Talon t1, Talon t2) {
            m1 = t1;
            m2 = t2;
        }

        public void pidWrite(double t1) {
            m1.set(t1);
            m2.set(t1);
        }

    }

    //start variable declaration
    private Talon m1 = new Talon(1);
    private Talon m2 = new Talon(2);
    private Talon m3 = new Talon(3);
    private Talon m4 = new Talon(4);
    private Joystick joy = new Joystick(1);
    private Encoder e1 = new Encoder(6, 7, false, CounterBase.EncodingType.k4X);//left encoder
    private Encoder e2 = new Encoder(4, 5, true, CounterBase.EncodingType.k4X);//right encoder
    private Gyro g1 = new Gyro(1);
    private double Ki = .15;
    private double Kd = .880;
    private double Ks = .95;
    private double Kf = .1;
    private double delay = .004;
    private source s1 = new source(m1, m2);
    private source s2 = new source(m3, m4);
    private boolean writing = false;
    private trapezoidalPID leftTrapezoidal;
    private trapezoidalPID rightTrapezoidal;
    //end variable declaration

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        //encoder init
        e1.reset();//reset encoders
        e2.reset();
        e1.setDistancePerPulse(3.14 * 6 / 250);//distance per pulse in inches, ~18 inch diameter, 250 ticks per rotation
        e2.setDistancePerPulse(3.14 * 6 / 250);
        e1.setPIDSourceParameter(PIDSource.PIDSourceParameter.kDistance);
        e2.setPIDSourceParameter(PIDSource.PIDSourceParameter.kDistance);
        e1.setSamplesToAverage(5);
        e2.setSamplesToAverage(5);
        e1.start();
        e2.start();
        
            g1.setPIDSourceParameter(PIDSource.PIDSourceParameter.kAngle);
                    
        //end encoder init
        //trapezoidal controller init. Starts controllers in distance mode.
        leftTrapezoidal = new trapezoidalPID(Ki, Kd, Ks, Kf, e1, s1);//new object
        rightTrapezoidal = new trapezoidalPID(Ki, Kd, Ks, Kf, e2, s2);//new object
        rightTrapezoidal.invertOutput();//someone wired the right side backwards
        leftTrapezoidal.setOutputRange(0, .5);//limits the motor speed for safety reasons
        rightTrapezoidal.setOutputRange(0, .5);
        //end trapezoidal controller init
        print();//prints values to the smartdash to prevent errors from reading values before they are written
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {

    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        run();
        System.out.println("m1 " + m1.get());
        System.out.println("m3 " + m3.get());
    }

    /**
     * This function is called periodically during periodic mode
     */
    private void run() {

        buttonLogic();//performs functions with the joystick buttons

        if (writing) {
            print();//write to the smartdash
            leftTrapezoidal.performCalculations();//run the trapezoidal controllers
            rightTrapezoidal.performCalculations();
        } else {
            update();//read from the smartdash
        }

        //prints values to the driverstation
        DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser1, 1, "Encoder left\t| " + e1.getDistance());
        DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser2, 1, "Encoder right\t| " + e2.getDistance());
        DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser3, 1, "Gyro\t| " + g1.getAngle());
        DriverStationLCD.getInstance().updateLCD();

    }

    /**
     * uses the joystick buttons to perform various functions
     */
    private void buttonLogic() {
        //reset encoders, gyro, and trapezoidal controllers
        if (joy.getRawButton(2)) {
            e1.reset();//reset encoders
            e2.reset();
            g1.reset();//reset gyro;
            leftTrapezoidal.reset();//reset trapezoidal controllers
            rightTrapezoidal.reset();
        }

        //disable trapezoidal controllers, set motors to 0
        if (joy.getRawButton(6)) {//R1
            disableTrapezoidalControllers();//read the dang method name
            drive(0, 0);//stops the robot
        } //enable trapezoidal controllers, set them to drive for a distance 
        else if (joy.getRawButton(5)) {//L1
            enableTrapezoidalControllers();//read the dang method name
            driveDistance(100);//drive 100 inches forward
        } //turns left
        else if (joy.getRawButton(7)) {//L2
            enableTrapezoidalControllers();//read the dang method name
            driveAngle(-90);//turns 90 degrees left
        } //turns right
        else if (joy.getRawButton(8)) {//R2
            enableTrapezoidalControllers();//read the dang method name
            driveAngle(90);//turns 90 degrees right
        }
    }

    /**
     * gets values from the smartdash and sets the trapezoidal controller
     * constants
     */
    private void update() {

        Ki = SmartDashboard.getNumber("Ki");
        Kd = SmartDashboard.getNumber("Kd");
        Ks = SmartDashboard.getNumber("Ks");
        Kf = SmartDashboard.getNumber("Kf");
        leftTrapezoidal.setIDSF(Ki, Kd, Ks, Kf);//sets the controller to smartdash values
        rightTrapezoidal.setIDSF(Ki, Kd, Ks, Kf);
    }

    /**
     * writes values to the smartdash
     */
    private void print() {

        SmartDashboard.putNumber("e1 rate", e1.getRate());
        SmartDashboard.putNumber("Ki", Ki);
        SmartDashboard.putNumber("Kd", Kd);
        SmartDashboard.putNumber("Ks", Ks);
        SmartDashboard.putNumber("Kf", Kf);
        SmartDashboard.putNumber("e2 rate", e2.getRate());
    }

    /**
     * sets writing to true and enables both controllers
     */
    private void enableTrapezoidalControllers() {
        writing = true;//enable writing to smartdash and disable reading from smartdash
        leftTrapezoidal.enable();//enable controllers
        rightTrapezoidal.enable();
    }

    /**
     * sets writing to false and disables both controllers
     */
    private void disableTrapezoidalControllers() {
        writing = false;//disable writing to smartdash and enable reading from smartdash
        leftTrapezoidal.disable();//disable controllers
        rightTrapezoidal.disable();
    }

    /**
     * change both trapezoidal controllers to go for distance. Automagically
     * resets encoders
     */
    private void trapezoidalDistanceMode() {
        e1.reset();
        e2.reset();
        leftTrapezoidal.changeSource(e1);//use left encoder as a source
        leftTrapezoidal.deinvertOutput();//drive straight
        rightTrapezoidal.changeSource(e2);//use right encoder as a source
        rightTrapezoidal.invertOutput();//drive straight
    }

    /**
     * change both trapezoidal controllers to go for a left turn. Automagically
     * resets gyro
     */
    private void trapezoidalStationaryLeftTurnMode() {
        g1.reset();
        leftTrapezoidal.changeSource(g1);//use gyro as a source
        leftTrapezoidal.invertOutput();//drive backwards
        rightTrapezoidal.changeSource(g1);//use gyro as a source
        rightTrapezoidal.invertOutput();//drive straight
    }

    /**
     * change both trapezoidal controllers to go for a right turn. Automagically
     * resets gyro
     */
    private void trapezoidalStationaryRightTurnMode() {
        g1.reset();
        leftTrapezoidal.changeSource(g1);//use gyro as a source
        leftTrapezoidal.deinvertOutput();//drive straight
        rightTrapezoidal.changeSource(g1);//use gyro as a source
        rightTrapezoidal.deinvertOutput();//drive backwards
    }

    /**
     * returns a double that is <code>d</code> units more than
     * {@link Encoder#getDistance() e.getDistance()}
     *
     * @param e an encoder
     * @param d the distance you want to drive from the current distance
     * @return
     */
    /*  private double getDistanceSetpoint(Encoder e, double d) {
     return e.getDistance() + d;//return a value d units more than the current encoder reading
     }*/
    /**
     * DON'T CALL THIS PERIODICALLY. CALL THIS ONCE. Sets the trapezoidal
     * controllers to drive for distance d
     *
     * @param d the distance to travel
     */
    private void driveDistance(double d) {
        trapezoidalDistanceMode();
        leftTrapezoidal.setSetpoint(d);
        rightTrapezoidal.setSetpoint(d);
    }

    /**
     * DON'T CALL THIS PERIODICALLY. CALL THIS ONCE. Sets the trapezoidal
     * controllers to drive to angle a. Automagically resets gyro
     *
     * @param a
     */
    private void driveAngle(double a) {
        if (a > 0) {
            //if the angle is positive, you want to turn right
            trapezoidalStationaryRightTurnMode();
        } else if (a < 0) {
            //if the angle is negative, you want to turn left
            trapezoidalStationaryLeftTurnMode();
        } else {
            return;
        }
        leftTrapezoidal.setSetpoint(a);
        rightTrapezoidal.setSetpoint(a);
    }

    /**
     * DON'T CALL THIS PERIODICALLY. CALL THIS ONCE. Drives towards the ball
     * using the camera outputs
     */
   /* private void driveTowardsBall() {
        double dist = SmartDashboard.getNumber("BallDistance");//reads the camera distance output
        driveDistance(dist);
    }*/

    /**
     * DON'T CALL THIS PERIODICALLY. CALL THIS ONCE. Automagically resets the
     * gyro. Drives towards the ball using camera output.
     */
    /*private void turnTowardsBall() {
        double angle = SmartDashboard.getNumber("BallAngle");//reads the camera angle output
        driveAngle(angle);
    }*/

    /**
     * This function is called periodically during operator control. Drives the
     * robot under arcade drive
     */
    public void arcade() {
        double speed = joy.getRawAxis(2) * .5;//get speed from left stick y axis, limit it for safety
        speed = roundDown(speed);//turn speed  to 0 if they are within tolerance

        boolean turnR = joy.getRawButton(5);
        boolean turning = joy.getRawButton(5) || joy.getRawButton(6);

        //moving forward
        if (speed != 0 && !turning) {
            drive(-speed, speed);//drive forward
        } else if (turning && speed == 0) {//turning in place
            if (turnR) {//turn right
                drive(-.5, -.5);//left wheel forward, right wheel back
            } else {//turn left
                drive(.5, .5);//left wheel back, right wheel forward
            }
        } else if (speed != 0 && turning) {//turning while moving
            double modifiedSpeed = speed * .5;//.5 can be changed to a bigger number for less speed, more handling, smaller for more speed, less handling
            if (turnR) {//turn right
                drive(-speed, modifiedSpeed);//left wheel forward, right wheel forward slower
            } else {//turn left
                drive(-modifiedSpeed, speed);//left wheel forward slower, right wheel forward
            }
        } else {
            drive(0, 0);//stop the robot
        }

    }

    /**
     * Sets t1 and t2 to speed <code>speed</code>
     *
     * @param t1 a talon
     * @param t2 another talon
     * @param speed a double between -1 and 1
     */
    private void drive(Talon t1, Talon t2, double speed) {
        t1.set(speed);
        t2.set(speed);
    }

    /**
     * Sets m1 and m2 to <code>s1</code> and m3 and m4 to <code>s2</code>
     *
     * @param s1 a double between -1 and 1
     * @param s2 a double between -1 and 1
     */
    private void drive(double s1, double s2) {
        m1.set(s1);
        m2.set(s1);
        m3.set(s2);
        m4.set(s2);
    }

    /**
     * Rounds <code>d</code> down if it is within -.05 and .05
     *
     * @param d a joystick value
     * @return the rounded down value
     */
    private double roundDown(double d) {
        if (d <= .05 && d >= -.05) {
            return 0;
        }
        return d;
    }

}
