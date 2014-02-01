/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;

/**
 * s
 *
 * @author Jacob Melite
 */
public class trapezoidalPID {

    /**
     * |
     * |s \ \ |d | |i / /
     */
    private double i;//the end of the increasing line
    private double d;//the start of the decreasing line
    private double s;//the end of the decreasing line
    private double f;//the feedforward constant
    private PIDSource source;
    private PIDOutput output;
    private boolean enabled;
    private double startingInput = 0.0;
    private double maxOutput = 1.0;
    private double minOutput = 0.0;
    private double setpt;
    private double result;
    private double tolerance = .05;
    private int invertOutput = 1;

    /**
     *
     * @param Ki a percentage between 0 and 1
     * @param Kd a percentage between <code>Ki</code> and 1
     * @param Ks a percentage between <code>Kd</code> and 1
     * @param src a {@link PIDSource} object
     * @param out a {@link PIDOutput} object
     */
    public trapezoidalPID(double Ki, double Kd, double Ks, PIDSource src, PIDOutput out) {
        this(Ki, Kd, Ks, .1, src, out);//reuses the constructor
    }

    /**
     *
     * @param Ki a percentage between 0 and 1
     * @param Kd a percentage between <code>Ki</code> and 1
     * @param Ks a percentage between <code>Kd</code> and 1
     * @param Kf the feedforward constant
     * @param src a {@link PIDSource} object
     * @param out a {@link PIDOutput} object
     */
    public trapezoidalPID(double Ki, double Kd, double Ks, double Kf, PIDSource src, PIDOutput out) {
        i = Ki;
        d = Kd;
        s = Ks;
        f = Kf;
        output = out;
        source = src;
        startingInput = src.pidGet();
    }

    /**
     * this should be called in a periodic function
     *
     * @return <code>result</code>, the value being passed to
     * {@link PIDOutput#pidWrite(double) output.pidWrite(result)}
     */
    public double performCalculations() {
        calculate();//calculates result
        output.pidWrite(invertOutput * result);//if the value is being inverted, invert the result, then write to the output
        System.out.println("final results" + invertOutput * result);//troubleshooting
        return invertOutput * result;
    }

    /**
     * changes <code>result</code> to a negative value, in case the motors are
     * wired backwards or you need to drive backwards
     */
    public void invertOutput() {
        invertOutput = -1;
    }

    /**
     * switches <code>result</code> back to a positive value, in case you
     * already called <code>invertOutput()</code>
     */
    public void deinvertOutput() {
        invertOutput = 1;
    }

    /**
     * tolerance as a value between 0 and 1 used in <code>ontarget()</code>
     *
     * @param tol a value between 0 and 1
     */
    public void setTolerance(double tol) {
        if (tol > 1 || tol < 0) {
            return;
        }
        tolerance = tol;
    }

    /**
     * returns if {@link PIDSource#pidGet() source.pidGet()} is within
     * <code>tolerance</code> of <code>setpt</code>
     *
     * @return
     */
    public boolean onTarget() {
        if (setpt >= 0) {//if setpt is positive
            return (source.pidGet() * tolerance < setpt && source.pidGet() * (1 + tolerance) > setpt);
        } else {//if setpt is negative
            return (source.pidGet() * tolerance > setpt && source.pidGet() * (1 + tolerance) < setpt);
        }
    }

    /**
     * Changes <code>source</code> to PIDSource <code>s<\code>
     *
     * @param s a PIDSource, such as an encoder or gyro
     */
    public void changeSource(PIDSource s) {
        source = s;
        reset();//set setpt to 0, sets startinginput to the current value of s
    }

    /**
     * calculates the <code>result</code>, which is used to write to the
     * {@link PIDOutput output}
     */
    private void calculate() {
        if (enabled) {
            double absSetpt = Math.abs(setpt);
            if (source.pidGet() < absSetpt * s * i) {
                //increasing speed
                double slope;
                try {//ensures slope is not 0
                    slope = calculateSlope(startingInput, absSetpt * s * i, minOutput, maxOutput);
                } catch (Exception ex) {
                    ex.printStackTrace();
                    result = 0;//stops the motors
                    return;
                }
                System.out.println("source get " + source.pidGet());//troubleshooting
                result = (source.pidGet() - absSetpt * s * i) / slope + maxOutput + f + .001;//uses point slope (y1-y2)= m(x1-x2), using setpt*i*s as y2 and maxOutput as x2 to find the x1 value using the known y1 value, pidGet()
                System.out.println("calculated result " + result);//troubleshooting
            } else if (source.pidGet() < absSetpt * s * d) {
                //constant speed
                result = maxOutput;
            } else if (source.pidGet() < absSetpt * s) {
                //decreasing speed
                double slope;
                try {//ensures slope is not 0
                    slope = calculateSlope(absSetpt * s, absSetpt * s * d, minOutput, maxOutput);
                } catch (Exception ex) {
                    ex.printStackTrace();
                    result = 0;//stops all motors
                    return;
                }
                result = (source.pidGet() - absSetpt * s) / slope + minOutput - .001;//uses point slope (y1-y2)= m(x1-x2), using setpt*s as y2 and minoutput as x2 to find the x1 value using the known y1 value, pidGet()
                if (result < f) {
                    result = f;//ensures that the robot doesnt stop
                }
            } else if (source.pidGet() < absSetpt) {
                //roll forward until the end
                result = f;//feeds the robot forward for the last segment
            } else {
                result = 0;//stops the robot once it reaches its setpt
            }
            if (result < minOutput) {
                result = minOutput;//ensures that result doesnt go below minOutput
            } else if (result > maxOutput) {
                result = maxOutput;//ensures that result doesnt go above maxOutput
            }
        } else {
            result = 0;//stops the robot if the controller is disabled
        }
    }

    /**
     * calculates the slope between these values. if <code>x1-x2==0</code>,
     * returns 0
     *
     * @param y1
     * @param y2
     * @param x1
     * @param x2
     * @return
     */
    private double calculateSlope(double y1, double y2, double x1, double x2) throws Exception {
        if (x1 - x2 == 0 || y1 - y2 == 0) {
            throw new IllegalStateException("Slope became 0 or undefined");
        }
        return (y1 - y2) / (x1 - x2);//change in y over the change in x
    }

    public void setSetpoint(double setpoint) {
        setpt = setpoint;
    }

    /**
     *
     * @return <code>setpt</code>
     */
    public double getSetpoint() {
        return setpt;
    }

    /**
     * enables the calculate() method
     */
    public void enable() {
        enabled = true;
    }

    /**
     * sets all outputs to 0 to stop the robot and makes calculate() return 0
     */
    public void disable() {
        output.pidWrite(0);//stops the robot
        enabled = false;
    }

    /* public void setInputRange(double min, double max) {
     if (min > max) {//in case the user puts the inputs in backwards, flip them
     minInput = max;
     maxInput = min;
     return;
     }
     maxInput = max;
     minInput = min;
     }*/
    /*   public void setMinInput(double min) {
     startingInput = min;
     }*/
    /**
     * sets the minimum and maximum outputs to the PIDOutput
     *
     * @param min the minimum output
     * @param max the maximum output
     */
    public void setOutputRange(double min, double max) {
        if (min == max) {
            System.out.println("Min output cannot equal max output");
            return;
        }
        if (min > max) {//in case the user puts the inputs in backwards, flip them
            minOutput = max;
            maxOutput = min;
            return;
        }
        minOutput = min;
        maxOutput = max;
    }

    /**
     * Ki must be less than Kd which must be less than Ks
     *
     * @param Ki a percentage between 0 and 1
     * @param Kd a percentage between <code>Ki</code> and 1
     * @param Ks a percentage between <code>Kd</code> and 1
     */
    public void setIDS(double Ki, double Kd, double Ks) {
        if (Ki < Kd && Kd < Ks) {
            i = Ki;
            d = Kd;
            s = Ks;
        } else {
            return;
        }
    }

    /**
     * sets i, d, and s values using an array of values
     *
     * @param K an array of constants, the first being Ki, 2nd Kd, 3rd Kf
     */
    public void setIDS(double[] K) {
        if (K.length < 3) {
            return;
        }
        setIDS(K[0], K[1], K[2]);
    }

    /**
     * Ki must be less than Ki which must be less than Ks
     *
     * @param Ki a percentage between 0 and 1
     * @param Kd a percentage between <code>Ki</code> and 1
     * @param Ks a percentage between <code>Kd</code> and 1
     * @param Kf
     */
    public void setIDSF(double Ki, double Kd, double Ks, double Kf) {
        setIDS(Ki, Kd, Ks);
        setF(Kf);
    }

    /**
     * Sets i, d, s, and f values using an array
     *
     * @param K an array of 4 values, the first being Ki, 2nd is Kd, 3rd Ks, 4th
     * Kf
     */
    public void setIDSF(double[] K) {
        if (K.length < 4) {
            return;
        }
        setIDS(K);
        setF(K[3]);

    }

    /**
     * set the feedforward constant
     *
     * @param Kf a value between <code>minOutput</code> and
     * <code>maxOutput</code>
     */
    public void setF(double Kf) {
        if (Kf < minOutput) {
            f = minOutput;
        } else if (Kf > maxOutput) {
            f = maxOutput;
        } else {
            f = Kf;
        }

    }

    public double getI() {
        return i;
    }

    public double getD() {
        return d;
    }

    /**
     * sets setpoint to 0 and startinginput to the current source value
     */
    public void reset() {
        setSetpoint(0);
        startingInput = source.pidGet();
    }

    public double getS() {
        return s;
    }

    public double getF() {
        return f;
    }
}
