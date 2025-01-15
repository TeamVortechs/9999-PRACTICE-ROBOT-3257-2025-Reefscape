package frc.robot.util;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class Vector {

    private double x;
    private double y;

    public Vector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    //getter/setter functions
    //gets the X component of this vector
    public double getX() {
        return 0;
    }

    //sets the x component of this vector
    public void setX(double x) {
    }

    //gets the y component of this vector
    public double getY() {
        return 0;
    }

    //sets the y component of this vector
    public void setY(double y) {
    }

    //advanced getters

    //gets the angle between this vector and another vector
    public double getAngleRad(Vector vec) {
        return 0;
    }


    //transumation functions
    //multiplies the vector by the x value
    public void multiply(double mul) {
        
    }

    //adds the given vector to this vector
    public void add(Vector vec) {

    }


    //static helper methods

    //converts given chassis speeds into a vector with the x and y component
    public static Vector fromChassisSpeeds(ChassisSpeeds speeds) {
        return null;
    }


    //automatically converts the chassis speeds to vectors and gets the angles between them 
    public static double getAngleRadChassis(ChassisSpeeds speeds1, ChassisSpeeds speeds2) {
        //converts the chassis speeds into vectors
        Vector vec1 = fromChassisSpeeds(speeds1);
        Vector vec2 = fromChassisSpeeds(speeds2);

        //returns the angle between the two vector
        return vec1.getAngleRad(vec2);
    }
    
}

/*
 * Add offsets to chassis speeds - UNTESTED
 * Get wanted chassis speeds
 * Find angle between chassis speeds - OUTLINED
 * Calculate chassis speed weight
 * Apply final offsets
 * Add it to controls
 * 
 */