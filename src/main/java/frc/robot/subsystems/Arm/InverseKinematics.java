// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

/** Inverse Kinematics */
public class InverseKinematics {
    public class ModuleConstants {
        public static final int ARM_SHOULDER_LENGTH = 24;
        public static final double ARM_ELBOW_LENGTH = 24;
    }

    public static class DoubleJointedArmAngles {
        public double shoulderAngleRads;
        public double elbowAngleRads;

        public DoubleJointedArmAngles() {
            shoulderAngleRads = 0;
            elbowAngleRads = 0;
        }
    };

    /**
     * Get the angles of the joints of a double jointed arm.
     * 
     * @return
     */
    public static DoubleJointedArmAngles getDoubleJointedArmAngles(double x, double y) {
        DoubleJointedArmAngles angles = new DoubleJointedArmAngles();

        double a1 = ModuleConstants.ARM_SHOULDER_LENGTH;
        double a2 = ModuleConstants.ARM_ELBOW_LENGTH;

        // Calculate the shoulder angle
        double q2 = - Math.acos((x * x + y * y - (a1 * a1) - (a2 * a2)) / (2 * a1 * a2));
        angles.elbowAngleRads = -q2;

        // Calculate the elbow angle
        double q1 = Math.atan2(y, x)
                + Math.atan2(a2 * Math.sin(q2), a1 + a2 * Math.cos(q2));
        angles.shoulderAngleRads = q1;
        return angles;
    }

    /**
     * Get the angles of the joints of a double jointed arm.
     * This keeps the shoulder above the elbow.
     * the shoulder angle is positive from the x-axis counter-clockwise.
     * The elbow angle is positive from the shoulder arm clockwise.
     * e.g. to get to the point (30, 30) gives s:90, e:90.
     * the shoulder angle is 90 degrees on the y-axis and the elbow angle is 90 degrees to the x-axis.
     * @param x
     * @param y
     * @return
     */
    public static DoubleJointedArmAngles getDoubleJointedArmAnglesB(double x, double y) {
        DoubleJointedArmAngles angles = new DoubleJointedArmAngles();

        double a1 = ModuleConstants.ARM_SHOULDER_LENGTH;
        double a2 = ModuleConstants.ARM_ELBOW_LENGTH;

        // Calculate the shoulder angle
        double q2 = Math.acos((x * x + y * y - (a1 * a1) - (a2 * a2)) / (2 * a1 * a2));
        angles.elbowAngleRads = -q2;

        // Calculate the elbow angle
        double q1 = Math.atan2(y, x)
                + Math.atan2(a2 * Math.sin(q2), a1 + a2 * Math.cos(q2));
        angles.shoulderAngleRads = q1;

        // System.out.println(String.format("a2: %.4f, q2: %.1f, sin(q2): %.4f, a1: %.4f, cos(q2): %.4f", a2 , q2, Math.sin(q2), a1 ,Math.cos(q2)));
        return angles;
    }

    public static DoubleJointedArmAngles getPreferredArmAngles(double x, double y) {
        var a1 = getDoubleJointedArmAngles(x, y);
        var a2 = getDoubleJointedArmAnglesB(x, y);
        if( Math.sin(a1.shoulderAngleRads) > Math.sin(a2.shoulderAngleRads)) {
            return a1;
        }
        return a2;
    }
    /**
     * Convert radians to degrees.
     * 
     * @param radians
     * @return degrees
     */
    public static double radiansToDegrees(double radians) {
        return radians * (180.0 / Math.PI);
    }

    static void test(double x, double y) {
        var pt1 = getDoubleJointedArmAngles(x, y);
        var pt2 = getDoubleJointedArmAnglesB(x, y);
        var pt3 = getPreferredArmAngles(x, y);
        System.out.println(String.format("Target: (%.1f, %.1f),  %.1f, %.1f or %.1f, %.1f     %.1f, %.1f",
                x, y, 
                radiansToDegrees(pt1.shoulderAngleRads), 
                radiansToDegrees(pt1.elbowAngleRads), 
                radiansToDegrees(pt2.shoulderAngleRads), 
                radiansToDegrees(pt2.elbowAngleRads),
                radiansToDegrees(pt3.shoulderAngleRads), 
                radiansToDegrees(pt3.elbowAngleRads)
                // radiansToDegrees(Math.atan2(y, x)
                // )
                )
                );
    }

    public static void main(String[] args) {
        test(30, 0);
        test(30, 10);
        test(30, 20);
        test(30, 30);
        test(20, 30);
        test(10, 30);
        test(0, 30);
        test(-10, 30);
        test(-20, 30);
        test(-30, 30);
        test(-30, 20);
        test(-30, 10);
        test(-30, 0);

        // test(25, 30);
        // test(20, 30);
        // test(15, 30);
        // test(10, 30);
        // test(5, 30);
        // test(0, 30);
    }

}
