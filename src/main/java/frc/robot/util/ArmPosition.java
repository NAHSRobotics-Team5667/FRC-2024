package frc.robot.util;

/**
 * ArmPosition.java
 * 
 * To be used solely as a class to hold information regarding Arm Positions.
 * Constructed for ease of use.
 */
public class ArmPosition {
    private double firstPivotAngle; // stores first pivot angle of the arm position
    private double secondPivotAngle; // second pivot angle of the arm position

    public ArmPosition() {
        firstPivotAngle = 0;
        secondPivotAngle = 0;
    }

    public ArmPosition(double firstPivotAngle, double secondPivotAngle) {
        this.firstPivotAngle = firstPivotAngle;
        this.secondPivotAngle = secondPivotAngle;
    }

    /**
     * Sets the first pivot angle to a new value.
     * 
     * @param firstPivotAngle value to set first pivot to.
     */
    public void setFirstPivot(double firstPivotAngle) {
        this.firstPivotAngle = firstPivotAngle;
    }

    /**
     * @return first pivot angle of arm.
     */
    public double getFirstPivot() {
        return firstPivotAngle;
    }

    /**
     * Sets the second pivot angle to a new value.
     * 
     * @param secondPivotAngle value to set second pivot to.
     */
    public void setSecondPivot(double secondPivotAngle) {
        this.secondPivotAngle = secondPivotAngle;
    }

    /**
     * @return second pivot angle of arm.
     */
    public double getSecondPivot() {
        return secondPivotAngle;
    }

    /**
     * @param armPosition arm position to be evaluated.
     * @return whether position is legal.
     */
    public static boolean isLegal(ArmPosition armPosition) {
        return false; // TODO: functionality for determining if an arm position is legal
    }

    /**
     * Prints the Arm Position attributes.
     */
    public String toString() {
        return String.format("Arm Position: (%f deg, %f deg)", firstPivotAngle, secondPivotAngle);
    }
}
