package frc.robot.Utils.MathUtils;

/**
 * a math library written myself for the control algorithm of steer wheels
 * stores a vector sitting a 2d space
 *
 * @author Sam
 * @version 0.1
 */
public class Vector2D {
    private double[] vector;

    /** initialize the vector, with a empty value */
    public Vector2D() {
        vector = new double[] { 0, 0 };
    }

    /**
     * initialize the vector, with a given initial vector value
     *
     * @param initialValue an array with length 2, representing the initial vector
     */
    public Vector2D(double[] initialValue) {
        vector = initialValue;
    }

    /**
     * initialize the vector, given an initial heading and magnitude
     *
     * @param heading   the direction of the vector, in radian, zero is to the left
     *                  and positive is counter-clockwise, like in geometry
     * @param magnitude the magnitude of the vector
     */
    public Vector2D(double heading, double magnitude) {
        vector = new double[] { Math.cos(heading) * magnitude, Math.sin(heading) * magnitude };
    }

    public double[] getValue() {
        return vector;
    }

    public double getX() {
        return getValue()[0];
    }

    public double getY() {
        return getValue()[1];
    }

    @Deprecated
    public void update(double[] newVector) {
        this.vector = newVector;
    }

    @Deprecated
    public void update(int index, double newValue) {
        this.vector[index] = newValue;
    }

    @Deprecated
    public void update(Vector2D target) {
        update(target.vector);
    }

    /** apply a given transformation to this vector */
    public Vector2D multiplyBy(Transformation2D transformation) {
        return transformation.multiply(this);
    }

    /** scale this vector by a given factor */
    public Vector2D multiplyBy(double scaler) {
        double[] newVector = { this.vector[0] * scaler, this.vector[1] * scaler };
        return new Vector2D(newVector);
    }

    public Vector2D addBy(Vector2D adder) {
        double[] newVector = { this.vector[0] + adder.getValue()[0], this.vector[1] + adder.getValue()[1] };
        return new Vector2D(newVector);
    }

    public double getHeading() {
        if (Math.abs(vector[0]) >= 1e-4) // when x is non-zero
            return AngleUtils.simplifyAngle(Math.atan2(vector[1], vector[0])); // arc-tangent will just do the job for us

        /* deal with zero x situations */
        if (vector[1] > 0)
            return Math.PI / 2;
        if (vector[1] < 0)
            return Math.PI * 3 / 2;

        /* when y is also zero */
        return 0; // just return a random angle
    }

    public double getMagnitude() {
        return Math.sqrt(vector[0] * vector[0] + vector[1] * vector[1]);
    }

    @Override
    public boolean equals(Object vector) {
        double[] numbers = ((Vector2D) vector).getValue();
        return this.vector[0] == numbers[0] && this.vector[1] == numbers[1];
    }

    @Override
    public String toString() {
        return "vector with value:\n [ " + vector[0] + " ]\n [ " + vector[1] + " ]";
    }

    public static Vector2D displacementToTarget(Vector2D currentPosition, Vector2D desiredPosition) {
        return desiredPosition.addBy(currentPosition.multiplyBy(-1));
    }
}
