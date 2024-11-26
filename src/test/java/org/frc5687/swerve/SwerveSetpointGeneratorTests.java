package org.frc5687.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import org.frc5687.lib.swerve.SwerveSetpointGenerator;
import org.frc5687.lib.swerve.SwerveSetpointGenerator.Function2d;

public class SwerveSetpointGeneratorTests {
    @Test
    public void functionOneRootIsFound() {
        Function2d func = (x, y) -> Math.sqrt(x*x+y*y)-5.0;
        var s = SwerveSetpointGenerator.findRoot(func, 1.0, 2.0, 3.0, 5.0, 100);
        assertEquals(0.769231, s, 0.00001); // found on desmos i made this https://www.desmos.com/calculator/rjhgpw9ah4 -- xavier
    }

    @Test
    public void functionTwoRootIsFound() {
        Function2d func = (x, y) -> Math.sin(x)*Math.cos(y)*Math.cos(y)-x*y;
        var s = SwerveSetpointGenerator.findRoot(func, -0.2, 2.0, 0.1, 5, 100);
        assertEquals(0.666667, s, 0.00001); // found on desmos i made this https://www.desmos.com/calculator/rjhgpw9ah4 -- xavier
    }

    @Test
    public void functionThreeRootIsFound() {
        Function2d func = (x, y) -> 2.0*x-3.0*y;
        var s = SwerveSetpointGenerator.findRoot(func, 4.9, 1.5, 0.1, 5, 100);
        assertEquals(0.263682, s, 0.00001); // found on desmos i made this https://www.desmos.com/calculator/rjhgpw9ah4 -- xavier
    }

    @Test
    public void functionFourRootIsFound() {
        Function2d func = (x, y) -> Math.sqrt(x*x+y*y)-5.0;
        var s = SwerveSetpointGenerator.findRoot(func, 4.9, 4.3, -0.6, 4.9, 100);
        assertEquals(0.546869, s, 0.00001); // found on desmos i made this https://www.desmos.com/calculator/rjhgpw9ah4 -- xavier
    }
}
