import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.assertEquals;

public class SwerveModuleTest {

    @Test
    public void testGetAngle() {
        SwerveModule swerveModule = new SwerveModule();
        Rotation2d expectedAngle = Rotation2d.fromDegrees(0); // Set your expected angle here

        Rotation2d actualAngle = swerveModule.getAngle();

        assertEquals(expectedAngle, actualAngle);
    }
}