package autopilot.core;

import autopilot.exception.SensorFailureException;
import autopilot.sensor.LaneSensor;
import autopilot.sensor.ObstacleSensor;
import autopilot.sensor.SpeedSensor;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

import java.util.stream.Stream;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class AutoDriverTest {

    private static final String NORMAL_DRIVING_BEHAVIOR = "DriveResult should match normal driving behavior";
    private static final String LANE_DEVIATION_TEST_DISPLAY_NAME = "lane offset = {0} m, expect negative steering = {1}";
    private static final String LANE_DEVIATION_CASES_SOURCE = "laneDeviationCases";
    private static final String POSITIVE_LANE_STEERING_MESSAGE = "Positive lane offset should produce negative steering";
    private static final String NEGATIVE_LANE_STEERING_MESSAGE = "Negative lane offset should produce positive steering";
    private static final String SENSOR_FAILURE_TEST_DISPLAY_NAME = "{0}";
    private static final String SENSOR_FAILURE_CASES_SOURCE = "sensorFailureCases";
    private static final String SENSOR_FAILURE_MESSAGE = "Sensor failed during test";
    private static final String SPEED_SENSOR_FAILURE_SCENARIO = "Speed sensor failure activates safe mode";
    private static final String OBSTACLE_SENSOR_FAILURE_SCENARIO = "Obstacle sensor failure activates safe mode";
    private static final String EXPECTED_SAFE_MODE_RESULT_MESSAGE = "A sensor failure should produce the expected safe-mode result";

    private static final double TARGET_SPEED_KMH = 80.0;
    private static final double OBSTACLE_DISTANCE_METERS = 100.0;
    private static final double CENTERED_LANE_OFFSET_METERS = 0.0;
    private static final double POSITIVE_LANE_OFFSET_METERS = 1.0;
    private static final double NEGATIVE_LANE_OFFSET_METERS =
            -POSITIVE_LANE_OFFSET_METERS;
    private static final double NO_STEERING_ANGLE_DEGREES = 0.0;
    private static final double NO_BRAKE_PRESSURE = 0.0;
    private static final double SAFE_MODE_BRAKE_PRESSURE = 30.0;

    // Expected results
    private static final AutoDriver.DriveResult EXPECTED_NORMAL_DRIVING_RESULT =
            new AutoDriver.DriveResult(
                    NO_STEERING_ANGLE_DEGREES,
                    NO_BRAKE_PRESSURE,
                    AdaptiveCruise.Mode.HOLD,
                    false,
                    false
            );

    private static final AutoDriver.DriveResult EXPECTED_SAFE_MODE_RESULT =
            new AutoDriver.DriveResult(
                    NO_STEERING_ANGLE_DEGREES,
                    SAFE_MODE_BRAKE_PRESSURE,
                    AdaptiveCruise.Mode.BRAKE,
                    false,
                    true
            );

    private SpeedSensor speedSensor;
    private ObstacleSensor obstacleSensor;
    private LaneSensor laneSensor;

    private AutoDriver autoDriver;

    @BeforeEach
    void setUp() {
        speedSensor = () -> TARGET_SPEED_KMH;
        obstacleSensor = () -> OBSTACLE_DISTANCE_METERS;
        laneSensor = () -> CENTERED_LANE_OFFSET_METERS;

        autoDriver = createAutoDriver(
                speedSensor,
                obstacleSensor,
                laneSensor
        );
    }

    @Test
    void tick_withValidSensorReadings_returnsExpectedNormalDrivingResult() {
        AutoDriver.DriveResult result =
                autoDriver.tick(GearBox.Gear.D);

        assertEquals(
                EXPECTED_NORMAL_DRIVING_RESULT,
                result,
                NORMAL_DRIVING_BEHAVIOR
        );
    }

    @ParameterizedTest(name = LANE_DEVIATION_TEST_DISPLAY_NAME)
    @MethodSource(LANE_DEVIATION_CASES_SOURCE)
    void tick_withLaneDeviation_correctsSteeringTowardLaneCenter(
            double laneOffsetMeters,
            boolean expectNegativeSteering) {

        LaneSensor deviatedLaneSensor = () -> laneOffsetMeters;

        AutoDriver driver = createAutoDriver(
                speedSensor,
                obstacleSensor,
                deviatedLaneSensor
        );

        AutoDriver.DriveResult result =
                driver.tick(GearBox.Gear.D);

        double steeringAngle = result.steeringAngleDegrees();

        if (expectNegativeSteering) {
            assertTrue(
                    steeringAngle < NO_STEERING_ANGLE_DEGREES,
                    POSITIVE_LANE_STEERING_MESSAGE
            );
        } else {
            assertTrue(
                    steeringAngle > NO_STEERING_ANGLE_DEGREES,
                    NEGATIVE_LANE_STEERING_MESSAGE
            );
        }
    }

    @ParameterizedTest(name = SENSOR_FAILURE_TEST_DISPLAY_NAME)
    @MethodSource(SENSOR_FAILURE_CASES_SOURCE)
    void tick_whenSensorFails_returnsSafeModeResult(
            String scenarioName,
            SpeedSensor testSpeedSensor,
            ObstacleSensor testObstacleSensor,
            LaneSensor testLaneSensor) {

        AutoDriver driver = createAutoDriver(
                testSpeedSensor,
                testObstacleSensor,
                testLaneSensor
        );

        AutoDriver.DriveResult result =
                driver.tick(GearBox.Gear.D);

        assertEquals(
                EXPECTED_SAFE_MODE_RESULT,
                result,
                EXPECTED_SAFE_MODE_RESULT_MESSAGE
        );
    }

    private AutoDriver createAutoDriver(
            SpeedSensor speedSensor,
            ObstacleSensor obstacleSensor,
            LaneSensor laneSensor) {

        return new AutoDriver(
                speedSensor,
                obstacleSensor,
                laneSensor,
                TARGET_SPEED_KMH
        );
    }

    private static Stream<Arguments> laneDeviationCases() {
        return Stream.of(
                Arguments.of(
                        POSITIVE_LANE_OFFSET_METERS,
                        true
                ),
                Arguments.of(
                        NEGATIVE_LANE_OFFSET_METERS,
                        false
                )
        );
    }

    private static Stream<Arguments> sensorFailureCases() {

        SpeedSensor failingSpeedSensor = () -> {
            throw new SensorFailureException(SENSOR_FAILURE_MESSAGE);
        };

        ObstacleSensor failingObstacleSensor = () -> {
            throw new SensorFailureException(SENSOR_FAILURE_MESSAGE);
        };

        SpeedSensor validSpeedSensor = () -> TARGET_SPEED_KMH;

        ObstacleSensor validObstacleSensor =
                () -> OBSTACLE_DISTANCE_METERS;

        LaneSensor validLaneSensor =
                () -> CENTERED_LANE_OFFSET_METERS;

        return Stream.of(
                Arguments.of(
                        SPEED_SENSOR_FAILURE_SCENARIO,
                        failingSpeedSensor,
                        validObstacleSensor,
                        validLaneSensor
                ),
                Arguments.of(
                        OBSTACLE_SENSOR_FAILURE_SCENARIO,
                        validSpeedSensor,
                        failingObstacleSensor,
                        validLaneSensor
                )
        );
    }
}
