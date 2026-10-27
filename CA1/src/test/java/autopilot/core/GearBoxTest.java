
package autopilot.core;

import autopilot.exception.IllegalGearChangeException;
import autopilot.exception.SensorFailureException;
import autopilot.sensor.SpeedSensor;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.CsvFileSource;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.EnumSource;
import org.junit.jupiter.params.provider.MethodSource;
import org.junit.jupiter.params.provider.ValueSource;

import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.*;


class GearBoxTest {

    private static final String SENSOR_FAILURE_MESSAGE = "Speed sensor failed during test";
    private static final String SAME_GEAR_TEST_DISPLAY_NAME = "Current gear {0} remains unchanged";
    private static final String REVERSE_ENTRY_TEST_DISPLAY_NAME = "P to R at {0} km/h: allowed = {1}";
    private static final String GEAR_TRANSITION_TEST_DISPLAY_NAME = "{0} to {1} at {2} km/h: allowed = {3}";
    private static final String GEAR_TRANSITION_CASES_SOURCE = "gearTransitionCases";
    private static final String NEGATIVE_SPEED_TEST_DISPLAY_NAME = "Negative speed {0} km/h is rejected";
    private static final String NON_FINITE_SPEED_TEST_DISPLAY_NAME = "Non-finite speed {0} is rejected";
    private static final String INVALID_SPEEDS_RESOURCE = "/autopilot/core/invalid-speeds.csv";
    private static final String INVALID_SPEED_ASSERTION_MESSAGE = "Invalid speed should cause IllegalArgumentException";
    private static final String EXPECTED_GEAR_STATE_MESSAGE = "Current gear should match the expected transition result";
    private static final String NULL_TARGET_MESSAGE = "A null target gear must be rejected";
    private static final String SENSOR_FAILURE_EXPECTED_MESSAGE = "Sensor failure should propagate without changing the gear";

    private static final double ZERO_SPEED_KMH = 0.0;
    private static final double NONZERO_SPEED_KMH = 1.0;
    private static final double POSITIVE_SPEED_KMH = 30.0;
    private static final double NEGATIVE_SPEED_KMH = -1.0;
    private static final double MORE_NEGATIVE_SPEED_KMH = -10.0;

    private static final String REVERSE_AT_ZERO_SPEED_CASE = "0.0, true";
    private static final String REVERSE_BELOW_LIMIT_CASE = "4.999, true";
    private static final String REVERSE_AT_LIMIT_CASE = "5.0, false";
    private static final String REVERSE_ABOVE_LIMIT_CASE = "5.1, false";

    @ParameterizedTest(name = SAME_GEAR_TEST_DISPLAY_NAME)
    @EnumSource(GearBox.Gear.class)
    void changeTo_whenTargetIsCurrentGear_keepsCurrentGear(
            GearBox.Gear currentGear) {

        AtomicReference<Double> speed =
                new AtomicReference<>(ZERO_SPEED_KMH);

        GearBox gearBox = createGearBoxInGear(
                currentGear,
                speed
        );

        gearBox.setSpeedSensor(() -> {
            throw new SensorFailureException(
                    SENSOR_FAILURE_MESSAGE
            );
        });

        assertDoesNotThrow(
                () -> gearBox.changeTo(currentGear)
        );

        assertEquals(
                currentGear,
                gearBox.getCurrentGear(),
                EXPECTED_GEAR_STATE_MESSAGE
        );
    }

    @ParameterizedTest(name = REVERSE_ENTRY_TEST_DISPLAY_NAME)
    @CsvSource({
            REVERSE_AT_ZERO_SPEED_CASE,
            REVERSE_BELOW_LIMIT_CASE,
            REVERSE_AT_LIMIT_CASE,
            REVERSE_ABOVE_LIMIT_CASE
    })
    void changeTo_fromParkToReverse_respectsSpeedLimit(
            double speedKmh,
            boolean expectedToSucceed) {

        AtomicReference<Double> speed =
                new AtomicReference<>(ZERO_SPEED_KMH);

        GearBox gearBox = createGearBoxInGear(
                GearBox.Gear.P,
                speed
        );

        speed.set(speedKmh);

        assertTransitionOutcome(
                gearBox,
                GearBox.Gear.P,
                GearBox.Gear.R,
                expectedToSucceed
        );
    }

    @ParameterizedTest(name = GEAR_TRANSITION_TEST_DISPLAY_NAME)
    @MethodSource(GEAR_TRANSITION_CASES_SOURCE)
    void changeTo_validatesGearTransitionRules(
            GearBox.Gear fromGear,
            GearBox.Gear targetGear,
            double speedKmh,
            boolean expectedToSucceed) {

        AtomicReference<Double> speed =
                new AtomicReference<>(ZERO_SPEED_KMH);

        GearBox gearBox = createGearBoxInGear(
                fromGear,
                speed
        );

        speed.set(speedKmh);

        assertTransitionOutcome(
                gearBox,
                fromGear,
                targetGear,
                expectedToSucceed
        );
    }


    private static Stream<Arguments> gearTransitionCases() {
        return Stream.of(
                Arguments.of(
                        GearBox.Gear.R,
                        GearBox.Gear.D,
                        ZERO_SPEED_KMH,
                        true
                ),
                Arguments.of(
                        GearBox.Gear.R,
                        GearBox.Gear.D,
                        NONZERO_SPEED_KMH,
                        false
                ),
                Arguments.of(
                        GearBox.Gear.D,
                        GearBox.Gear.R,
                        ZERO_SPEED_KMH,
                        true
                ),
                Arguments.of(
                        GearBox.Gear.D,
                        GearBox.Gear.R,
                        NONZERO_SPEED_KMH,
                        false
                ),
                Arguments.of(
                        GearBox.Gear.R,
                        GearBox.Gear.N,
                        POSITIVE_SPEED_KMH,
                        true
                ),
                Arguments.of(
                        GearBox.Gear.N,
                        GearBox.Gear.D,
                        POSITIVE_SPEED_KMH,
                        true
                ),
                Arguments.of(
                        GearBox.Gear.P,
                        GearBox.Gear.N,
                        ZERO_SPEED_KMH,
                        false
                ),
                Arguments.of(
                        GearBox.Gear.P,
                        GearBox.Gear.D,
                        ZERO_SPEED_KMH,
                        false
                ),
                Arguments.of(
                        GearBox.Gear.R,
                        GearBox.Gear.P,
                        ZERO_SPEED_KMH,
                        false
                ),
                Arguments.of(
                        GearBox.Gear.N,
                        GearBox.Gear.P,
                        ZERO_SPEED_KMH,
                        false
                ),
                Arguments.of(
                        GearBox.Gear.N,
                        GearBox.Gear.R,
                        ZERO_SPEED_KMH,
                        false
                ),
                Arguments.of(
                        GearBox.Gear.D,
                        GearBox.Gear.P,
                        ZERO_SPEED_KMH,
                        false
                ),
                Arguments.of(
                        GearBox.Gear.D,
                        GearBox.Gear.N,
                        ZERO_SPEED_KMH,
                        false
                )
        );
    }

    @ParameterizedTest(name = NEGATIVE_SPEED_TEST_DISPLAY_NAME)
    @ValueSource(doubles = {
            NEGATIVE_SPEED_KMH,
            MORE_NEGATIVE_SPEED_KMH
    })
    void changeTo_whenSpeedIsNegative_rejectsGearChange(
            double invalidSpeedKmh) {

        assertInvalidSpeedRejected(invalidSpeedKmh);
    }
    @ParameterizedTest(name = NON_FINITE_SPEED_TEST_DISPLAY_NAME)
    @CsvFileSource(
            resources = INVALID_SPEEDS_RESOURCE,
            numLinesToSkip = 1
    )
    void changeTo_whenSpeedIsNotFinite_rejectsGearChange(
            double invalidSpeedKmh) {

        assertInvalidSpeedRejected(invalidSpeedKmh);
    }


    private static void assertInvalidSpeedRejected(
            double invalidSpeedKmh) {

        GearBox gearBox = new GearBox(
                () -> invalidSpeedKmh
        );

        assertThrows(
                IllegalArgumentException.class,
                () -> gearBox.changeTo(GearBox.Gear.R),
                INVALID_SPEED_ASSERTION_MESSAGE
        );

        assertEquals(
                GearBox.Gear.P,
                gearBox.getCurrentGear(),
                EXPECTED_GEAR_STATE_MESSAGE
        );
    }
    @Test
    void changeTo_whenTargetGearIsNull_throwsException() {

        GearBox gearBox = new GearBox(
                () -> ZERO_SPEED_KMH
        );

        assertThrows(
                NullPointerException.class,
                () -> gearBox.changeTo(null),
                NULL_TARGET_MESSAGE
        );

        assertEquals(
                GearBox.Gear.P,
                gearBox.getCurrentGear(),
                EXPECTED_GEAR_STATE_MESSAGE
        );
    }
    @Test
    void changeTo_whenSpeedSensorFails_propagatesException() {

        SpeedSensor failingSpeedSensor = () -> {
            throw new SensorFailureException(
                    SENSOR_FAILURE_MESSAGE
            );
        };

        GearBox gearBox = new GearBox(
                failingSpeedSensor
        );

        assertThrows(
                SensorFailureException.class,
                () -> gearBox.changeTo(GearBox.Gear.R),
                SENSOR_FAILURE_EXPECTED_MESSAGE
        );

        assertEquals(
                GearBox.Gear.P,
                gearBox.getCurrentGear(),
                EXPECTED_GEAR_STATE_MESSAGE
        );
    }
    private static GearBox createGearBoxInGear(
            GearBox.Gear targetGear,
            AtomicReference<Double> speed) {

        speed.set(ZERO_SPEED_KMH);

        GearBox gearBox = new GearBox(
                speed::get
        );

        switch (targetGear) {
            case P -> {
            }

            case R -> gearBox.changeTo(GearBox.Gear.R);

            case N -> {
                gearBox.changeTo(GearBox.Gear.R);
                gearBox.changeTo(GearBox.Gear.N);
            }

            case D -> {
                gearBox.changeTo(GearBox.Gear.R);
                gearBox.changeTo(GearBox.Gear.N);
                gearBox.changeTo(GearBox.Gear.D);
            }
        }

        return gearBox;
    }


    private static void assertTransitionOutcome(
            GearBox gearBox,
            GearBox.Gear fromGear,
            GearBox.Gear targetGear,
            boolean expectedToSucceed) {

        if (expectedToSucceed) {
            assertDoesNotThrow(
                    () -> gearBox.changeTo(targetGear)
            );
        } else {
            assertThrows(
                    IllegalGearChangeException.class,
                    () -> gearBox.changeTo(targetGear)
            );
        }

        GearBox.Gear expectedGear = expectedToSucceed
                ? targetGear
                : fromGear;

        assertEquals(
                expectedGear,
                gearBox.getCurrentGear(),
                EXPECTED_GEAR_STATE_MESSAGE
        );
    }
}
