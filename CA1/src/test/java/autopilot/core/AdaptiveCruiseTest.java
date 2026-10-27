package autopilot.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.CsvFileSource;
import org.junit.jupiter.params.provider.EnumSource;
import org.junit.jupiter.params.provider.MethodSource;
import autopilot.core.AdaptiveCruise.Decision;
import autopilot.core.AdaptiveCruise.Mode;

class AdaptiveCruiseTest {
    private static final double TARGET_SPEED_KMH = 120.0;
    private static final double TEST_SPEED_KMH = 100.0;
    private static final double ASSERTION_DELTA = 1e-9;
    private static final int CSV_HEADER_LINES_TO_SKIP = 1;
    private static final double ACCELERATION_DISTANCE_METERS = 70.0;
    private static final double INITIAL_BRAKING_DISTANCE_METERS = 40.0;
    private static final double DISTANCE_BELOW_SAFE_METERS = 45.0;
    private static final double SAFE_DISTANCE_METERS = 50.0;
    private static final double DISTANCE_WITHIN_RELEASE_BAND_METERS = 55.0;
    private static final double EMERGENCY_DISTANCE_METERS = 25.0;
    private static final double RELEASE_DISTANCE_METERS = 60.0;
    private static final double FAR_OBSTACLE_DISTANCE_METERS = 100.0;
    private static final double NEAR_ZERO_DISTANCE_METERS = 0.1;
    private static final double NEGATIVE_SPEED_KMH = -1.0;
    private static final double STANDSTILL_SPEED_KMH = 0.0;
    private static final double ONE_KMH_BELOW_TARGET_SPEED = 119.0;
    private static final double HALF_KMH_BELOW_TARGET_SPEED = 119.5;
    private static final double ONE_KMH_ABOVE_TARGET_SPEED = 121.0;
    private static final double NO_CONTROL_AMOUNT = 0.0;
    private static final double MAX_ACCELERATION_AMOUNT = 5.0;
    private static final double ACCELERATION_AMOUNT_ONE_KMH_BELOW_TARGET = 1.0;
    private static final double ACCELERATION_AMOUNT_HALF_KMH_BELOW_TARGET = 0.5;
    private static final double BRAKING_AMOUNT_FOR_INITIAL_DISTANCE = 20.0;
    private static final double MAINTAINED_BRAKING_AMOUNT = 10.0;
    private static final double EMERGENCY_BRAKE_AMOUNT = 100.0;
    private static final String SAFE_DISTANCE_CASES_RESOURCE = "/autopilot/core/safe-distance-cases.csv";
    private static final String INVALID_INPUTS_RESOURCE = "/autopilot/core/adaptive-cruise-invalid-inputs.csv";
    private static final String INVALID_SPEED_SCENARIOS_SOURCE = "invalidSpeedScenarios";
    private static final String SPEED_TARGET_SCENARIOS_SOURCE = "speedTargetScenarios";
    private static final String BRAKING_SCENARIOS_SOURCE = "brakingScenarios";
    private static final String UNEXPECTED_DECISION_MODE_MESSAGE = "Unexpected decision mode";
    private static final String UNEXPECTED_DECISION_AMOUNT_MESSAGE = "Unexpected decision amount";

    @ParameterizedTest
    @CsvFileSource(resources = SAFE_DISTANCE_CASES_RESOURCE, numLinesToSkip = CSV_HEADER_LINES_TO_SKIP)
    void safeDistanceFor_ValidSpeed_ReturnsHalfOfSpeed(double speedKmh, double expectedDistanceMeters) {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);
        double actualDistance = cruise.safeDistanceFor(speedKmh);
        assertEquals(expectedDistanceMeters, actualDistance, ASSERTION_DELTA);
    }

    @ParameterizedTest
    @MethodSource(INVALID_SPEED_SCENARIOS_SOURCE)
    void safeDistanceFor_NegativeOrNonFiniteSpeed_ThrowsIllegalArgumentException(double invalidSpeedKmh) {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);
        assertThrows(IllegalArgumentException.class,() -> cruise.safeDistanceFor(invalidSpeedKmh));
    }

    static Stream<Double> invalidSpeedScenarios() {
        return Stream.of(NEGATIVE_SPEED_KMH, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY);
    }

    @ParameterizedTest
    @EnumSource(Mode.class)
    void update_EachDecisionMode_ReturnsExpectedModeAndAmount(Mode expectedMode) {
        double distanceMeters = switch (expectedMode) {
            case ACCELERATE -> ACCELERATION_DISTANCE_METERS;
            case HOLD -> SAFE_DISTANCE_METERS;
            case BRAKE -> INITIAL_BRAKING_DISTANCE_METERS;
            case EMERGENCY_BRAKE -> EMERGENCY_DISTANCE_METERS;
        };
        Decision expectedDecision = switch (expectedMode) {
            case ACCELERATE -> new Decision(Mode.ACCELERATE, MAX_ACCELERATION_AMOUNT);
            case HOLD -> new Decision(Mode.HOLD, NO_CONTROL_AMOUNT);
            case BRAKE -> new Decision(Mode.BRAKE, BRAKING_AMOUNT_FOR_INITIAL_DISTANCE);
            case EMERGENCY_BRAKE -> new Decision(Mode.EMERGENCY_BRAKE, EMERGENCY_BRAKE_AMOUNT);
        };
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);
        Decision actualDecision = cruise.update(TEST_SPEED_KMH, distanceMeters);
        assertDecisionEquals(expectedDecision, actualDecision);
    }

    @ParameterizedTest
    @MethodSource(SPEED_TARGET_SCENARIOS_SOURCE)
    void update_SpeedBelowAtOrAboveTarget_ReturnsLimitedAccelerationOrHold(double speedKmh, double targetSpeedKmh, double distanceMeters, Mode expectedMode, double expectedAmount) {
        AdaptiveCruise cruise = new AdaptiveCruise(targetSpeedKmh);
        Decision actualDecision = cruise.update(speedKmh, distanceMeters);
        Decision expectedDecision = new Decision(expectedMode, expectedAmount);
        assertDecisionEquals(expectedDecision, actualDecision);
    }

    static Stream<Arguments> speedTargetScenarios() {
        return Stream.of(
                Arguments.of(STANDSTILL_SPEED_KMH, TARGET_SPEED_KMH, NEAR_ZERO_DISTANCE_METERS, Mode.ACCELERATE, MAX_ACCELERATION_AMOUNT),
                Arguments.of(ONE_KMH_BELOW_TARGET_SPEED, TARGET_SPEED_KMH, FAR_OBSTACLE_DISTANCE_METERS, Mode.ACCELERATE, ACCELERATION_AMOUNT_ONE_KMH_BELOW_TARGET),
                Arguments.of(HALF_KMH_BELOW_TARGET_SPEED, TARGET_SPEED_KMH, FAR_OBSTACLE_DISTANCE_METERS, Mode.ACCELERATE, ACCELERATION_AMOUNT_HALF_KMH_BELOW_TARGET),
                Arguments.of(TARGET_SPEED_KMH, TARGET_SPEED_KMH, FAR_OBSTACLE_DISTANCE_METERS, Mode.HOLD, NO_CONTROL_AMOUNT),
                Arguments.of(ONE_KMH_ABOVE_TARGET_SPEED, TARGET_SPEED_KMH, FAR_OBSTACLE_DISTANCE_METERS, Mode.HOLD, NO_CONTROL_AMOUNT)
        );
    }

    @ParameterizedTest
    @CsvFileSource(resources = INVALID_INPUTS_RESOURCE, numLinesToSkip = CSV_HEADER_LINES_TO_SKIP)
    void update_NegativeOrNonFiniteSpeedOrDistance_ThrowsIllegalArgumentException(double speedKmh, double distanceMeters) {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);
        assertThrows(IllegalArgumentException.class,() -> cruise.update(speedKmh, distanceMeters));
    }

    @ParameterizedTest
    @MethodSource(BRAKING_SCENARIOS_SOURCE)
    void update_BrakingActiveBelowReleaseDistance_MaintainsBraking(double distanceMeters, Decision expectedDecision) {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);
        cruise.update(TEST_SPEED_KMH, INITIAL_BRAKING_DISTANCE_METERS);
        Decision actualDecision = cruise.update(TEST_SPEED_KMH, distanceMeters);
        assertDecisionEquals(expectedDecision, actualDecision);
    }

    static Stream<Arguments> brakingScenarios() {
        return Stream.of(
                Arguments.of(DISTANCE_BELOW_SAFE_METERS, new Decision(Mode.BRAKE, MAINTAINED_BRAKING_AMOUNT)),
                Arguments.of(SAFE_DISTANCE_METERS, new Decision(Mode.BRAKE, MAINTAINED_BRAKING_AMOUNT)),
                Arguments.of(DISTANCE_WITHIN_RELEASE_BAND_METERS, new Decision(Mode.BRAKE, MAINTAINED_BRAKING_AMOUNT))
        );
    }

    @Test
    void update_BrakingActiveAtReleaseDistance_ReleasesBrakingAndAccelerates() {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);
        cruise.update(TEST_SPEED_KMH, INITIAL_BRAKING_DISTANCE_METERS);
        Decision actualDecision = cruise.update(TEST_SPEED_KMH, RELEASE_DISTANCE_METERS);
        assertDecisionEquals(new Decision(Mode.ACCELERATE, MAX_ACCELERATION_AMOUNT), actualDecision);
    }

    @Test
    void update_BrakingReleasedThenDistanceFallsBelowReleaseThreshold_RemainsReleasedAndAccelerates() {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);
        cruise.update(TEST_SPEED_KMH, INITIAL_BRAKING_DISTANCE_METERS);
        cruise.update(TEST_SPEED_KMH, RELEASE_DISTANCE_METERS);
        Decision actualDecision = cruise.update(TEST_SPEED_KMH, DISTANCE_WITHIN_RELEASE_BAND_METERS);
        assertDecisionEquals(new Decision(Mode.ACCELERATE, MAX_ACCELERATION_AMOUNT), actualDecision);
    }

    private static void assertDecisionEquals(Decision expected, Decision actual) {
        assertEquals(expected.mode(), actual.mode(), UNEXPECTED_DECISION_MODE_MESSAGE);
        assertEquals(expected.amount(), actual.amount(), ASSERTION_DELTA, UNEXPECTED_DECISION_AMOUNT_MESSAGE);
    }
}