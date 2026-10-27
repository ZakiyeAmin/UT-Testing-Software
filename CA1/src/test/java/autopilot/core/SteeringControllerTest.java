package autopilot.core;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.EnumSource;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

class SteeringControllerTest {
    private static final double ASSERTION_DELTA = 1.0e-9;
    private static final double ZERO_DEGREES = 0.0;
    private static final double ZERO_SPEED_KMH = 0.0;
    private static final double LOW_SPEED_KMH = 100.0;
    private static final double SPEED_MARGIN_ABOVE_THRESHOLD_KMH = 1.0;
    private static final double PHYSICAL_LIMIT_DEGREES = 450.0;
    private static final double HIGH_SPEED_LIMIT_DEGREES = 90.0;
    private static final double HIGH_SPEED_THRESHOLD_KMH = 120.0;
    private static final double HIGH_SPEED_TEST_KMH = HIGH_SPEED_THRESHOLD_KMH + SPEED_MARGIN_ABOVE_THRESHOLD_KMH;
    private static final double MAX_CHANGE_PER_TICK_DEGREES = 60.0;
    private static final double POSITIVE_DIRECTION_SIGN = 1.0;
    private static final double NEGATIVE_DIRECTION_SIGN = -1.0;
    private static final double INVALID_NEGATIVE_SPEED_KMH = -1.0;
    private static final double ANGLE_WITHIN_ONE_TICK_DEGREES = 30.0;
    private static final double ANGLE_TWO_TICKS_AWAY_DEGREES = 120.0;
    private static final double ANGLE_THREE_TICKS_AWAY_DEGREES = 180.0;
    private static final double ANGLE_ABOVE_PHYSICAL_LIMIT_DEGREES = 510.0;
    private static final double ANGLE_ABOVE_HIGH_SPEED_LIMIT_DEGREES = 150.0;
    private static final String ZERO_SPEED_WITHIN_ONE_TICK_CASE = ANGLE_WITHIN_ONE_TICK_DEGREES + ", " + ZERO_SPEED_KMH + ", "
                                                                + ANGLE_WITHIN_ONE_TICK_DEGREES + ", " + ANGLE_WITHIN_ONE_TICK_DEGREES;
    private static final String POSITIVE_ONE_TICK_AT_LOW_SPEED_CASE = MAX_CHANGE_PER_TICK_DEGREES + ", " + LOW_SPEED_KMH + ", "
                                                                    + MAX_CHANGE_PER_TICK_DEGREES + ", " + MAX_CHANGE_PER_TICK_DEGREES;
    private static final String NEGATIVE_ONE_TICK_AT_LOW_SPEED_CASE = -MAX_CHANGE_PER_TICK_DEGREES + ", " + LOW_SPEED_KMH + ", "
                                                                    + -MAX_CHANGE_PER_TICK_DEGREES + ", " + -MAX_CHANGE_PER_TICK_DEGREES;
    private static final String POSITIVE_PHYSICAL_LIMIT_CASE = PHYSICAL_LIMIT_DEGREES + ", " + LOW_SPEED_KMH + ", "
                                                             + PHYSICAL_LIMIT_DEGREES + ", " + MAX_CHANGE_PER_TICK_DEGREES;
    private static final String NEGATIVE_PHYSICAL_LIMIT_CASE = -PHYSICAL_LIMIT_DEGREES + ", " + LOW_SPEED_KMH + ", "
                                                             + -PHYSICAL_LIMIT_DEGREES + ", " + -MAX_CHANGE_PER_TICK_DEGREES;
    private static final String ABOVE_POSITIVE_PHYSICAL_LIMIT_CASE = ANGLE_ABOVE_PHYSICAL_LIMIT_DEGREES + ", " + LOW_SPEED_KMH + ", "
                                                                   + PHYSICAL_LIMIT_DEGREES + ", " + MAX_CHANGE_PER_TICK_DEGREES;
    private static final String ABOVE_NEGATIVE_PHYSICAL_LIMIT_CASE = -ANGLE_ABOVE_PHYSICAL_LIMIT_DEGREES + ", " + LOW_SPEED_KMH + ", "
                                                                   + -PHYSICAL_LIMIT_DEGREES + ", " + -MAX_CHANGE_PER_TICK_DEGREES;
    private static final String POSITIVE_HIGH_SPEED_LIMIT_CASE = HIGH_SPEED_LIMIT_DEGREES + ", " + HIGH_SPEED_TEST_KMH + ", "
                                                               + HIGH_SPEED_LIMIT_DEGREES + ", " + MAX_CHANGE_PER_TICK_DEGREES;
    private static final String NEGATIVE_HIGH_SPEED_LIMIT_CASE = -HIGH_SPEED_LIMIT_DEGREES + ", " + HIGH_SPEED_TEST_KMH + ", "
                                                                 + -HIGH_SPEED_LIMIT_DEGREES + ", " + -MAX_CHANGE_PER_TICK_DEGREES;
    private static final String ABOVE_POSITIVE_HIGH_SPEED_LIMIT_CASE = ANGLE_ABOVE_HIGH_SPEED_LIMIT_DEGREES + ", " + HIGH_SPEED_TEST_KMH + ", "
                                                                     + HIGH_SPEED_LIMIT_DEGREES + ", " + MAX_CHANGE_PER_TICK_DEGREES;
    private static final String ABOVE_NEGATIVE_HIGH_SPEED_LIMIT_CASE = -ANGLE_ABOVE_HIGH_SPEED_LIMIT_DEGREES + ", " + HIGH_SPEED_TEST_KMH + ", "
                                                                     + -HIGH_SPEED_LIMIT_DEGREES + ", " + -MAX_CHANGE_PER_TICK_DEGREES;
    private static final String INVALID_ANGLE_EXCEPTION_MESSAGE_PREFIX = "Expected an exception for invalid angle: ";
    private static final String INVALID_SPEED_EXCEPTION_MESSAGE_PREFIX = "Expected an exception for invalid speed: ";
    private static final String UNEXPECTED_CURRENT_ANGLE_MESSAGE = "Unexpected current steering angle";
    private static final String UNEXPECTED_TARGET_ANGLE_MESSAGE = "Unexpected target steering angle";

    private enum SteeringDirection {
        POSITIVE(POSITIVE_DIRECTION_SIGN),
        NEGATIVE(NEGATIVE_DIRECTION_SIGN);
        private final double sign;
        SteeringDirection(double sign) {
            this.sign = sign;
        }
    }

    @ParameterizedTest
    @CsvSource({
            ZERO_SPEED_WITHIN_ONE_TICK_CASE,
            POSITIVE_ONE_TICK_AT_LOW_SPEED_CASE,
            NEGATIVE_ONE_TICK_AT_LOW_SPEED_CASE,
            POSITIVE_PHYSICAL_LIMIT_CASE,
            NEGATIVE_PHYSICAL_LIMIT_CASE,
            ABOVE_POSITIVE_PHYSICAL_LIMIT_CASE,
            ABOVE_NEGATIVE_PHYSICAL_LIMIT_CASE,
            POSITIVE_HIGH_SPEED_LIMIT_CASE,
            NEGATIVE_HIGH_SPEED_LIMIT_CASE,
            ABOVE_POSITIVE_HIGH_SPEED_LIMIT_CASE,
            ABOVE_NEGATIVE_HIGH_SPEED_LIMIT_CASE
    })
    void update_RespectsAngleLimitsAndMovementPerTick(double requestedAngleDegrees, double speedKmh, double expectedTargetAngleDegrees, double expectedCurrentAngleDegrees) {
        SteeringController controller = new SteeringController();
        double actualAngle = controller.update(requestedAngleDegrees, speedKmh);
        assertEquals(expectedCurrentAngleDegrees, actualAngle, ASSERTION_DELTA);
        assertControllerState(controller, expectedCurrentAngleDegrees, expectedTargetAngleDegrees);
    }

    @Test
    void update_UsesStrictSpeedThresholdImmediatelyBelowAtAndAboveIt() {
        SteeringController belowThresholdController = new SteeringController();

        belowThresholdController.update(ANGLE_ABOVE_PHYSICAL_LIMIT_DEGREES, Math.nextDown(HIGH_SPEED_THRESHOLD_KMH));
        assertControllerState(belowThresholdController, MAX_CHANGE_PER_TICK_DEGREES, PHYSICAL_LIMIT_DEGREES);
        SteeringController atThresholdController = new SteeringController();
        atThresholdController.update(ANGLE_ABOVE_PHYSICAL_LIMIT_DEGREES, HIGH_SPEED_THRESHOLD_KMH);
        assertControllerState(atThresholdController, MAX_CHANGE_PER_TICK_DEGREES, PHYSICAL_LIMIT_DEGREES);
        SteeringController aboveThresholdController = new SteeringController();
        aboveThresholdController.update(ANGLE_ABOVE_PHYSICAL_LIMIT_DEGREES, Math.nextUp(HIGH_SPEED_THRESHOLD_KMH));
        assertControllerState(aboveThresholdController, MAX_CHANGE_PER_TICK_DEGREES, HIGH_SPEED_LIMIT_DEGREES);
    }

    @Test
    void update_RepeatedRequestsTowardDistantTarget_LimitsMovementPerTick() {
        SteeringController controller = new SteeringController();
        double firstTickAngle = controller.update(ANGLE_THREE_TICKS_AWAY_DEGREES, LOW_SPEED_KMH);
        double secondTickAngle = controller.update(ANGLE_THREE_TICKS_AWAY_DEGREES, LOW_SPEED_KMH);
        double thirdTickAngle = controller.update(ANGLE_THREE_TICKS_AWAY_DEGREES, LOW_SPEED_KMH);
        assertEquals(MAX_CHANGE_PER_TICK_DEGREES, firstTickAngle, ASSERTION_DELTA);
        assertEquals(ANGLE_TWO_TICKS_AWAY_DEGREES, secondTickAngle, ASSERTION_DELTA);
        assertEquals(ANGLE_THREE_TICKS_AWAY_DEGREES, thirdTickAngle, ASSERTION_DELTA);
        assertEquals(ANGLE_THREE_TICKS_AWAY_DEGREES, controller.getTargetAngleDegrees(),  ASSERTION_DELTA);
    }

    @Test
    void update_ChangingTargetReversesMovementTowardTheNewTarget() {
        SteeringController controller = new SteeringController();
        double firstAngle = controller.update(ANGLE_THREE_TICKS_AWAY_DEGREES, LOW_SPEED_KMH);
        assertEquals(MAX_CHANGE_PER_TICK_DEGREES, firstAngle, ASSERTION_DELTA);
        double secondAngle = controller.update(-ANGLE_THREE_TICKS_AWAY_DEGREES, LOW_SPEED_KMH);
        assertEquals(ZERO_DEGREES, secondAngle, ASSERTION_DELTA);
        assertControllerState(controller, ZERO_DEGREES, -ANGLE_THREE_TICKS_AWAY_DEGREES);
    }

    @Test
    void update_NonFiniteRequestedAngles_ThrowAndPreserveExistingState() {
        double[] invalidAngles = {
                Double.NaN,
                Double.POSITIVE_INFINITY,
                Double.NEGATIVE_INFINITY
        };
        for (double invalidAngleDegrees : invalidAngles) {
            SteeringController controller = controllerWithNonZeroState();
            double previousCurrentAngle = controller.getCurrentAngleDegrees();
            double previousTargetAngle = controller.getTargetAngleDegrees();
            assertThrows(IllegalArgumentException.class, () -> controller.update( invalidAngleDegrees, LOW_SPEED_KMH), INVALID_ANGLE_EXCEPTION_MESSAGE_PREFIX + invalidAngleDegrees);
            assertControllerState(controller, previousCurrentAngle, previousTargetAngle);
        }
    }

    @Test
    void update_InvalidSpeeds_ThrowAndPreserveExistingState() {
        double[] invalidSpeeds = {
                INVALID_NEGATIVE_SPEED_KMH,
                Double.NaN,
                Double.POSITIVE_INFINITY,
                Double.NEGATIVE_INFINITY
        };
        for (double invalidSpeedKmh : invalidSpeeds) {
            SteeringController controller = controllerWithNonZeroState();
            double previousCurrentAngle = controller.getCurrentAngleDegrees();
            double previousTargetAngle = controller.getTargetAngleDegrees();
            assertThrows(IllegalArgumentException.class, () -> controller.update(ANGLE_ABOVE_HIGH_SPEED_LIMIT_DEGREES, invalidSpeedKmh), INVALID_SPEED_EXCEPTION_MESSAGE_PREFIX + invalidSpeedKmh);
            assertControllerState(controller, previousCurrentAngle, previousTargetAngle);
        }
    }

    @ParameterizedTest
    @EnumSource(SteeringDirection.class)
    void selfCenter_MovesTowardZeroFromEitherDirection(SteeringDirection direction) {
        SteeringController controller = new SteeringController();
        double startingAngle = direction.sign * ANGLE_THREE_TICKS_AWAY_DEGREES;
        controller.update(startingAngle, LOW_SPEED_KMH);
        controller.update(startingAngle, LOW_SPEED_KMH);
        controller.update(startingAngle, LOW_SPEED_KMH);
        assertEquals(startingAngle, controller.getCurrentAngleDegrees(), ASSERTION_DELTA);
        double actualAngle = controller.selfCenter();
        assertEquals(direction.sign * ANGLE_TWO_TICKS_AWAY_DEGREES, actualAngle, ASSERTION_DELTA);
        assertControllerState(controller, actualAngle, ZERO_DEGREES);
    }

    @Test
    void selfCenter_CurrentAngleWithinOneTick_ReachesZero() {
        SteeringController controller = new SteeringController();
        controller.update(ANGLE_WITHIN_ONE_TICK_DEGREES, LOW_SPEED_KMH);
        double actualAngle = controller.selfCenter();
        assertEquals(ZERO_DEGREES, actualAngle, ASSERTION_DELTA);
        assertControllerState(controller, ZERO_DEGREES, ZERO_DEGREES);
    }

    private static SteeringController controllerWithNonZeroState() {
        SteeringController controller = new SteeringController();
        controller.update(ANGLE_THREE_TICKS_AWAY_DEGREES, LOW_SPEED_KMH);
        return controller;
    }

    private static void assertControllerState(SteeringController controller, double expectedCurrentAngleDegrees, double expectedTargetAngleDegrees) {
        assertEquals(expectedCurrentAngleDegrees, controller.getCurrentAngleDegrees(), ASSERTION_DELTA, UNEXPECTED_CURRENT_ANGLE_MESSAGE);
        assertEquals(expectedTargetAngleDegrees, controller.getTargetAngleDegrees(), ASSERTION_DELTA, UNEXPECTED_TARGET_ANGLE_MESSAGE);
    }
}