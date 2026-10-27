package autopilot.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.CsvFileSource;
import org.junit.jupiter.params.provider.MethodSource;
import java.util.stream.Stream;

class BrakeControllerTest {
    private static final double ASSERTION_DELTA = 1e-9;
    private static final double ABS_PRESSURE_THRESHOLD = 80.0;
    private static final double ABS_SPEED_THRESHOLD_KMH = 30.0;
    private static final double PRESSURE_ABOVE_ABS_THRESHOLD = 80.0001;
    private static final double SPEED_ABOVE_ABS_THRESHOLD_KMH = 30.0001;
    private static final double MAX_BRAKE_PRESSURE = 100.0;
    private static final double ABS_HIGH_PULSE_PRESSURE = 90.0;
    private static final double ABS_LOW_PULSE_PRESSURE = 60.0;
    private static final double ZERO_PRESSURE = 0.0;
    private static final double LOW_REQUESTED_PRESSURE = 20.0;
    private static final double NORMAL_REQUESTED_PRESSURE = 35.0;
    private static final double APPLIED_TEST_PRESSURE = 42.5;
    private static final double PARK_GEAR_TEST_PRESSURE = 50.0;
    private static final double LOW_SPEED_KMH = 10.0;
    private static final double NORMAL_SPEED_KMH = 20.0;
    private static final double ABS_TEST_SPEED_KMH = 50.0;
    private static final double EXPECTED_NORMAL_PRESSURE = 80.0;
    private static final double EXPECTED_LOW_SPEED_PRESSURE = 100.0;
    private static final int CSV_HEADER_LINES_TO_SKIP = 1;
    private static final String VALID_CASES_RESOURCE = "/autopilot/core/brake-controller-valid-cases.csv";
    private static final String INVALID_CASES_RESOURCE = "/autopilot/core/brake-controller-invalid-cases.csv";
    private static final String ABS_THRESHOLD_SCENARIOS_SOURCE = "absThresholdScenarios";
    private static final String NON_PARK_GEAR_REQUIRED_MESSAGE = "GearBox.Gear must contain at least one non-park gear";

    @ParameterizedTest
    @CsvFileSource(resources = VALID_CASES_RESOURCE, numLinesToSkip = CSV_HEADER_LINES_TO_SKIP)
    void apply_ValidRequestsWithoutAbsActivation_ReturnsBoundedPressure(double requestedPressure, double speedKmh, double expectedPressure) {
        BrakeController controller = new BrakeController();
        double actualPressure = controller.apply(requestedPressure, speedKmh, nonParkGear());
        assertEquals(expectedPressure, actualPressure, ASSERTION_DELTA);
    }

    @ParameterizedTest
    @CsvFileSource(resources = INVALID_CASES_RESOURCE, numLinesToSkip = CSV_HEADER_LINES_TO_SKIP
    )
    void apply_InvalidPressureOrSpeed_ThrowsIllegalArgumentException(double requestedPressure, double speedKmh) {
        BrakeController controller = new BrakeController();
        assertThrows(IllegalArgumentException.class, () -> controller.apply(requestedPressure, speedKmh, nonParkGear()));
    }

    @ParameterizedTest
    @MethodSource(ABS_THRESHOLD_SCENARIOS_SOURCE)
    void apply_AtOrAboveAbsThresholds_UsesStrictGreaterThanConditions(double requestedPressure, double speedKmh, double expectedPressure) {
        BrakeController controller = new BrakeController();
        double actualPressure = controller.apply(requestedPressure, speedKmh, nonParkGear());
        assertEquals(expectedPressure, actualPressure, ASSERTION_DELTA);
    }

    static Stream<Arguments> absThresholdScenarios() {
        return Stream.of(Arguments.of(ABS_PRESSURE_THRESHOLD, ABS_TEST_SPEED_KMH, ABS_PRESSURE_THRESHOLD),
                Arguments.of(MAX_BRAKE_PRESSURE, ABS_SPEED_THRESHOLD_KMH, MAX_BRAKE_PRESSURE),
                Arguments.of(PRESSURE_ABOVE_ABS_THRESHOLD, ABS_TEST_SPEED_KMH, ABS_HIGH_PULSE_PRESSURE),
                Arguments.of(MAX_BRAKE_PRESSURE, SPEED_ABOVE_ABS_THRESHOLD_KMH, ABS_HIGH_PULSE_PRESSURE)
        );
    }

    @Test
    void apply_AbsConditionsMet_AlternatesHighAndLowPulses() {
        BrakeController controller = new BrakeController();
        GearBox.Gear gear = nonParkGear();
        double firstPressure = controller.apply(MAX_BRAKE_PRESSURE, ABS_TEST_SPEED_KMH, gear);
        double secondPressure = controller.apply(MAX_BRAKE_PRESSURE, ABS_TEST_SPEED_KMH, gear);
        double thirdPressure = controller.apply(MAX_BRAKE_PRESSURE, ABS_TEST_SPEED_KMH, gear);
        assertEquals(ABS_HIGH_PULSE_PRESSURE, firstPressure, ASSERTION_DELTA);
        assertEquals(ABS_LOW_PULSE_PRESSURE, secondPressure, ASSERTION_DELTA);
        assertEquals(ABS_HIGH_PULSE_PRESSURE, thirdPressure, ASSERTION_DELTA);
    }

    @Test
    void apply_PressureAtOrBelowAbsThreshold_ResetsPulseSequence() {
        BrakeController controller = new BrakeController();
        GearBox.Gear gear = nonParkGear();
        controller.apply(MAX_BRAKE_PRESSURE, ABS_TEST_SPEED_KMH, gear);
        double normalPressure = controller.apply(ABS_PRESSURE_THRESHOLD, ABS_TEST_SPEED_KMH, gear);
        double nextAbsPressure = controller.apply(MAX_BRAKE_PRESSURE, ABS_TEST_SPEED_KMH, gear);
        assertEquals(ABS_PRESSURE_THRESHOLD, normalPressure, ASSERTION_DELTA);
        assertEquals(ABS_HIGH_PULSE_PRESSURE, nextAbsPressure, ASSERTION_DELTA);
    }

    @Test
    void apply_SpeedAtOrBelowAbsThreshold_ResetsPulseSequence() {
        BrakeController controller = new BrakeController();
        GearBox.Gear gear = nonParkGear();
        controller.apply(MAX_BRAKE_PRESSURE, ABS_TEST_SPEED_KMH, gear);
        double normalPressure = controller.apply(MAX_BRAKE_PRESSURE, ABS_SPEED_THRESHOLD_KMH, gear);
        double nextAbsPressure = controller.apply(MAX_BRAKE_PRESSURE, ABS_TEST_SPEED_KMH, gear);
        assertEquals(MAX_BRAKE_PRESSURE, normalPressure, ASSERTION_DELTA);
        assertEquals(ABS_HIGH_PULSE_PRESSURE, nextAbsPressure, ASSERTION_DELTA);
    }

    @Test
    void apply_ParkGear_ThrowsIllegalStateException() {
        BrakeController controller = new BrakeController();
        assertThrows(IllegalStateException.class,() -> controller.apply(PARK_GEAR_TEST_PRESSURE, NORMAL_SPEED_KMH, GearBox.Gear.P));
    }

    @Test
    void apply_NullGear_ThrowsNullPointerException() {
        BrakeController controller = new BrakeController();
        assertThrows(NullPointerException.class,() -> controller.apply(PARK_GEAR_TEST_PRESSURE, NORMAL_SPEED_KMH, null));
    }

    @Test
    void getPressure_InitialState_ReturnsZero() {
        BrakeController controller = new BrakeController();
        assertEquals(ZERO_PRESSURE, controller.getPressure(), ASSERTION_DELTA);
    }

    @Test
    void getPressure_AfterApply_ReturnsAppliedPressure() {
        BrakeController controller = new BrakeController();
        controller.apply(APPLIED_TEST_PRESSURE, NORMAL_SPEED_KMH, nonParkGear());
        assertEquals(APPLIED_TEST_PRESSURE, controller.getPressure(), ASSERTION_DELTA);
    }

    @Test
    void isWarningActive_InitialState_ReturnsFalse() {
        BrakeController controller = new BrakeController();
        assertFalse(controller.isWarningActive());
    }

    @Test
    void emergencyBrake_NonParkGear_SetsMaximumPressure() {
        BrakeController controller = new BrakeController();
        controller.apply(NORMAL_REQUESTED_PRESSURE, LOW_SPEED_KMH, nonParkGear());
        controller.emergencyBrake(nonParkGear());
        assertEquals(MAX_BRAKE_PRESSURE, controller.getPressure(), ASSERTION_DELTA);
    }

    @Test
    void emergencyBrake_NonParkGear_ActivatesWarning() {
        BrakeController controller = new BrakeController();
        controller.emergencyBrake(nonParkGear());
        assertTrue(controller.isWarningActive());
    }

    @Test
    void emergencyBrake_ParkGear_ThrowsIllegalStateException() {
        BrakeController controller = new BrakeController();
        assertThrows(IllegalStateException.class,() -> controller.emergencyBrake(GearBox.Gear.P));
    }

    @Test
    void emergencyBrake_NullGear_ThrowsNullPointerException() {
        BrakeController controller = new BrakeController();
        assertThrows(NullPointerException.class, () -> controller.emergencyBrake(null));
    }

    @Test
    void emergencyBrake_AfterAbsActivity_ResetsPulseSequence() {
        BrakeController controller = new BrakeController();
        GearBox.Gear gear = nonParkGear();
        double firstAbsPressure = controller.apply(MAX_BRAKE_PRESSURE, ABS_TEST_SPEED_KMH, gear);
        controller.emergencyBrake(gear);
        double pressureAfterEmergencyBrake = controller.apply(MAX_BRAKE_PRESSURE, ABS_TEST_SPEED_KMH, gear);
        assertEquals(ABS_HIGH_PULSE_PRESSURE, firstAbsPressure, ASSERTION_DELTA);
        assertEquals(ABS_HIGH_PULSE_PRESSURE, pressureAfterEmergencyBrake, ASSERTION_DELTA);
    }

    @Test
    void isWarningActive_AfterNormalApply_RemainsTrue() {
        BrakeController controller = new BrakeController();
        controller.emergencyBrake(nonParkGear());
        controller.apply(LOW_REQUESTED_PRESSURE, LOW_SPEED_KMH, nonParkGear());
        assertTrue(controller.isWarningActive());
    }

    private static GearBox.Gear nonParkGear() {
        for (GearBox.Gear gear : GearBox.Gear.values()) {
            if (gear != GearBox.Gear.P) 
                return gear;
        }
        throw new AssertionError(NON_PARK_GEAR_REQUIRED_MESSAGE);
    }
}
