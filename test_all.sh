#!/usr/bin/env bash
# ============================================================
# DRILLPULSE — Automated Integrated Test Suite (Tests 1 to 12)
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/setup_environment.sh"

if [ -f "${SCRIPT_DIR}/rover_ws/install/setup.bash" ]; then
    source "${SCRIPT_DIR}/rover_ws/install/setup.bash"
fi

if [ -f "${SCRIPT_DIR}/station_ws/install/setup.bash" ]; then
    source "${SCRIPT_DIR}/station_ws/install/setup.bash"
fi

echo "========================================================="
echo "DRILLPULSE — AUTOMATED VALIDATION TEST SUITE"
echo "========================================================="

FAILURES=0

run_test() {
    local test_name="$1"
    local script="$2"
    echo ""
    echo ">>> Running ${test_name} (${script})..."
    if python3 "${SCRIPT_DIR}/scripts/${script}"; then
        echo ">>> ${test_name}: SUCCESS"
    else
        echo ">>> ${test_name}: FAILED"
        FAILURES=$((FAILURES + 1))
    fi
}

# Run all test modules
run_test "Test 1: Source Code Integrity" "test_source_integrity.py"
run_test "Test 2: Canonical Topic Contracts" "test_topics.py"
run_test "Test 3: TF Hierarchy & Transforms" "test_tf_tree.py"
run_test "Test 4: Odometry Modes & Covariances" "test_odometry.py"
run_test "Test 5: Threshold Consistency (5-State)" "test_threshold.py"
run_test "Tests 6, 7, 8: Nav2 & Mode Separation" "test_navigation.py"
run_test "Test 9: Dual-Panel Dashboard Locking" "test_dashboard_lock.py"
run_test "Test 10: Station <-> Rover Communication" "test_communication.py"
run_test "Test 11: Recovery Actuators & Safety" "test_recovery.py"
run_test "Test 12: Hardware & Mobility Layer Integration" "test_hardware_mobility.py"
run_test "Test 13: Full Navigation, SLAM & Sensor Integration (16 Checks)" "test_navigation_integration.py"
run_test "Test 14: Communication, Adaptive Telemetry & Dashboard Validation (17 Checks)" "test_communication_adaptive.py"
run_test "Test 15: Mission Resilience & Connection-Aware Returns (22 Checks)" "test_mission_resilience.py"
run_test "Test 16: Mission Orchestration & Autonomous Exploration (20 Checks)" "test_mission_orchestration.py"
run_test "Test 17: Robust Communication & Adaptive Telemetry (20 Checks)" "test_robust_communication.py"

echo ""
echo "========================================================="
if [ $FAILURES -eq 0 ]; then
    echo "ALL 17 TESTS PASSED PERFECTLY!"
    echo "========================================================="
    exit 0
else
    echo "${FAILURES} TEST(S) FAILED!"
    echo "========================================================="
    exit 1
fi
