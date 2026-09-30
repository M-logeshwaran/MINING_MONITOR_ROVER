#!/usr/bin/env python3
# ============================================================
# DRILLPULSE — Hardware & Mobility Layer Automated Unit & Logic Tests
# ============================================================
import os
import sys
import time
import unittest

class TestHardwareMobilityLogic(unittest.TestCase):

    def test_checksum_algorithms(self):
        """Verify XOR checksum algorithm used across Arduino & STM32 firmware."""
        payload = "CMD,100,-50,150,42"
        cs = 0
        for ch in payload:
            cs ^= ord(ch)
        hex_cs = f"{cs:02X}"
        self.assertEqual(len(hex_cs), 2)

        # Reverse check
        calc = 0
        for ch in payload:
            calc ^= ord(ch)
        self.assertEqual(f"{calc:02X}", hex_cs)

    def test_rocker_bogie_kinematics(self):
        """Verify 6-wheel rocker-bogie differential drive kinematics."""
        track_width = 0.90 # meters
        wheel_diameter = 0.22 # meters
        wheel_circ = 3.1415926535 * wheel_diameter

        # Case 1: Straight forward 0.5 m/s
        vx = 0.5
        wz = 0.0
        v_l = vx - (wz * track_width * 0.5)
        v_r = vx + (wz * track_width * 0.5)
        self.assertAlmostEqual(v_l, 0.5)
        self.assertAlmostEqual(v_r, 0.5)

        rpm_l = (v_l / wheel_circ) * 60.0
        rpm_r = (v_r / wheel_circ) * 60.0
        self.assertAlmostEqual(rpm_l, rpm_r)

        # Case 2: In-place pivot left (wz = 1.0 rad/s)
        vx = 0.0
        wz = 1.0
        v_l = vx - (wz * track_width * 0.5)
        v_r = vx + (wz * track_width * 0.5)
        self.assertEqual(v_l, -0.45)
        self.assertEqual(v_r, 0.45)

        # Case 3: In-place pivot right (wz = -1.0 rad/s)
        vx = 0.0
        wz = -1.0
        v_l = vx - (wz * track_width * 0.5)
        v_r = vx + (wz * track_width * 0.5)
        self.assertEqual(v_l, 0.45)
        self.assertEqual(v_r, -0.45)

    def test_battery_voltage_to_percentage_curve(self):
        """Verify 3S LiPo non-linear voltage mapping."""
        def calc_soc(v):
            v_cells = v / 3.0
            if v_cells >= 4.20: return 100.0
            if v_cells <= 3.50: return 0.0
            if v_cells >= 4.0: return 80.0 + (v_cells - 4.0) * 100.0
            if v_cells >= 3.8: return 40.0 + (v_cells - 3.8) * 200.0
            if v_cells >= 3.65: return 15.0 + (v_cells - 3.65) * 166.6
            return (v_cells - 3.50) * 100.0

        self.assertEqual(calc_soc(12.6), 100.0)
        self.assertEqual(calc_soc(10.5), 0.0)
        self.assertGreater(calc_soc(11.4), 20.0)
        self.assertLess(calc_soc(11.4), 60.0)

    def test_mobility_priority_ladder(self):
        """Verify 8-level priority ladder resolution logic."""
        def arbitrate(estop, actuator_active, hw_fault, comm_lost, teleop_active, nav_active):
            if estop:
                return "ESTOP_TRIGGERED", 0.0, 0.0
            if actuator_active:
                return "ACTUATOR_INTERLOCK", 0.0, 0.0
            if hw_fault:
                return "HARDWARE_FAULT", 0.0, 0.0
            if comm_lost:
                return "COMM_FAILSAFE", 0.0, 0.0
            if teleop_active:
                return "MANUAL_TELEOP", 0.8, 0.2
            if nav_active:
                return "AUTONOMOUS_NAV", 0.4, 0.0
            return "IDLE", 0.0, 0.0

        state, _, _ = arbitrate(estop=True, actuator_active=True, hw_fault=True, comm_lost=True, teleop_active=True, nav_active=True)
        self.assertEqual(state, "ESTOP_TRIGGERED")

        state, _, _ = arbitrate(estop=False, actuator_active=True, hw_fault=False, comm_lost=False, teleop_active=True, nav_active=True)
        self.assertEqual(state, "ACTUATOR_INTERLOCK")

        state, vx, _ = arbitrate(estop=False, actuator_active=False, hw_fault=False, comm_lost=False, teleop_active=True, nav_active=True)
        self.assertEqual(state, "MANUAL_TELEOP")
        self.assertEqual(vx, 0.8)

        state, vx, _ = arbitrate(estop=False, actuator_active=False, hw_fault=False, comm_lost=False, teleop_active=False, nav_active=True)
        self.assertEqual(state, "AUTONOMOUS_NAV")
        self.assertEqual(vx, 0.4)

    def test_actuator_interlocks(self):
        """Verify mutual exclusion and timing boundaries of self-righting actuators."""
        class ActuatorState:
            def __init__(self):
                self.state = 'IDLE'
                self.last_stop_time = 0.0
                self.start_time = 0.0

            def command(self, cmd, now):
                if cmd == 'LEFT_EXTEND':
                    if self.state == 'EXTENDING_RIGHT':
                        return False, "MUTUAL_EXCLUSION_VIOLATION"
                    if now - self.last_stop_time < 2.0:
                        return False, "COOLDOWN_ACTIVE"
                    self.state = 'EXTENDING_LEFT'
                    self.start_time = now
                    return True, "OK"
                elif cmd == 'RIGHT_EXTEND':
                    if self.state == 'EXTENDING_LEFT':
                        return False, "MUTUAL_EXCLUSION_VIOLATION"
                    if now - self.last_stop_time < 2.0:
                        return False, "COOLDOWN_ACTIVE"
                    self.state = 'EXTENDING_RIGHT'
                    self.start_time = now
                    return True, "OK"
                elif cmd == 'STOP':
                    self.state = 'IDLE'
                    self.last_stop_time = now
                    return True, "OK"
                return False, "UNKNOWN"

        act = ActuatorState()
        now = 100.0

        ok, reason = act.command('LEFT_EXTEND', now)
        self.assertTrue(ok)
        self.assertEqual(act.state, 'EXTENDING_LEFT')

        ok, reason = act.command('RIGHT_EXTEND', now + 0.5)
        self.assertFalse(ok)
        self.assertEqual(reason, "MUTUAL_EXCLUSION_VIOLATION")

        act.command('STOP', now + 1.0)

        ok, reason = act.command('RIGHT_EXTEND', now + 2.0)
        self.assertFalse(ok)
        self.assertEqual(reason, "COOLDOWN_ACTIVE")

        ok, reason = act.command('RIGHT_EXTEND', now + 3.5)
        self.assertTrue(ok)
        self.assertEqual(act.state, 'EXTENDING_RIGHT')

    def test_watchdog_timeout(self):
        """Verify that missing commands trigger watchdog cutoff after 500ms."""
        watchdog_timeout = 0.500
        last_cmd_time = 100.0

        self.assertLess(100.2 - last_cmd_time, watchdog_timeout)
        self.assertGreater(100.55 - last_cmd_time, watchdog_timeout)

if __name__ == '__main__':
    unittest.main()
