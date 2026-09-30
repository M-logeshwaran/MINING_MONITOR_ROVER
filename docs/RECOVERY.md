# DrillPulse Recovery Subsystem & Rollover Righting

## Overview

Underground mine drift traversal involves irregular rubble piles, rib falls, and steep ditch slopes that can destabilize a 6-wheel rover. The DrillPulse recovery system (`drillpulse_recovery`) integrates real-time IMU attitude monitoring with dual high-torque linear self-righting actuators to autonomously recover from tip-overs and rollovers.

---

## 1. Rollover Detection Algorithm

To eliminate false triggers caused by sudden shock impacts or transient rocker-bogie articulation over rocks:
- **Angle Trigger**: Continuous roll angle |roll| >= 45.0 deg or pitch angle |pitch| >= 45.0 deg.
- **Temporal Filter**: The condition must persist continuously for duration >= 1.0 second.
- **Drive Lockout**: The instant rollover is confirmed, the recovery manager publishes an emergency lock to the mobility controller, halting all 6 drive motors to prevent wheel entanglement.

---

## 2. Recovery Controller State Machine

- **IDLE**: Normal operation; actuators fully retracted.
- **ARMED**: Rollover confirmed; drive motors locked; telemetry alert broadcasted.
- **EXTENDING**: Target actuator powered (max 3.5s stroke).
- **HOLDING**: 1.5s stabilization delay; verify ground settling.
- **RETRACTING**: Actuators retracted back to home position (after 2.0s cooldown).
- **SUCCESS**: IMU verifies |roll| < 15 deg; drive interlocks cleared.

---

## 3. Righting Physics & Directional Selection

The recovery node resolves the sign of the roll angle:
- **Right Rollover (roll >= +45 deg)**: Left Actuator extends downwards against the tunnel floor, exerting an upward restoring moment.
- **Left Rollover (roll <= -45 deg)**: Right Actuator extends.
- **Both Jacks Extend**: If a pitch-axis forward rollover occurs, both linear actuators deploy symmetrically.

---

## 4. Hardware Safety & Operator Overrides

1. **Max Stroke Duration (3.5 Seconds)**: Both firmware and software enforce a hard 3.5s power limit to prevent actuator stall and motor burnout at physical stroke limits.
2. **Thermal Cooldown (2.0 Seconds)**: Reversing motor direction requires a 2000 ms pause to extinguish back-EMF spikes and protect relay contacts.
3. **Manual Overrides**: Key `6` (Extend), Key `7` (Retract), Key `ESC` (Stop).
