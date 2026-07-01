"""
SSG-AI Highway VSL Simulation - DYNAMIC TRIGGER VERSION
==========================================================

This is a modified version of vsl_highway_fog.py. The OBU enforcement logic,
two-gate safety check (TTC + headway), packet-loss/latency simulation, and
CSV logging are UNCHANGED from the original validated script.

What's different: in the original script, the speed-limit drop happened at a
fixed, hardcoded step (FOG_TRIGGER_TIME = 300) to a fixed target (60 km/h).
That means the "AI-TCC computes dynamic safe speeds based on weather and
traffic" claim in the paper was never actually exercised - the number and
the timing were both decided by the person writing the script, not by any
live condition in the simulation.

This version replaces that fixed trigger with a live monitoring loop:

  1. A simulated fog event (severity 0->1->0, "rolling in" then clearing,
     starting at a RANDOM step within a window rather than a fixed step)
  2. A live traffic-congestion signal read directly from SUMO each step
     (edge occupancy, via traci.edge.getLastStepOccupancy)

Each step, the AI-TCC's "Dynamic Speed Planner" combines these into a
target speed via a small, transparent cost function (NOT a trained DRL/MPC
model - that is future work, as already noted in the paper). Whenever the
computed target drifts far enough from the currently active target (and a
cooldown has passed, to avoid flapping), the AI-TCC issues a new
ENFORCE_SPEED_TOKEN and the edge speed ramps toward the new target, exactly
as before. This also means the limit can recover back upward once fog
clears or congestion eases - genuine two-way variability, not just one
scripted drop.

Run this exactly like the original: place it in the same folder as
highway_vsl.sumocfg and the highway_*.xml network/route files, then:

    python vsl_highway_dynamic.py

Toggle ENABLE_FOG_TRIGGER / ENABLE_CONGESTION_TRIGGER below to run
fog-only, congestion-only, or combined ablations if you want extra
sub-results for the paper.

UPDATE: all output files (vsl_highway_log.csv, tcc_decision_log.csv, and a
new tcc_run_summary.txt) are now written to the same folder this script
lives in, regardless of what directory you launch it from - if you saw an
old vsl_highway_log.csv that didn't seem to update, it was almost certainly
written to a different working directory than the one you were looking in.
The full console summary is also now saved to tcc_run_summary.txt, so it's
not lost if the console window closes automatically when the run finishes.

NOTE: this has not been run against a live SUMO instance (no SUMO/traci
available in the environment this was written in). Please sanity-check the
printed summary and the new tcc_decision_log.csv against what you'd expect
before trusting these numbers in the paper.
"""

import traci
import json
import time
import csv
import random
import os
import sys
import pandas as pd
import numpy as np

# All output files are anchored to the folder this script lives in, NOT the
# current working directory. This matters because some IDEs / double-click
# launchers run scripts from a different working directory than the script's
# own folder - if that happens with relative paths, Python silently writes
# (or reads) files somewhere other than where you're looking, which looks
# like "it didn't generate a new file" even though it did, just elsewhere.
BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# --- PARAMETERS FOR SCENARIO B/C (Highway VSL) ---
LOG_FILE = os.path.join(BASE_DIR, "vsl_highway_log.csv")
DECISION_LOG_FILE = os.path.join(BASE_DIR, "tcc_decision_log.csv")   # NEW: one row per step, AI-TCC's view of the world
SUMMARY_FILE = os.path.join(BASE_DIR, "tcc_run_summary.txt")          # NEW: saved to disk so it survives the console closing
TRAFFIC_CONTROL_LOG = os.path.join(BASE_DIR, "vsl_traffic_control_log.csv")
FCD_OUTPUT_FILE = os.path.join(BASE_DIR, "fcd_data.xml")

# Speed Conversion (km/h to m/s)
KMH_TO_MS = 1 / 3.6

# VSL SCENARIO PARAMETERS
HIGHWAY_EDGE = "E0"                  # ID of the highway segment defined in highway_network.net.xml
INITIAL_SPEED_MS = 140 * KMH_TO_MS   # Free-flow target speed (~38.89 m/s)
MIN_ENFORCED_SPEED_MS = 60 * KMH_TO_MS  # Floor speed the AI-TCC will not drop below (~16.67 m/s)
SPEED_TRANSITION_STEPS = 50          # Steps over which the edge speed ramps toward a new target

# --- NEW: LIVE TRIGGER PARAMETERS (replaces the old fixed FOG_TRIGGER_TIME) ---
ENABLE_FOG_TRIGGER = True
ENABLE_CONGESTION_TRIGGER = True

# Fog: random onset, ramps in, holds, then clears - a full up/down cycle.
FOG_SEVERITY_ONSET_WINDOW = (200, 350)   # fog starts rolling in at a random step in this range
FOG_RAMP_STEPS = 50                      # steps for severity to go 0 -> 1 (and 1 -> 0 when clearing)
FOG_DURATION_STEPS = 150                 # steps spent at full severity before clearing begins

# Congestion: read live from SUMO each step. Tune these two to your network -
# typical SUMO edge occupancy is low single digits in free flow and climbs
# into the 20-40% range under real congestion.
FREE_FLOW_OCCUPANCY_BASELINE = 8.0       # % occupancy below which congestion factor is 0 (normal traffic)
CONGESTION_OCCUPANCY_THRESHOLD = 20.0    # % occupancy at which congestion factor reaches 1.0

# Re-issuance control, so the AI-TCC doesn't flap between targets every step.
REISSUE_COOLDOWN_STEPS = 20              # minimum steps between two token re-issuances
REISSUE_MARGIN_KMH = 3.0                 # minimum change in computed target before re-issuing
REISSUE_MARGIN_MS = REISSUE_MARGIN_KMH * KMH_TO_MS

# Compliance and Intervention Parameters
WARNING_PERIOD_STEPS = 30            # 30 seconds for drivers to manually adjust speed (Grace Period)
NON_COMPLIANT_COUNT = 100            # 40 'veryAggressive' + 60 'aggressive' drivers
NON_COMPLIANT_VTYPES = {"aggressive", "veryAggressive"}  # Types considered non-compliant

# Safety Parameters (Used by OBU logic, MIN_HEADWAY is proxy for Rear-Gap Check)
MIN_TTC = 1.0                        # Minimum safe Time-To-Collision (s)
MIN_HEADWAY = 1.5                    # Minimum Headway (s)

# --- SCENARIO B PARAMETERS: ADVERSE COMMUNICATIONS ---
# Set both to 0 to run this as "Scenario B" (normal comms) instead.
PACKET_LOSS_RATE = 0.00              # 0% chance of RSU token being lost
MAX_LATENCY_STEPS = 0                # Max delay in steps (seconds) for successful packet

# --- GLOBAL TRACKING FOR NON-COMPLIANCE AND LATENCY ---
GLOBAL_NON_COMPLIANT_DRIVERS = set()
VEHICLE_ENFORCEMENT_SCHEDULE = {}    # Stores the scheduled enforcement step for each vehicle (inf if lost)


# --- HELPER FUNCTIONS: LIVE CONDITION MONITORING (the new "AI-TCC" part) ---
def get_fog_severity(step, onset_step):
    """Returns fog severity in [0, 1]: 0 before onset, ramps up, holds, ramps back down."""
    if not ENABLE_FOG_TRIGGER:
        return 0.0
    if step < onset_step:
        return 0.0
    elapsed = step - onset_step
    if elapsed < FOG_RAMP_STEPS:
        return elapsed / FOG_RAMP_STEPS
    elapsed -= FOG_RAMP_STEPS
    if elapsed < FOG_DURATION_STEPS:
        return 1.0
    elapsed -= FOG_DURATION_STEPS
    if elapsed < FOG_RAMP_STEPS:
        return max(0.0, 1.0 - elapsed / FOG_RAMP_STEPS)
    return 0.0


def get_congestion_factor(occupancy_pct):
    """Returns congestion severity in [0, 1] from live SUMO edge occupancy.
    Occupancy below FREE_FLOW_OCCUPANCY_BASELINE counts as normal traffic (0);
    only occupancy above that baseline contributes to congestion risk."""
    if not ENABLE_CONGESTION_TRIGGER:
        return 0.0
    span = CONGESTION_OCCUPANCY_THRESHOLD - FREE_FLOW_OCCUPANCY_BASELINE
    if span <= 0:
        return 0.0
    return min(1.0, max(0.0, (occupancy_pct - FREE_FLOW_OCCUPANCY_BASELINE) / span))


def compute_target_speed(fog_severity, congestion_factor):
    """
    AI-TCC 'Dynamic Speed Planner' cost function (Section III.G in the paper).
    Transparent rule, not a trained model: respond to whichever risk factor
    is currently worse, and never drop below the floor speed.
    """
    risk = max(fog_severity, congestion_factor)
    target = INITIAL_SPEED_MS - risk * (INITIAL_SPEED_MS - MIN_ENFORCED_SPEED_MS)
    return max(MIN_ENFORCED_SPEED_MS, target)


# --- HELPER FUNCTIONS: LOGGING ---
def save_log_header():
    with open(LOG_FILE, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            "step", "vehicle_id", "speed_ms", "max_allowed_ms", "is_overspeed",
            "ttc_check_fail", "decel_intent_sent", "is_non_compliant", "is_delayed",
            "target_speed_kph"
        ])


def log_data(data):
    with open(LOG_FILE, 'a', newline='') as f:
        writer = csv.writer(f)
        writer.writerows(data)


def save_decision_log(rows):
    """One row per simulation step: what the AI-TCC saw and decided."""
    with open(DECISION_LOG_FILE, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            "step", "fog_severity", "congestion_pct", "computed_target_kph",
            "active_target_kph", "enforced_speed_kph", "token_issued"
        ])
        writer.writerows(rows)


# --- MAIN SIMULATION LOGIC ---
def run_scenario_b_c(sumo_cfg):
    """Runs Scenario B or C: Highway VSL enforcement with optional adverse communications."""
    if "SUMO_HOME" in os.environ:
        sumo_binary = os.path.join(os.environ["SUMO_HOME"], "bin", "sumo")
    else:
        sumo_binary = "sumo"

    step = 0
    scenario_type = "B (Normal Comms)" if PACKET_LOSS_RATE == 0.0 and MAX_LATENCY_STEPS == 0 else "C (Adverse Comms)"

    try:
        traci.start([
            sumo_binary,
            "-c", sumo_cfg,
            "--step-length", "1.0",
            "--fcd-output", FCD_OUTPUT_FILE,
            "--fcd-output.geo",
            "--collision.check-junctions",
            "--collision.action", "none",
            "--device.rerouting.threads", "0"
        ])
    except traci.exceptions.TraCIException as e:
        print(f"FATAL ERROR: Could not start TraCI server with '{sumo_cfg}'.")
        print(f"Error details: {e}")
        return
    except FileNotFoundError:
        print("FATAL ERROR: SUMO executable not found.")
        return

    # Fog onset is randomized per run - the AI-TCC does NOT know this value in
    # advance, it only ever sees get_fog_severity()'s live output each step.
    fog_onset_step = random.randint(*FOG_SEVERITY_ONSET_WINDOW)

    current_target_speed = INITIAL_SPEED_MS   # most recent v_target commanded by the AI-TCC
    current_enforced_speed = INITIAL_SPEED_MS  # live edge speed limit, ramps toward current_target_speed
    transition_active = False
    transition_remaining_steps = 0
    last_issue_step = -10 ** 9
    first_trigger_step = None
    token_issue_steps = []

    # --- EPISODE TRACKING (fixes two bugs found after the first real run) ---
    # Bug 1: is_grace_period was computed from last_issue_step, which gets
    # reset on EVERY reissuance. Because the fog ramp triggers ~4 reissuances
    # ~20 steps apart, the grace window kept getting refreshed before it could
    # ever expire, so non-compliant drivers were never actually forced into
    # compliance (0 across the whole run) - the grace clock now anchors to the
    # START of an episode (a genuinely new event, not an in-progress ramp) and
    # is left alone by subsequent reissuances within that same episode.
    # Bug 2: VEHICLE_ENFORCEMENT_SCHEDULE locked in a vehicle's packet-loss/
    # latency outcome on its FIRST contact with any token and reused that same
    # outcome forever, including for the unrelated, later recovery episode.
    # Each new episode should be an independent RSU broadcast with its own
    # independent chance of being received, so the schedule is now cleared at
    # the start of each new episode too, with running totals kept separately
    # so the final loss-rate stats aren't reset along with it.
    GRACE_EPISODE_GAP_STEPS = WARNING_PERIOD_STEPS + SPEED_TRANSITION_STEPS
    grace_anchor_step = -10 ** 9
    total_attempts_accum = 0
    total_loss_accum = 0

    data_to_log = []
    decision_log_rows = []
    collisions = 0
    throughput_count = 0

    summary_lines = []

    def out(line=""):
        print(line)
        summary_lines.append(str(line))

    traci.edge.setMaxSpeed(HIGHWAY_EDGE, INITIAL_SPEED_MS)
    save_log_header()

    out(f"--- SCENARIO {scenario_type}: DYNAMIC TRIGGER VERSION ---")
    out(f"Fog onset randomly scheduled at step {fog_onset_step} (window {FOG_SEVERITY_ONSET_WINDOW}, "
        f"ramp {FOG_RAMP_STEPS}s, hold {FOG_DURATION_STEPS}s)")
    out(f"Congestion trigger: {'ENABLED' if ENABLE_CONGESTION_TRIGGER else 'DISABLED'} "
        f"(threshold {CONGESTION_OCCUPANCY_THRESHOLD}% occupancy)")
    if scenario_type == "C (Adverse Comms)":
        out(f"Communication Impairment: Packet Loss Rate={PACKET_LOSS_RATE*100:.0f}%, Max Latency={MAX_LATENCY_STEPS}s")

    try:
        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()

            # --- 1. LIVE CONDITION MONITORING (AI-TCC Dynamic Speed Planner) ---
            fog_severity = get_fog_severity(step, fog_onset_step)
            occupancy_pct = traci.edge.getLastStepOccupancy(HIGHWAY_EDGE)
            congestion_factor = get_congestion_factor(occupancy_pct)
            computed_target = compute_target_speed(fog_severity, congestion_factor)

            token_issued_this_step = False
            vsl_token_is_active = False

            if (step - last_issue_step >= REISSUE_COOLDOWN_STEPS) and \
               (abs(computed_target - current_target_speed) > REISSUE_MARGIN_MS):
                # AI-TCC decides conditions have changed enough to issue a new
                # ENFORCE_SPEED_TOKEN with an updated v_target.
                is_new_episode = (step - grace_anchor_step) > GRACE_EPISODE_GAP_STEPS
                current_target_speed = computed_target
                last_issue_step = step
                transition_active = True
                transition_remaining_steps = SPEED_TRANSITION_STEPS
                token_issued_this_step = True
                token_issue_steps.append(step)
                if first_trigger_step is None:
                    first_trigger_step = step
                if is_new_episode:
                    grace_anchor_step = step
                    VEHICLE_ENFORCEMENT_SCHEDULE.clear()
                out(f"[step {step}] AI-TCC issues new token: target={current_target_speed*3.6:.1f} km/h "
                    f"(fog={fog_severity:.2f}, congestion={congestion_factor:.2f}, occupancy={occupancy_pct:.1f}%)"
                    f"{' [NEW EPISODE]' if is_new_episode else ''}")

            if transition_active:
                vsl_token_is_active = True
                step_size = abs(INITIAL_SPEED_MS - MIN_ENFORCED_SPEED_MS) / SPEED_TRANSITION_STEPS
                if current_target_speed > current_enforced_speed:
                    current_enforced_speed = min(current_target_speed, current_enforced_speed + step_size)
                else:
                    current_enforced_speed = max(current_target_speed, current_enforced_speed - step_size)
                traci.edge.setMaxSpeed(HIGHWAY_EDGE, current_enforced_speed)
                transition_remaining_steps -= 1
                if transition_remaining_steps <= 0 or current_enforced_speed == current_target_speed:
                    transition_active = False

            decision_log_rows.append([
                step, round(fog_severity, 3), round(occupancy_pct, 2),
                round(computed_target * 3.6, 2), round(current_target_speed * 3.6, 2),
                round(current_enforced_speed * 3.6, 2), int(token_issued_this_step)
            ])

            # --- 2. OBU ENFORCEMENT & SAFETY LOGIC ---
            step_log = []
            is_grace_period = (step < grace_anchor_step + WARNING_PERIOD_STEPS)

            for vid in traci.vehicle.getIDList():

                if vid not in GLOBAL_NON_COMPLIANT_DRIVERS:
                    v_type = traci.vehicle.getTypeID(vid)
                    if v_type in NON_COMPLIANT_VTYPES:
                        GLOBAL_NON_COMPLIANT_DRIVERS.add(vid)

                is_non_compliant = vid in GLOBAL_NON_COMPLIANT_DRIVERS

                v_speed = traci.vehicle.getSpeed(vid)
                v_max_allowed = traci.vehicle.getAllowedSpeed(vid)

                is_overspeed = v_speed > v_max_allowed

                ttc_fail = 0
                decel_intent = 0
                is_delayed = 0

                enforcement_step = step

                if vsl_token_is_active:
                    if vid not in VEHICLE_ENFORCEMENT_SCHEDULE:
                        if random.random() < PACKET_LOSS_RATE:
                            schedule_step = float('inf')
                            total_loss_accum += 1
                        else:
                            latency = random.randint(0, MAX_LATENCY_STEPS)
                            schedule_step = step + latency
                            if latency > 0:
                                is_delayed = 1
                        total_attempts_accum += 1
                        VEHICLE_ENFORCEMENT_SCHEDULE[vid] = schedule_step
                    enforcement_step = VEHICLE_ENFORCEMENT_SCHEDULE[vid]

                if enforcement_step <= step:
                    if is_overspeed:
                        enforce_speed = True
                        if is_non_compliant and is_grace_period:
                            enforce_speed = False

                        if enforce_speed:
                            leader_id_data = traci.vehicle.getLeader(vid)
                            safe_to_intervene = True

                            if leader_id_data:
                                leader_id, gap = leader_id_data
                                try:
                                    gap = float(gap)
                                except ValueError:
                                    safe_to_intervene = False
                                    ttc_fail = 0
                                    leader_id = None

                                if leader_id:
                                    leader_speed = traci.vehicle.getSpeed(leader_id)
                                    relative_speed = v_speed - leader_speed

                                    if relative_speed > 0:
                                        ttc = gap / relative_speed if relative_speed > 0 else float('inf')
                                        if ttc < MIN_TTC:
                                            safe_to_intervene = False
                                            ttc_fail = 1

                                    if v_speed > 0 and (gap / v_speed < MIN_HEADWAY):
                                        safe_to_intervene = False

                            if safe_to_intervene:
                                traci.vehicle.setSpeed(vid, v_max_allowed)
                                decel_intent = 1

                step_log.append([step, vid, v_speed, v_max_allowed, int(is_overspeed), ttc_fail,
                                  decel_intent, int(is_non_compliant), is_delayed,
                                  round(current_target_speed * 3.6, 2)])

            data_to_log.extend(step_log)

            # --- 3. METRICS GATHERING ---
            throughput_count += len(traci.simulation.getArrivedIDList())
            collisions += len(traci.simulation.getCollisions())
            step += 1

        out(f"\n--- SIMULATION LOOP TERMINATED SUCCESSFULLY AT STEP {step} ---")

    except traci.exceptions.FatalTraCIError as e:
        out(f"\nFATAL TRACI ERROR: Simulation crashed or connection lost at step {step}.")
        out(f"Error: {e}")
        pass

    traci.close()

    # --- 4. RESULTS CALCULATION ---
    log_data(data_to_log)
    save_decision_log(decision_log_rows)

    try:
        df = pd.read_csv(LOG_FILE)
    except FileNotFoundError:
        out("Error: Log file not found for analysis.")
        with open(SUMMARY_FILE, 'w') as f:
            f.write("\n".join(summary_lines))
        return
    except pd.errors.EmptyDataError:
        out("Error: Log file is empty. The simulation likely failed to run or crashed immediately.")
        with open(SUMMARY_FILE, 'w') as f:
            f.write("\n".join(summary_lines))
        return

    # PRE/POST split now uses the ACTUAL first trigger step, not a fixed constant.
    PRE_VSL_STEPS = first_trigger_step if first_trigger_step is not None else step

    df_pre = df[df['step'] < PRE_VSL_STEPS]
    df_post = df[df['step'] >= PRE_VSL_STEPS]

    def calculate_phase_metrics(df_phase, phase_name):
        if df_phase.empty:
            return {'Phase': phase_name, 'Overspeed Events': 0, 'Safety Gate Fails (TTC)': 0,
                    'Avg Speed (km/h)': 0.0, 'Speed Std Dev (m/s)': 0.0}
        overspeed_events = df_phase['is_overspeed'].sum()
        ttc_fails = df_phase['ttc_check_fail'].sum()
        avg_speed_ms = df_phase['speed_ms'].mean()
        speed_std = df_phase['speed_ms'].std() if len(df_phase['speed_ms']) > 1 else 0.0
        return {
            'Phase': phase_name,
            'Overspeed Events': int(overspeed_events),
            'Safety Gate Fails (TTC)': int(ttc_fails),
            'Avg Speed (km/h)': avg_speed_ms * 3.6,
            'Speed Std Dev (m/s)': speed_std,
            'Total Data Points': len(df_phase)
        }

    pre_metrics = calculate_phase_metrics(df_pre, f'PRE-TRIGGER (Steps 0-{PRE_VSL_STEPS-1})')
    post_metrics = calculate_phase_metrics(df_post, f'POST-TRIGGER (Steps {PRE_VSL_STEPS}+)')
    results_df = pd.DataFrame([pre_metrics, post_metrics])

    df_forced_intervened = df[(df['is_non_compliant'] == 1) & (df['decel_intent_sent'] == 1)]
    unique_forced_intervened_drivers = df_forced_intervened['vehicle_id'].nunique()

    # Use accumulators, not the cleared-per-episode schedule dict
    total_attempts = total_attempts_accum
    loss_count = total_loss_accum
    success_count = total_attempts - loss_count
    delayed_count = int(df['is_delayed'].sum())

    CII = 100.0 if collisions == 0 else 0.0

    out("\n" + "=" * 80)
    out(f"=== SCENARIO {scenario_type}: DYNAMIC-TRIGGER VSL SUMMARY ===")
    out(f"Total Simulation Steps: {step}")
    out(f"Final Throughput (Vehicles): {throughput_count}")
    out(f"Total Collisions (Overall): {collisions}")
    out(f"AI-TCC issued {len(token_issue_steps)} token update(s) at steps: {token_issue_steps}")
    out(f"First trigger step (used for PRE/POST split): {PRE_VSL_STEPS}")
    out("=" * 80)

    # Three-phase split: pre / enforcement window / post-clearance
    fog_cleared_step = None
    for row in reversed(decision_log_rows):
        if row[1] == 0.0 and row[0] > PRE_VSL_STEPS:
            fog_cleared_step = row[0]
            break
    fog_cleared_step = fog_cleared_step if fog_cleared_step else step

    df_pre3     = df[df['step'] < PRE_VSL_STEPS]
    df_enforce  = df[(df['step'] >= PRE_VSL_STEPS) & (df['step'] < fog_cleared_step)]
    df_post3    = df[df['step'] >= fog_cleared_step]

    def calculate_phase_metrics3(df_phase, phase_name):
        if df_phase.empty:
            return {'Phase': phase_name, 'Overspeed': 0, 'TTC Fails': 0,
                    'Avg Speed (km/h)': 0.0, 'Speed Std Dev (m/s)': 0.0}
        return {
            'Phase': phase_name,
            'Overspeed': int(df_phase['is_overspeed'].sum()),
            'TTC Fails': int(df_phase['ttc_check_fail'].sum()),
            'Avg Speed (km/h)': round(df_phase['speed_ms'].mean() * 3.6, 2),
            'Speed Std Dev (m/s)': round(df_phase['speed_ms'].std(), 2),
        }

    phases3 = pd.DataFrame([
        calculate_phase_metrics3(df_pre3,    f'Phase 1 – Pre-fog (steps 0–{PRE_VSL_STEPS-1})'),
        calculate_phase_metrics3(df_enforce, f'Phase 2 – Enforcement (steps {PRE_VSL_STEPS}–{fog_cleared_step-1})'),
        calculate_phase_metrics3(df_post3,   f'Phase 3 – Post-clearance (steps {fog_cleared_step}+)'),
    ])

    out("\n--- Three-Phase Compliance, Safety & Flow Comparison ---")
    out(phases3.to_string(index=False))

    out("\n--- SSG-AI VSL Compliance Assessment ---")
    out(f"Total Non-Compliant Drivers Targeted (Types {', '.join(NON_COMPLIANT_VTYPES)}): {NON_COMPLIANT_COUNT}")
    out(f"Unique Non-Compliant Drivers Subjected to Forced SSG-AI Enforcement: {unique_forced_intervened_drivers}")

    if total_attempts > 0:
        out(f"\n--- Communication Failure & Efficiency Metrics ---")
        out(f"1. Total Tokens Sent (Attempts to Enforce, all episodes): {total_attempts}")
        out(f"2. Tokens Lost (Failed to Reach OBU): {loss_count}")
        out(f"3. Tokens Successfully Received (Delayed/Undelayed): {success_count}")
        out(f"4. Actual Packet Loss Rate: {loss_count/total_attempts*100:.2f}%")
        out(f"5. Total Collisions: {collisions}")
        out(f"6. Collision Immunity Index (CII): {CII:.2f}%")

    out("\n" + "=" * 80)
    out(f"Decision log saved to {DECISION_LOG_FILE} - plot fog_severity / congestion_pct / "
        f"active_target_kph over step for a figure showing the AI-TCC's live decisions.")
    out(f"Conclusion: AI-TCC adjusted the enforced speed {len(token_issue_steps)} time(s) in response to "
        f"live fog/congestion signals (not a pre-scripted single drop), reaching a minimum of "
        f"{min(r[5] for r in decision_log_rows):.1f} km/h, with CII={CII:.2f}% across {loss_count} lost tokens.")
    out(f"\nFiles written to: {BASE_DIR}")
    out(f"  - {os.path.basename(LOG_FILE)} (per-vehicle, per-step log)")
    out(f"  - {os.path.basename(DECISION_LOG_FILE)} (per-step AI-TCC decision log)")
    out(f"  - {os.path.basename(SUMMARY_FILE)} (this summary)")

    with open(SUMMARY_FILE, 'w') as f:
        f.write("\n".join(summary_lines))


if __name__ == '__main__':
    # Running with Scenario C parameters (50% loss, 5s max latency).
    # Set PACKET_LOSS_RATE = 0.0 and MAX_LATENCY_STEPS = 0 above to run as Scenario B instead.
    run_scenario_b_c(os.path.join(BASE_DIR, "highway_vsl.sumocfg"))

    # Keep the window open if this was double-clicked rather than run from a
    # terminal, so the summary is readable before the console closes.
    # (Also written to tcc_run_summary.txt regardless, so it's never lost.)
    try:
        if sys.stdin and sys.stdin.isatty():
            input("\nDone. Press Enter to close this window...")
    except Exception:
        pass
