import traci
import csv
import os
import pandas as pd

# --- PARAMETERS FOR SCENARIO A (Urban Street VSL) ---
LOG_FILE = "urban_a_log_test1.csv" 
SUMO_CONFIG_FILE = "urban_a.sumocfg"

# Speed Conversion (km/h to m/s)
KMH_TO_MS = 1 / 3.6

# SCENARIO A PARAMETERS
URBAN_MAX_SPEED_MS = 80 * KMH_TO_MS # 22.22 m/s (The legal speed limit)
URBAN_EDGES = ["E1", "E2", "E3", "E4"] # All segments of the road
SIMULATION_END_TIME = 10000 

# Compliance and Intervention Parameters
NON_COMPLIANT_VTYPES = {"veryAggressive"} 
MIN_TTC = 1.5                       
MIN_HEADWAY = 2.0                   

# --- GLOBAL TRACKING ---
GLOBAL_NON_COMPLIANT_DRIVERS = set()
TOTAL_NON_COMPLIANT_TARGETS = 5 

def save_log_header():
    """Saves the header for the main data log."""
    with open(LOG_FILE, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            "step", "vehicle_id", "vType", "speed_ms", "max_allowed_ms", "is_overspeed", 
            "ttc_check_fail", "decel_intent_sent", "is_non_compliant"
        ])

def log_data(data):
    """Appends simulation data to the log file."""
    with open(LOG_FILE, 'a', newline='') as f:
        writer = csv.writer(f)
        writer.writerows(data)

def run_scenario_a(sumo_cfg):
    """Runs Scenario A: Urban Street VSL enforcement."""
    
    # 1. Locate SUMO Binary
    if "SUMO_HOME" in os.environ:
        sumo_binary = os.path.join(os.environ["SUMO_HOME"], "bin", "sumo")
    else:
        sumo_binary = "sumo" 

    # 2. Check if Config File Exists
    if not os.path.exists(sumo_cfg):
        print(f"FATAL ERROR: Configuration file '{sumo_cfg}' not found in {os.getcwd()}")
        return

    print(f"Attempting to start SUMO using config: {sumo_cfg}")

    # 3. Start SUMO with Logging Enabled
    # We add '--log' to capture any startup errors to a file
    cmd = [
        sumo_binary, 
        "-c", sumo_cfg, 
        "--step-length", "1.0",
        "--log", "sumo_error.log" 
    ]

    try:
        traci.start(cmd)
    except traci.exceptions.FatalTraCIError:
        print("\nFATAL ERROR: SUMO crashed during startup.")
        print("Check 'sumo_error.log' for details. Common causes:")
        print("1. Invalid XML in .sumocfg or .rou.xml files.")
        print("2. Mismatched file names.")
        return
    except Exception as e:
        print(f"FATAL ERROR: Could not start TraCI server. Error: {e}")
        return

    # Set the actual urban speed limit on all relevant edges
    for edge in URBAN_EDGES:
        try:
            traci.edge.setMaxSpeed(edge, URBAN_MAX_SPEED_MS)
        except traci.exceptions.TraCIException:
            print(f"Warning: Could not set speed for edge {edge}. Is the network loaded?")

    save_log_header()
    
    data_to_log = []
    collisions = 0
    throughput_count = 0
    overspeed_events = 0
    interventions = 0
    step = 0
    
    print(f"--- SCENARIO A: URBAN VSL (80 km/h Limit) - Test 1 ---")

    try:
        while traci.simulation.getMinExpectedNumber() > 0 and step < SIMULATION_END_TIME:
            traci.simulationStep()
            step_log = []

            for vid in traci.vehicle.getIDList():
                v_type = traci.vehicle.getTypeID(vid)
                
                # Identify Target Drivers
                if vid not in GLOBAL_NON_COMPLIANT_DRIVERS and v_type in NON_COMPLIANT_VTYPES:
                    GLOBAL_NON_COMPLIANT_DRIVERS.add(vid)
                is_non_compliant = vid in GLOBAL_NON_COMPLIANT_DRIVERS

                v_speed = traci.vehicle.getSpeed(vid)
                v_max_allowed = URBAN_MAX_SPEED_MS
                
                is_overspeed = v_speed > v_max_allowed
                if is_overspeed: overspeed_events += 1
                
                ttc_fail = 0  
                decel_intent = 0
                
                # --- OBU ENFORCEMENT & SAFETY LOGIC ---
                if is_non_compliant and is_overspeed:
                    leader_data = traci.vehicle.getLeader(vid)
                    safe_to_intervene = True
                    
                    if leader_data:
                        leader_id, gap = leader_data
                        if gap is not None and leader_id: 
                             # Calculate TTC and Headway
                            leader_speed = traci.vehicle.getSpeed(leader_id)
                            relative_speed = v_speed - leader_speed
                            
                            if relative_speed > 0: 
                                ttc = gap / relative_speed
                                if ttc < MIN_TTC:
                                    safe_to_intervene = False
                                    ttc_fail = 1 
                            
                            if v_speed > 0 and (gap / v_speed < MIN_HEADWAY):
                                safe_to_intervene = False

                    if safe_to_intervene:
                        traci.vehicle.setSpeed(vid, v_max_allowed) 
                        decel_intent = 1
                        interventions += 1 

                step_log.append([step, vid, v_type, v_speed, v_max_allowed, int(is_overspeed), ttc_fail, decel_intent, int(is_non_compliant)])

            data_to_log.extend(step_log)
            throughput_count += len(traci.simulation.getArrivedIDList())
            collisions += len(traci.simulation.getCollisions())
            step += 1
            
        print(f"\n--- SIMULATION FINISHED AT STEP {step} ---")

    except Exception as e:
        print(f"\nError during simulation step {step}: {e}")
    finally:
        traci.close()
    
    # --- RESULTS ANALYSIS ---
    log_data(data_to_log) 
    
    if not os.path.exists(LOG_FILE):
        print("Error: Log file was not created.")
        return

    try:
        df = pd.read_csv(LOG_FILE)
        if df.empty:
            print("Log file is empty.")
            return

        df_agg = df[df['vType'].isin(NON_COMPLIANT_VTYPES)]
        overspeed_agg = df_agg['is_overspeed'].sum()
        interventions_agg = df_agg['decel_intent_sent'].sum()
        ttc_fails_agg = df_agg['ttc_check_fail'].sum()
        avg_speed = df['speed_ms'].mean() * 3.6

        print("\n" + "="*80)
        print(f"=== RESULTS SUMMARY ===")
        print(f"Throughput: {throughput_count} vehicles")
        print(f"Collisions: {collisions}")
        print(f"Avg Speed: {avg_speed:.2f} km/h")
        print(f"Aggressive Overspeed Events: {overspeed_agg}")
        print(f"Interventions Triggered: {interventions_agg}")
        print(f"Interventions Blocked (Safety): {ttc_fails_agg}")
        print("="*80)

    except Exception as e:
        print(f"Error analyzing results: {e}")

if __name__ == '__main__':
    run_scenario_a(SUMO_CONFIG_FILE)