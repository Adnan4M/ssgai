import os
import subprocess

# --- DEFINITIONS ---

# 1. Clean SUMO Configuration (Removes all [source] tags)
config_xml = """<?xml version="1.0" encoding="UTF-8"?>
<configuration>
    <input>
        <net-file value="urban_a_network.net.xml"/>
        <route-files value="urban_a_routes_test1.rou.xml"/>
    </input>
    <time>
        <begin value="0"/>
        <end value="10000"/>
        <step-length value="1"/>
    </time>
    <output>
        <fcd-output value="urban_a_fcd_data_test1.xml"/>
    </output>
</configuration>
"""

# 2. Clean Route File (Defines vehicles and flows)
routes_xml = """<?xml version="1.0" encoding="UTF-8"?>
<routes xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/routes_file.xsd">
    <vType id="normal" accel="3.0" decel="6.0" sigma="0.5" length="5.0" minGap="2.5" maxSpeed="22.22" lcStrategic="0.5"/>
    <vType id="aggressive" accel="4.0" decel="8.0" sigma="0.9" length="5.0" minGap="1.0" maxSpeed="45.0" lcStrategic="0.1" />
    <vType id="veryAggressive" accel="5.0" decel="6.0" sigma="1.0" length="5.0" minGap="1.5" maxSpeed="25.0" lcStrategic="0.05"/>

    <route id="r0" edges="E1 E2 E3 E4"/>

    <flow id="flow_normal" type="normal" route="r0" begin="0" end="500" number="85"/>
    <flow id="flow_aggressive" type="aggressive" route="r0" begin="0" end="500" number="10"/>
    <flow id="flow_very_aggressive" type="veryAggressive" route="r0" begin="0" end="500" number="5"/>
</routes>
"""

# 3. Nodes (Junctions) for Network Generation
nodes_xml = """<?xml version="1.0" encoding="UTF-8"?>
<nodes xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/nodes_file.xsd">
    <node id="J0" x="0.0" y="0.0" type="priority"/>
    <node id="J1" x="1000.0" y="0.0" type="priority"/>
    <node id="J2" x="2500.0" y="0.0" type="priority"/>
    <node id="J3" x="4000.0" y="0.0" type="priority"/>
    <node id="J4" x="5000.0" y="0.0" type="dead_end"/>
</nodes>
"""

# 4. Edges (Road Segments) for Network Generation
edges_xml = """<?xml version="1.0" encoding="UTF-8"?>
<edges xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/edges_file.xsd">
    <edge id="E1" from="J0" to="J1" numLanes="2" speed="22.22" priority="1"/>
    <edge id="E2" from="J1" to="J2" numLanes="2" speed="22.22" priority="1"/>
    <edge id="E3" from="J2" to="J3" numLanes="2" speed="22.22" priority="1"/>
    <edge id="E4" from="J3" to="J4" numLanes="2" speed="22.22" priority="1"/>
</edges>
"""

def create_file(filename, content):
    with open(filename, 'w') as f:
        f.write(content.strip())
    print(f"✔ Created clean file: {filename}")

def main():
    print("--- Initializing SUMO Files for Scenario A ---")
    
    # 1. Create the Raw XML Files
    create_file("urban.nod.xml", nodes_xml)
    create_file("urban.edg.xml", edges_xml)
    create_file("urban_a_routes_test1.rou.xml", routes_xml)
    create_file("urban_a.sumocfg", config_xml)
    
    # 2. Locate netconvert binary
    if "SUMO_HOME" in os.environ:
        netconvert_binary = os.path.join(os.environ["SUMO_HOME"], "bin", "netconvert")
    else:
        netconvert_binary = "netconvert" # Hope it's in PATH

    # 3. Compile the Network
    print("\nRunning netconvert to generate compiled network...")
    cmd = [
        netconvert_binary,
        "--node-files", "urban.nod.xml",
        "--edge-files", "urban.edg.xml",
        "-o", "urban_a_network.net.xml"
    ]
    
    try:
        # Run netconvert (suppress verbose output unless error)
        subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL)
        print("✔ SUCCESS: Network file 'urban_a_network.net.xml' generated.")
        print("\nAll files are ready. You can now run 'scenario_a_urban.py'.")
        
    except subprocess.CalledProcessError:
        print("❌ ERROR: 'netconvert' failed.")
        print("Ensure SUMO is installed correctly and 'netconvert' is in your PATH or SUMO_HOME is set.")
    except FileNotFoundError:
        print("❌ ERROR: 'netconvert' executable not found.")
        print("Please check your SUMO installation.")

if __name__ == "__main__":
    main()