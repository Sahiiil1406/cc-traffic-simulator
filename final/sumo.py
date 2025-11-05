#!/usr/bin/env python3
"""
SUMO Ring Road Simulation with Lane Changing Around Obstacles
Based on experimental site diagram with inner/outer ring configuration
15+ vehicles of each type, 3-4 minute simulation
FIXED VERSION - All errors resolved
"""

import os
import sys
import subprocess

def create_network_file():
    """Create the road network XML file matching the diagram specifications"""
    
    print("🔧 Generating network using netconvert...")
    
    # Create nodes file
    nodes_xml = '''<?xml version="1.0" encoding="UTF-8"?>
<nodes>
    <node id="junction_bl" x="66.00" y="60.00" type="priority"/>
    <node id="junction_br" x="146.00" y="60.00" type="priority"/>
    <node id="junction_tl" x="66.00" y="140.00" type="priority"/>
    <node id="junction_tr" x="146.00" y="140.00" type="priority"/>
</nodes>'''
    
    with open('ring_road.nod.xml', 'w') as f:
        f.write(nodes_xml)
    
    # Create edges file
    edges_xml = '''<?xml version="1.0" encoding="UTF-8"?>
<edges>
    <edge id="bottom_straight" from="junction_br" to="junction_bl" numLanes="2" speed="13.89" priority="1"/>
    <edge id="left_curve" from="junction_bl" to="junction_tl" numLanes="2" speed="11.11" priority="1"/>
    <edge id="top_straight" from="junction_tl" to="junction_tr" numLanes="2" speed="13.89" priority="1"/>
    <edge id="right_curve" from="junction_tr" to="junction_br" numLanes="2" speed="11.11" priority="1"/>
</edges>'''
    
    with open('ring_road.edg.xml', 'w') as f:
        f.write(edges_xml)
    
    # Create connection file to ensure proper lane connections
    connections_xml = '''<?xml version="1.0" encoding="UTF-8"?>
<connections>
    <connection from="bottom_straight" to="left_curve" fromLane="0" toLane="0"/>
    <connection from="bottom_straight" to="left_curve" fromLane="1" toLane="1"/>
    <connection from="left_curve" to="top_straight" fromLane="0" toLane="0"/>
    <connection from="left_curve" to="top_straight" fromLane="1" toLane="1"/>
    <connection from="top_straight" to="right_curve" fromLane="0" toLane="0"/>
    <connection from="top_straight" to="right_curve" fromLane="1" toLane="1"/>
    <connection from="right_curve" to="bottom_straight" fromLane="0" toLane="0"/>
    <connection from="right_curve" to="bottom_straight" fromLane="1" toLane="1"/>
</connections>'''
    
    with open('ring_road.con.xml', 'w') as f:
        f.write(connections_xml)
    
    # Run netconvert to generate the network
    if 'SUMO_HOME' in os.environ:
        netconvert_binary = os.path.join(os.environ['SUMO_HOME'], 'bin', 'netconvert')
        if os.name == 'nt' and not netconvert_binary.endswith('.exe'):
            netconvert_binary += '.exe'
        
        try:
            result = subprocess.run([
                netconvert_binary,
                '--node-files=ring_road.nod.xml',
                '--edge-files=ring_road.edg.xml',
                '--connection-files=ring_road.con.xml',
                '--output-file=ring_road.net.xml',
                '--no-turnarounds=true',
                '--junctions.corner-detail=5',
                '--default.lanewidth=4.0'
            ], check=True, capture_output=True, text=True)
            
            print("✓ Network file created: ring_road.net.xml")
            return True
            
        except subprocess.CalledProcessError as e:
            print(f"❌ Error running netconvert:")
            print(e.stderr)
            return False
        except FileNotFoundError:
            print(f"❌ Error: netconvert not found at {netconvert_binary}")
            return False
    else:
        print("❌ Error: SUMO_HOME not set")
        return False


def create_route_file():
    """Create routes for 15+ inner (yellow) and 15+ outer (red) lane vehicles"""
    
    route_xml = '''<?xml version="1.0" encoding="UTF-8"?>
<routes xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noSchemaLocation="http://sumo.dlr.de/xsd/routes_file.xsd">

    <!-- Vehicle types -->
    <vType id="yellow_car" vClass="passenger" color="1,1,0" length="4.5" minGap="2.5" maxSpeed="15.0" 
           accel="2.5" decel="4.5" sigma="0.5" lcStrategic="1.0" lcCooperative="1.0" lcSpeedGain="2.0" 
           lcKeepRight="0" lcAssertive="1.0" lcLookaheadLeft="3.0" lcSpeedGainRight="0.5"/>
    
    <vType id="red_car" vClass="passenger" color="1,0,0" length="4.5" minGap="2.5" maxSpeed="14.0" 
           accel="2.0" decel="4.5" sigma="0.5" lcStrategic="0.5" lcCooperative="1.0" lcSpeedGain="1.0" 
           lcKeepRight="0" lcAssertive="0.5" lcLookaheadLeft="3.0" lcSpeedGainRight="0.5"/>

    <!-- Route definition: complete loop -->
    <route id="loop_route" edges="bottom_straight left_curve top_straight right_curve"/>

    <!-- Yellow cars (inner lane) - 20 vehicles -->
    <vehicle id="yellow_01" type="yellow_car" route="loop_route" depart="0.0" departLane="0" departPos="0" departSpeed="max"/>
    <vehicle id="yellow_02" type="yellow_car" route="loop_route" depart="2.0" departLane="0" departPos="15" departSpeed="max"/>
    <vehicle id="yellow_03" type="yellow_car" route="loop_route" depart="4.0" departLane="0" departPos="30" departSpeed="max"/>
    <vehicle id="yellow_04" type="yellow_car" route="loop_route" depart="6.0" departLane="0" departPos="45" departSpeed="max"/>
    <vehicle id="yellow_05" type="yellow_car" route="loop_route" depart="8.0" departLane="0" departPos="60" departSpeed="max"/>
    <vehicle id="yellow_06" type="yellow_car" route="loop_route" depart="10.0" departLane="0" departPos="5" departSpeed="max"/>
    <vehicle id="yellow_07" type="yellow_car" route="loop_route" depart="12.0" departLane="0" departPos="20" departSpeed="max"/>
    <vehicle id="yellow_08" type="yellow_car" route="loop_route" depart="14.0" departLane="0" departPos="35" departSpeed="max"/>
    <vehicle id="yellow_09" type="yellow_car" route="loop_route" depart="16.0" departLane="0" departPos="50" departSpeed="max"/>
    <vehicle id="yellow_10" type="yellow_car" route="loop_route" depart="18.0" departLane="0" departPos="65" departSpeed="max"/>
    <vehicle id="yellow_11" type="yellow_car" route="loop_route" depart="20.0" departLane="0" departPos="10" departSpeed="max"/>
    <vehicle id="yellow_12" type="yellow_car" route="loop_route" depart="22.0" departLane="0" departPos="25" departSpeed="max"/>
    <vehicle id="yellow_13" type="yellow_car" route="loop_route" depart="24.0" departLane="0" departPos="40" departSpeed="max"/>
    <vehicle id="yellow_14" type="yellow_car" route="loop_route" depart="26.0" departLane="0" departPos="55" departSpeed="max"/>
    <vehicle id="yellow_15" type="yellow_car" route="loop_route" depart="28.0" departLane="0" departPos="70" departSpeed="max"/>
    <vehicle id="yellow_16" type="yellow_car" route="loop_route" depart="30.0" departLane="0" departPos="12" departSpeed="max"/>
    <vehicle id="yellow_17" type="yellow_car" route="loop_route" depart="32.0" departLane="0" departPos="27" departSpeed="max"/>
    <vehicle id="yellow_18" type="yellow_car" route="loop_route" depart="34.0" departLane="0" departPos="42" departSpeed="max"/>
    <vehicle id="yellow_19" type="yellow_car" route="loop_route" depart="36.0" departLane="0" departPos="57" departSpeed="max"/>
    <vehicle id="yellow_20" type="yellow_car" route="loop_route" depart="38.0" departLane="0" departPos="72" departSpeed="max"/>

    <!-- Red cars (outer lane) - 20 vehicles -->
    <vehicle id="red_01" type="red_car" route="loop_route" depart="1.0" departLane="1" departPos="7" departSpeed="max"/>
    <vehicle id="red_02" type="red_car" route="loop_route" depart="3.0" departLane="1" departPos="22" departSpeed="max"/>
    <vehicle id="red_03" type="red_car" route="loop_route" depart="5.0" departLane="1" departPos="37" departSpeed="max"/>
    <vehicle id="red_04" type="red_car" route="loop_route" depart="7.0" departLane="1" departPos="52" departSpeed="max"/>
    <vehicle id="red_05" type="red_car" route="loop_route" depart="9.0" departLane="1" departPos="67" departSpeed="max"/>
    <vehicle id="red_06" type="red_car" route="loop_route" depart="11.0" departLane="1" departPos="2" departSpeed="max"/>
    <vehicle id="red_07" type="red_car" route="loop_route" depart="13.0" departLane="1" departPos="17" departSpeed="max"/>
    <vehicle id="red_08" type="red_car" route="loop_route" depart="15.0" departLane="1" departPos="32" departSpeed="max"/>
    <vehicle id="red_09" type="red_car" route="loop_route" depart="17.0" departLane="1" departPos="47" departSpeed="max"/>
    <vehicle id="red_10" type="red_car" route="loop_route" depart="19.0" departLane="1" departPos="62" departSpeed="max"/>
    <vehicle id="red_11" type="red_car" route="loop_route" depart="21.0" departLane="1" departPos="4" departSpeed="max"/>
    <vehicle id="red_12" type="red_car" route="loop_route" depart="23.0" departLane="1" departPos="19" departSpeed="max"/>
    <vehicle id="red_13" type="red_car" route="loop_route" depart="25.0" departLane="1" departPos="34" departSpeed="max"/>
    <vehicle id="red_14" type="red_car" route="loop_route" depart="27.0" departLane="1" departPos="49" departSpeed="max"/>
    <vehicle id="red_15" type="red_car" route="loop_route" depart="29.0" departLane="1" departPos="64" departSpeed="max"/>
    <vehicle id="red_16" type="red_car" route="loop_route" depart="31.0" departLane="1" departPos="9" departSpeed="max"/>
    <vehicle id="red_17" type="red_car" route="loop_route" depart="33.0" departLane="1" departPos="24" departSpeed="max"/>
    <vehicle id="red_18" type="red_car" route="loop_route" depart="35.0" departLane="1" departPos="39" departSpeed="max"/>
    <vehicle id="red_19" type="red_car" route="loop_route" depart="37.0" departLane="1" departPos="54" departSpeed="max"/>
    <vehicle id="red_20" type="red_car" route="loop_route" depart="39.0" departLane="1" departPos="69" departSpeed="max"/>

    <!-- Additional flow for continuous traffic during 3-4 minutes -->
    <flow id="yellow_flow" type="yellow_car" route="loop_route" begin="45" end="240" 
          number="10" departLane="0" departSpeed="max" departPos="random"/>

    <flow id="red_flow" type="red_car" route="loop_route" begin="45" end="240" 
          number="10" departLane="1" departSpeed="max" departPos="random"/>

</routes>'''
    
    with open('ring_road.rou.xml', 'w') as f:
        f.write(route_xml)
    
    print("✓ Route file created: ring_road.rou.xml")
    print("  - 20 yellow cars (inner lane)")
    print("  - 20 red cars (outer lane)")
    print("  - Additional flows for continuous traffic")


def create_additional_file():
    """Create additional file with obstacles (polygons) and detectors"""
    
    # Note: Detector positions are set to safe values within lane length
    additional_xml = '''<?xml version="1.0" encoding="UTF-8"?>
<additional xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noSchemaLocation="http://sumo.dlr.de/xsd/additional_file.xsd">

    <!-- Main obstacle on bottom straight (forces lane change) -->
    <poly id="obstacle_main" type="obstacle" color="0,0,0" fill="1" layer="2" 
          shape="115.00,54.00 120.00,54.00 120.00,57.50 115.00,57.50"/>
    
    <!-- Bottleneck area markers (beige/tan color) -->
    <poly id="bottleneck_1" type="bottleneck" color="0.96,0.87,0.70" fill="1" layer="1" 
          shape="95.00,52.00 100.00,52.00 100.00,60.00 95.00,60.00"/>
    
    <poly id="bottleneck_2" type="bottleneck" color="0.96,0.87,0.70" fill="1" layer="1" 
          shape="125.00,52.00 130.00,52.00 130.00,60.00 125.00,60.00"/>

    <!-- Visual markers for allowed lane change area -->
    <poly id="lane_change_area" type="area" color="0.9,0.95,0.9" fill="0" layer="0"
          shape="66.00,52.00 146.00,52.00 146.00,60.00 66.00,60.00"/>

    <!-- Loop detectors for traffic monitoring (positions within safe range) -->
    <inductionLoop id="detector_bottom_inner" lane="bottom_straight_0" pos="40" freq="30" file="detector_output.xml"/>
    <inductionLoop id="detector_bottom_outer" lane="bottom_straight_1" pos="40" freq="30" file="detector_output.xml"/>
    <inductionLoop id="detector_top_inner" lane="top_straight_0" pos="40" freq="30" file="detector_output.xml"/>
    <inductionLoop id="detector_top_outer" lane="top_straight_1" pos="40" freq="30" file="detector_output.xml"/>
    
    <!-- Pre-obstacle detector (safe position) -->
    <inductionLoop id="detector_pre_obstacle_inner" lane="bottom_straight_0" pos="60" freq="10" file="detector_output.xml"/>
    <inductionLoop id="detector_pre_obstacle_outer" lane="bottom_straight_1" pos="60" freq="10" file="detector_output.xml"/>

    <!-- Lane area detectors for bottleneck monitoring -->
    <laneAreaDetector id="bottleneck_detector_1" lane="bottom_straight_0" pos="30" endPos="35" freq="30" file="detector_output.xml"/>
    <laneAreaDetector id="bottleneck_detector_2" lane="bottom_straight_1" pos="50" endPos="55" freq="30" file="detector_output.xml"/>

</additional>'''
    
    with open('ring_road.add.xml', 'w') as f:
        f.write(additional_xml)
    
    print("✓ Additional file created: ring_road.add.xml")


def create_config_file():
    """Create SUMO configuration file for 3-4 minute simulation"""
    
    config_xml = '''<?xml version="1.0" encoding="UTF-8"?>
<configuration xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noSchemaLocation="http://sumo.dlr.de/xsd/sumoConfiguration.xsd">

    <input>
        <net-file value="ring_road.net.xml"/>
        <route-files value="ring_road.rou.xml"/>
        <additional-files value="ring_road.add.xml"/>
    </input>

    <time>
        <begin value="0"/>
        <end value="240"/>
        <step-length value="0.1"/>
    </time>

    <processing>
        <lateral-resolution value="0.8"/>
        <collision.action value="warn"/>
        <collision.check-junctions value="true"/>
        <collision.mingap-factor value="0.1"/>
        <time-to-teleport value="-1"/>
        <max-depart-delay value="0"/>
    </processing>

    <routing>
        <device.rerouting.probability value="0"/>
    </routing>

    <report>
        <verbose value="true"/>
        <no-step-log value="true"/>
        <duration-log.statistics value="true"/>
    </report>

    <gui_only>
        <gui-settings-file value="ring_road.settings.xml"/>
        <start value="true"/>
        <quit-on-end value="false"/>
        <delay value="20"/>
    </gui_only>

</configuration>'''
    
    with open('ring_road.sumocfg', 'w') as f:
        f.write(config_xml)
    
    print("✓ Configuration file created: ring_road.sumocfg")
    print("  - Simulation duration: 240 seconds (4 minutes)")


def create_gui_settings():
    """Create GUI settings for better visualization"""
    
    settings_xml = '''<?xml version="1.0" encoding="UTF-8"?>
<viewsettings>
    <viewport zoom="380" x="106" y="100" angle="0"/>
    <delay value="20"/>
    <scheme name="real world"/>
    <background backgroundColor="220,220,220" showGrid="0"/>
</viewsettings>'''
    
    with open('ring_road.settings.xml', 'w') as f:
        f.write(settings_xml)
    
    print("✓ GUI settings file created: ring_road.settings.xml")


def print_instructions():
    """Print detailed instructions for running the simulation"""
    
    print("\n" + "="*70)
    print(" SUMO RING ROAD SIMULATION - READY TO RUN")
    print("="*70)
    print("\n📋 SIMULATION DETAILS:")
    print("   • Duration: 4 minutes (240 seconds)")
    print("   • Yellow vehicles: 20+ (inner lane)")
    print("   • Red vehicles: 20+ (outer lane)")
    print("   • Black obstacle forcing lane changes")
    print("   • Beige bottleneck areas")
    print("   • Green loop detectors")
    print("\n🎮 GUI CONTROLS:")
    print("   • SPACE: Play/Pause simulation")
    print("   • +/-: Speed up/slow down")
    print("   • Left mouse: Pan view")
    print("   • Right mouse: Zoom")
    print("   • Ctrl+A: Fit view to network")
    print("\n💡 WHAT TO OBSERVE:")
    print("   • Yellow cars (inner lane) encountering black obstacle")
    print("   • Lane change behavior to avoid obstacle")
    print("   • Red cars (outer lane) maintaining their lane")
    print("   • Traffic flow through bottleneck areas")
    print("   • Vehicle interactions and spacing")
    print("\n" + "="*70)


def run_simulation():
    """Run the SUMO simulation with GUI"""
    
    # Check if SUMO is installed
    if 'SUMO_HOME' not in os.environ:
        print("\n❌ ERROR: SUMO_HOME environment variable not set!")
        print("\n📥 INSTALLATION INSTRUCTIONS:")
        print("   1. Download SUMO from: https://sumo.dlr.de/docs/Downloads.php")
        print("   2. Install SUMO")
        print("   3. Set SUMO_HOME environment variable:")
        print("      - Windows: setx SUMO_HOME \"C:\\Program Files (x86)\\Eclipse\\Sumo\"")
        print("      - Linux/Mac: export SUMO_HOME=/usr/share/sumo")
        print("   4. Re-run this script")
        return False
    
    sumo_binary = os.path.join(os.environ['SUMO_HOME'], 'bin', 'sumo-gui')
    
    # Windows executable check
    if os.name == 'nt' and not sumo_binary.endswith('.exe'):
        sumo_binary += '.exe'
    
    # Check if sumo-gui exists
    if not os.path.exists(sumo_binary):
        print(f"\n❌ ERROR: sumo-gui not found at: {sumo_binary}")
        print("\nPlease ensure SUMO is properly installed.")
        return False
    
    print("\n🚀 Starting SUMO GUI...")
    print("⏱️  Please wait for the simulation window to open...\n")
    
    # Run SUMO GUI
    try:
        result = subprocess.run([sumo_binary, '-c', 'ring_road.sumocfg'], 
                              capture_output=False, 
                              check=False)
        
        if result.returncode == 0:
            print("\n✅ Simulation completed successfully!")
        else:
            print(f"\n⚠️  Simulation ended with return code: {result.returncode}")
        
        return True
        
    except FileNotFoundError:
        print(f"\n❌ ERROR: Could not execute sumo-gui")
        print(f"   Path tried: {sumo_binary}")
        return False
    except Exception as e:
        print(f"\n❌ ERROR: Unexpected error during simulation: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """Main function to set up and run the simulation"""
    
    print("\n" + "="*70)
    print(" SUMO RING ROAD SIMULATION - SETUP")
    print("="*70 + "\n")
    
    try:
        # Create all necessary files
        print("📁 Creating simulation files...\n")
        
        if not create_network_file():
            print("\n❌ Failed to create network file. Cannot continue.")
            return 1
            
        create_route_file()
        create_additional_file()
        create_config_file()
        create_gui_settings()
        
        print("\n✅ All simulation files created successfully!")
        
        # Print instructions
        print_instructions()
        
        # Run the simulation
        input("\n▶️  Press ENTER to start the simulation...")
        
        if not run_simulation():
            print("\n⚠️  Simulation failed to start. Please check the errors above.")
            return 1
        
        print("\n✅ Simulation session ended.")
        print("📊 Check 'detector_output.xml' for traffic statistics.")
        
    except KeyboardInterrupt:
        print("\n\n⚠️  Setup interrupted by user.")
        return 1
    except Exception as e:
        print(f"\n❌ ERROR during setup: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    return 0


if __name__ == "__main__":
    sys.exit(main())