import os
import subprocess
import math

def create_network_file():
    """Create the circular two-lane network based on the paper specifications"""
    
    # Paper specs: Inner ring = 411.2m (r=40m), Outer ring = 461.4m (r=48m)
    # Lane width = 4m, Vehicle length = 5m, Obstacle length = 5m, Lane change area = 80m
    
    num_nodes = 16
    inner_radius = 40.0
    outer_radius = 48.0
    
    nodes_xml = '<?xml version="1.0" encoding="UTF-8"?>\n'
    nodes_xml += '<nodes xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/nodes_file.xsd">\n'
    
    # Create circular nodes
    for i in range(num_nodes):
        angle = 2 * math.pi * i / num_nodes
        x = inner_radius + (inner_radius + outer_radius) / 2 * math.cos(angle)
        y = inner_radius + (inner_radius + outer_radius) / 2 * math.sin(angle)
        nodes_xml += f'    <node id="n{i}" x="{x:.2f}" y="{y:.2f}" type="priority"/>\n'
    
    nodes_xml += '</nodes>'
    
    with open('mandatory_lc.nod.xml', 'w') as f:
        f.write(nodes_xml)
    
    # Create edges with bottleneck area
    edges_xml = '<?xml version="1.0" encoding="UTF-8"?>\n'
    edges_xml += '<edges xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/edges_file.xsd">\n'
    
    # Calculate bottleneck position (top of the circle, around nodes 4-7)
    bottleneck_start = 4
    bottleneck_end = 7
    
    for i in range(num_nodes):
        next_i = (i + 1) % num_nodes
        
        # Check if this edge is in the bottleneck area
        if bottleneck_start <= i < bottleneck_end:
            # Bottleneck: outer lane blocked (obstacle area)
            edges_xml += f'    <edge id="e{i}" from="n{i}" to="n{next_i}" numLanes="2" speed="16.67" width="4.0">\n'
            edges_xml += f'        <lane index="0" allow="all" width="4.0"/>\n'
            edges_xml += f'        <lane index="1" disallow="all" width="4.0"/>\n'  # Outer lane blocked
            edges_xml += f'    </edge>\n'
        else:
            # Normal two-lane section
            edges_xml += f'    <edge id="e{i}" from="n{i}" to="n{next_i}" numLanes="2" speed="16.67" width="4.0"/>\n'
    
    edges_xml += '</edges>'
    
    with open('mandatory_lc.edg.xml', 'w') as f:
        f.write(edges_xml)
    
    # Generate network
    print("Generating SUMO network based on paper specifications...")
    print("Inner ring: 411.2m (radius 40m)")
    print("Outer ring: 461.4m (radius 48m)")
    print("Lane width: 4m")
    print("Bottleneck: 5m obstacle causing lane closure")
    
    result = subprocess.run([
        'netconvert',
        '--node-files=mandatory_lc.nod.xml',
        '--edge-files=mandatory_lc.edg.xml',
        '--output-file=mandatory_lc.net.xml',
        '--no-turnarounds',
        '--junctions.corner-detail=5',
        '--default.lanewidth=4.0'
    ], capture_output=True, text=True)
    
    if result.returncode != 0:
        print("ERROR generating network:")
        print(result.stderr)
        return False
    
    print("✓ Network created successfully!")
    return True

def create_route_file(num_inner=15, num_outer=15):
    """
    Create route file with vehicles in inner and outer lanes
    Paper setup: 30 vehicles total (15 inner, 15 outer)
    Creates continuous traffic flow with vehicles re-entering
    """
    
    routes_xml = '<?xml version="1.0" encoding="UTF-8"?>\n'
    routes_xml += '<routes xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/routes_file.xsd">\n'
    
    # Vehicle type matching paper specs
    routes_xml += '    <vType id="car_inner" accel="4.0" decel="4.0" sigma="0.5" length="5" minGap="2.5" maxSpeed="16.67" guiShape="passenger" color="1,1,0"/>\n'
    routes_xml += '    <vType id="car_outer" accel="4.0" decel="4.0" sigma="0.5" length="5" minGap="2.5" maxSpeed="16.67" guiShape="passenger" color="1,0,0"/>\n'
    
    # Create circular route
    num_edges = 16
    edge_list = ' '.join([f'e{i}' for i in range(num_edges)])
    routes_xml += f'    <route id="circular_route" edges="{edge_list}"/>\n'
    
    # Create flows instead of individual vehicles for continuous traffic
    routes_xml += f'    <flow id="inner_flow" type="car_inner" route="circular_route" begin="0" end="3600" number="{num_inner}" departLane="0" departSpeed="random" color="1,1,0"/>\n'
    routes_xml += f'    <flow id="outer_flow" type="car_outer" route="circular_route" begin="0" end="3600" number="{num_outer}" departLane="1" departSpeed="random" color="1,0,0"/>\n'
    
    routes_xml += '</routes>'
    
    with open('mandatory_lc.rou.xml', 'w') as f:
        f.write(routes_xml)
    
    print(f"✓ Route file created with {num_inner} inner (yellow) + {num_outer} outer (red) vehicles in continuous flow!")

def create_sumo_config():
    """Create SUMO configuration file matching paper simulation setup"""
    
    config_xml = '<?xml version="1.0" encoding="UTF-8"?>\n'
    config_xml += '<configuration xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/sumoConfiguration.xsd">\n'
    config_xml += '    <input>\n'
    config_xml += '        <net-file value="mandatory_lc.net.xml"/>\n'
    config_xml += '        <route-files value="mandatory_lc.rou.xml"/>\n'
    config_xml += '    </input>\n'
    config_xml += '    <time>\n'
    config_xml += '        <begin value="0"/>\n'
    config_xml += '        <end value="3600"/>\n'  # 1 hour simulation (3600 seconds)
    config_xml += '        <step-length value="0.1"/>\n'  # 0.1s timestep as in paper
    config_xml += '    </time>\n'
    config_xml += '    <processing>\n'
    config_xml += '        <collision.action value="warn"/>\n'
    config_xml += '        <collision.check-junctions value="true"/>\n'
    config_xml += '        <time-to-teleport value="-1"/>\n'
    config_xml += '        <lanechange.duration value="3.0"/>\n'  # Realistic lane change duration
    config_xml += '    </processing>\n'
    config_xml += '    <report>\n'
    config_xml += '        <verbose value="true"/>\n'
    config_xml += '        <no-step-log value="true"/>\n'
    config_xml += '    </report>\n'
    config_xml += '    <gui_only>\n'
    config_xml += '        <gui-settings-file value="viewsettings.xml"/>\n'
    config_xml += '    </gui_only>\n'
    config_xml += '</configuration>'
    
    with open('mandatory_lc.sumocfg', 'w') as f:
        f.write(config_xml)
    
    print("✓ SUMO configuration file created!")

def create_view_settings():
    """Create GUI view settings for better visualization"""
    
    view_xml = '''<?xml version="1.0" encoding="UTF-8"?>
<viewsettings>
    <scheme name="real world">
        <background backgroundColor="255,255,255" showGrid="0"/>
    </scheme>
    <delay value="100"/>
    <viewport zoom="200" x="50" y="50"/>
</viewsettings>'''
    
    with open('viewsettings.xml', 'w') as f:
        f.write(view_xml)

def create_additional_file():
    """Create additional file for loop detectors as in the paper"""
    
    add_xml = '<?xml version="1.0" encoding="UTF-8"?>\n'
    add_xml += '<additional xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/additional_file.xsd">\n'
    
    # Add loop detectors at bottleneck area (as in paper: loops 1,2,3,4)
    bottleneck_edge = "e4"
    add_xml += f'    <inductionLoop id="loop1" lane="{bottleneck_edge}_0" pos="0" freq="60" file="loop1.xml"/>\n'
    add_xml += f'    <inductionLoop id="loop2" lane="{bottleneck_edge}_0" pos="15" freq="60" file="loop2.xml"/>\n'
    add_xml += f'    <inductionLoop id="loop3" lane="{bottleneck_edge}_0" pos="30" freq="60" file="loop3.xml"/>\n'
    
    add_xml += '</additional>'
    
    with open('mandatory_lc.add.xml', 'w') as f:
        f.write(add_xml)
    
    print("✓ Loop detectors added for traffic measurement!")

def run_simulation():
    """Run the SUMO simulation with GUI"""
    print("\nStarting SUMO-GUI simulation...")
    print("=" * 60)
    print("SIMULATION OVERVIEW:")
    print("- Yellow vehicles: Inner lane (can stay in lane)")
    print("- Red vehicles: Outer lane (MUST change to inner lane at bottleneck)")
    print("- Black area: Obstacle/bottleneck (outer lane blocked)")
    print("- Watch for mandatory lane changes and traffic congestion!")
    print("=" * 60)
    
    subprocess.run(['sumo-gui', '-c', 'mandatory_lc.sumocfg'])

if __name__ == '__main__':
    print("=" * 70)
    print("MANDATORY LANE CHANGE SIMULATION")
    print("Based on: Wang et al. (2024) - Transportation Research Part C")
    print("=" * 70)
    print("\nPaper Setup:")
    print("- Two-lane circular road (inner: 411.2m, outer: 461.4m)")
    print("- 30 human-driven vehicles (15 inner, 15 outer)")
    print("- 5m obstacle creating mandatory lane change scenario")
    print("- 80m allowed lane change area")
    print("- Lane width: 4m, Vehicle length: 5m")
    print("=" * 70)
    print()
    
    # Create all necessary files
    if not create_network_file():
        print("\n❌ Failed to create network. Please check that netconvert is installed.")
        exit(1)
    
    create_route_file(num_inner=15, num_outer=15)
    create_sumo_config()
    create_view_settings()
    create_additional_file()
    
    print("\n" + "=" * 70)
    print("✓ All files created successfully!")
    print("\nFiles created:")
    print("  - mandatory_lc.nod.xml (nodes)")
    print("  - mandatory_lc.edg.xml (edges with bottleneck)")
    print("  - mandatory_lc.net.xml (network)")
    print("  - mandatory_lc.rou.xml (routes with 30 vehicles)")
    print("  - mandatory_lc.sumocfg (configuration)")
    print("  - mandatory_lc.add.xml (loop detectors)")
    print("  - viewsettings.xml (GUI settings)")
    print("=" * 70)
    
    # Check if network file exists
    if not os.path.exists('mandatory_lc.net.xml'):
        print("\n❌ ERROR: Network file was not created!")
        exit(1)
    
    # Run simulation
    response = input("\nRun simulation now? (y/n): ")
    if response.lower() == 'y':
        run_simulation()
    else:
        print("\nTo run later, execute: sumo-gui -c mandatory_lc.sumocfg")