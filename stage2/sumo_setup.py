import os
import subprocess
import math

def create_network_file():
    """Create circular two-lane network with one blocked section on the outer lane causing lane change."""

    num_nodes = 16
    inner_radius = 40.0
    outer_radius = 48.0
    center_x, center_y = 44.0, 44.0

    # Create nodes
    nodes_xml = '<?xml version="1.0" encoding="UTF-8"?>\n'
    nodes_xml += '<nodes xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/nodes_file.xsd">\n'

    for i in range(num_nodes):
        angle = 2 * math.pi * i / num_nodes
        nodes_xml += f'    <node id="inner_{i}" x="{center_x + inner_radius * math.cos(angle):.2f}" y="{center_y + inner_radius * math.sin(angle):.2f}" type="priority"/>\n'
        nodes_xml += f'    <node id="outer_{i}" x="{center_x + outer_radius * math.cos(angle):.2f}" y="{center_y + outer_radius * math.sin(angle):.2f}" type="priority"/>\n'

    nodes_xml += '</nodes>'

    with open('bottleneck.nod.xml', 'w') as f:
        f.write(nodes_xml)

    # Create edges
    edges_xml = '<?xml version="1.0" encoding="UTF-8"?>\n'
    edges_xml += '<edges xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/edges_file.xsd">\n'

    bottleneck_start, bottleneck_end = 4, 6

    for i in range(num_nodes):
        next_i = (i + 1) % num_nodes
        edges_xml += f'    <edge id="inner_e{i}" from="inner_{i}" to="inner_{next_i}" numLanes="1" speed="16.67" width="4.0"/>\n'
        if bottleneck_start <= i <= bottleneck_end:
            edges_xml += f'    <edge id="outer_e{i}" from="outer_{i}" to="outer_{next_i}" numLanes="1" speed="16.67" disallow="all" width="4.0"/>\n'
        else:
            edges_xml += f'    <edge id="outer_e{i}" from="outer_{i}" to="outer_{next_i}" numLanes="1" speed="16.67" width="4.0"/>\n'

    # Add merge links for lane change and return
    edges_xml += '    <edge id="merge_in" from="outer_3" to="inner_4" numLanes="1" speed="13.89" width="4.0"/>'\
                 '    <edge id="merge_out" from="inner_7" to="outer_8" numLanes="1" speed="13.89" width="4.0"/>'
    edges_xml += '</edges>'

    with open('bottleneck.edg.xml', 'w') as f:
        f.write(edges_xml)

    print("Generating SUMO network...")
    result = subprocess.run([
        'netconvert',
        '--node-files=bottleneck.nod.xml',
        '--edge-files=bottleneck.edg.xml',
        '--output-file=bottleneck.net.xml',
        '--no-turnarounds',
        '--junctions.corner-detail=5',
        '--default.lanewidth=4.0'
    ], capture_output=True, text=True)

    if result.returncode != 0:
        print(result.stderr)
        return False
    print("✓ Network created successfully!")
    return True

def create_route_file(num_inner=15, num_outer=25):
    """Create sorted routes with lane-changing behavior for outer vehicles."""

    routes_xml = '<?xml version="1.0" encoding="UTF-8"?>\n'
    routes_xml += '<routes xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/routes_file.xsd">\n'
    routes_xml += '    <vType id="car_inner" accel="4.0" decel="4.0" sigma="0.5" length="5" minGap="2.5" maxSpeed="16.67" color="1,1,0"/>'\
                  '    <vType id="car_outer" accel="4.0" decel="4.0" sigma="0.5" length="5" minGap="2.5" maxSpeed="16.67" color="1,0,0"/>\n'

    inner_edges = ' '.join([f'inner_e{i}' for i in range(16)])
    outer_edges = ' '.join([f'outer_e{i}' for i in range(16)])

    outer_route_with_merge = f'outer_e0 outer_e1 outer_e2 outer_e3 merge_in inner_e4 inner_e5 inner_e6 inner_e7 merge_out outer_e8 outer_e9 outer_e10 outer_e11 outer_e12 outer_e13 outer_e14 outer_e15'

    routes_xml += f'    <route id="inner_route" edges="{inner_edges}"/>\n'
    routes_xml += f'    <route id="outer_route" edges="{outer_route_with_merge}"/>\n'

    vehicles = []
    for i in range(num_inner):
        depart_time = i * 0.5
        vehicles.append((depart_time, f'    <vehicle id="inner_{i}" type="car_inner" route="inner_route" depart="{depart_time}" departSpeed="random" color="1,1,0"/>'))

    for i in range(num_outer):
        depart_time = (i * 0.5) + (num_inner * 0.5)
        vehicles.append((depart_time, f'    <vehicle id="outer_{i}" type="car_outer" route="outer_route" depart="{depart_time}" departSpeed="random" color="1,0,0"/>'))

    vehicles.sort(key=lambda v: v[0])
    for _, v in vehicles:
        routes_xml += v + '\n'

    routes_xml += '</routes>'

    with open('bottleneck.rou.xml', 'w') as f:
        f.write(routes_xml)

    print(f"✓ Route file created with {num_inner} inner + {num_outer} outer vehicles!")

def create_sumo_config():
    config_xml = '''<?xml version="1.0" encoding="UTF-8"?>
<configuration xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/sumoConfiguration.xsd">
    <input>
        <net-file value="bottleneck.net.xml"/>
        <route-files value="bottleneck.rou.xml"/>
    </input>
    <time>
        <begin value="0"/>
        <end value="300"/>
        <step-length value="0.1"/>
    </time>
    <processing>
        <collision.action value="warn"/>
        <time-to-teleport value="-1"/>
    </processing>
    <gui_only>
        <gui-settings-file value="viewsettings.xml"/>
    </gui_only>
</configuration>'''

    with open('bottleneck.sumocfg', 'w') as f:
        f.write(config_xml)
    print("✓ Configuration file created!")

def create_view_settings():
    view_xml = '''<?xml version="1.0" encoding="UTF-8"?>
<viewsettings>
    <scheme name="default">
        <background backgroundColor="255,255,255" showGrid="0"/>
        <vehicles vehicleColorer="given" vehicleSize="1.0"/>
    </scheme>
    <delay value="50"/>
    <viewport zoom="300" x="44" y="44"/>
</viewsettings>'''

    with open('viewsettings.xml', 'w') as f:
        f.write(view_xml)

def run_simulation():
    print("\nStarting SUMO-GUI simulation...\n")
    subprocess.run(['sumo-gui', '-c', 'bottleneck.sumocfg'])

if __name__ == '__main__':
    print("=" * 70)
    print("CIRCULAR ROAD MANDATORY LANE CHANGE SIMULATION")
    print("=" * 70)

    if not create_network_file():
        exit(1)
    create_route_file()
    create_sumo_config()
    create_view_settings()

    print("\n✓ All files generated successfully!")
    if input("Run simulation now? (y/n): ").lower() == 'y':
        run_simulation()