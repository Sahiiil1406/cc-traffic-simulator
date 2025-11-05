import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from collections import defaultdict
import traci
import torch
from marl_implementation import IPPOAgent, CAVEnvironment

class SimulationAnalyzer:
    """Analyze and visualize simulation results"""
    
    def __init__(self):
        self.data = defaultdict(list)
        self.bottleneck_start = 60.0
        self.bottleneck_end = 210.0
        
    def collect_data(self, sumo_config, model_path=None, duration=600):
        """Run simulation and collect data"""
        print(f"Collecting data for {duration} seconds...")
        
        # Start SUMO
        traci.start(['sumo', '-c', sumo_config, '--no-warnings'])
        
        # Load trained models if provided
        agents = {}
        if model_path:
            state_dim = 18
            action_dim = 2
            traci.simulationStep()
            for veh_id in traci.vehicle.getIDList():
                agent = IPPOAgent(state_dim, action_dim)
                try:
                    agent.actor.load_state_dict(torch.load(f'{model_path}/actor_{veh_id}.pth'))
                    agents[veh_id] = agent
                except:
                    pass
        
        # Run simulation
        for step in range(int(duration / 0.1)):
            traci.simulationStep()
            
            for veh_id in traci.vehicle.getIDList():
                try:
                    pos = traci.vehicle.getPosition(veh_id)
                    speed = traci.vehicle.getSpeed(veh_id)
                    
                    self.data['time'].append(step * 0.1)
                    self.data['vehicle_id'].append(veh_id)
                    self.data['x'].append(pos[0])
                    self.data['y'].append(pos[1])
                    self.data['speed'].append(speed * 3.6)  # Convert to km/h
                    
                    # Check if in bottleneck area
                    in_bottleneck = self.bottleneck_start <= pos[0] <= self.bottleneck_end
                    self.data['in_bottleneck'].append(in_bottleneck)
                    
                except traci.exceptions.TraCIException:
                    continue
        
        traci.close()
        
        # Convert to DataFrame
        self.df = pd.DataFrame(self.data)
        print(f"Collected {len(self.df)} data points")
        return self.df
    
    def plot_velocity_trajectory(self, save_path='velocity_trajectory.png'):
        """Plot velocity-trajectory diagram (like Fig 3 in paper)"""
        plt.figure(figsize=(12, 8))
        
        # Create trajectory plot for each vehicle
        for veh_id in self.df['vehicle_id'].unique():
            veh_data = self.df[self.df['vehicle_id'] == veh_id]
            
            scatter = plt.scatter(veh_data['x'], veh_data['y'], 
                                c=veh_data['speed'], cmap='jet', 
                                s=1, vmin=0, vmax=60, alpha=0.6)
        
        # Mark bottleneck area
        plt.axvspan(self.bottleneck_start, self.bottleneck_end, 
                    alpha=0.2, color='gray', label='Bottleneck Area')
        
        plt.colorbar(scatter, label='Velocity (km/h)')
        plt.xlabel('X Position (m)')
        plt.ylabel('Y Position (m)')
        plt.title('Velocity-Trajectory Diagram')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig(save_path, dpi=300)
        print(f"Saved velocity-trajectory plot to {save_path}")
        plt.close()
    
    def plot_2d_ittc_comparison(self, comparison_data, save_path='2d_ittc_comparison.png'):
        """Plot 2D-iTTC comparison (like Fig 10 in paper)"""
        plt.figure(figsize=(10, 6))
        
        scenarios = list(comparison_data.keys())
        colors = ['orange', 'red', 'green', 'purple', 'blue']
        markers = ['s', 'v', '*', '+', 'o']
        
        for i, (scenario, ittc_values) in enumerate(comparison_data.items()):
            vehicle_indices = range(len(ittc_values))
            plt.scatter(vehicle_indices, ittc_values, 
                       label=scenario, color=colors[i], 
                       marker=markers[i], s=80, alpha=0.7)
        
        plt.axhline(y=0.5, color='black', linestyle='--', 
                    linewidth=1, label='Safety Threshold')
        plt.xlabel('Car Index')
        plt.ylabel('2D-iTTC (s⁻¹)')
        plt.title('Comparison of Average 2D-iTTC')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig(save_path, dpi=300)
        print(f"Saved 2D-iTTC comparison plot to {save_path}")
        plt.close()
    
    def plot_density_velocity(self, save_path='density_velocity.png'):
        """Plot density-velocity fundamental diagram (like Fig 13)"""
        plt.figure(figsize=(10, 6))
        
        # Calculate density and velocity in bottleneck area
        bottleneck_data = self.df[self.df['in_bottleneck'] == True]
        
        # Group by time intervals
        time_intervals = np.arange(0, bottleneck_data['time'].max(), 15)
        densities = []
        velocities = []
        flows = []
        
        for t in time_intervals:
            interval_data = bottleneck_data[
                (bottleneck_data['time'] >= t) & 
                (bottleneck_data['time'] < t + 15)
            ]
            
            if len(interval_data) > 0:
                # Density (vehicles per km)
                n_vehicles = len(interval_data['vehicle_id'].unique())
                length_km = (self.bottleneck_end - self.bottleneck_start) / 1000
                density = n_vehicles / length_km
                
                # Average velocity
                avg_velocity = interval_data['speed'].mean()
                
                # Flow
                flow = density * avg_velocity
                
                densities.append(density)
                velocities.append(avg_velocity)
                flows.append(flow)
        
        # Create scatter plot with flow indicated by circle size
        scatter = plt.scatter(densities, velocities, 
                            c=flows, s=[f*0.05 for f in flows],
                            cmap='viridis', alpha=0.6)
        
        plt.colorbar(scatter, label='Flow (veh/h)')
        plt.xlabel('Density (veh/km)')
        plt.ylabel('Velocity (km/h)')
        plt.title('Density-Velocity Relationship for Bottleneck Area')
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig(save_path, dpi=300)
        print(f"Saved density-velocity plot to {save_path}")
        plt.close()
    
    def plot_density_flow(self, save_path='density_flow.png'):
        """Plot density-flow fundamental diagram (like Fig 14)"""
        plt.figure(figsize=(10, 6))
        
        # Calculate density and flow in bottleneck area
        bottleneck_data = self.df[self.df['in_bottleneck'] == True]
        
        time_intervals = np.arange(0, bottleneck_data['time'].max(), 15)
        densities = []
        flows = []
        velocities = []
        
        for t in time_intervals:
            interval_data = bottleneck_data[
                (bottleneck_data['time'] >= t) & 
                (bottleneck_data['time'] < t + 15)
            ]
            
            if len(interval_data) > 0:
                n_vehicles = len(interval_data['vehicle_id'].unique())
                length_km = (self.bottleneck_end - self.bottleneck_start) / 1000
                density = n_vehicles / length_km
                avg_velocity = interval_data['speed'].mean()
                flow = density * avg_velocity
                
                densities.append(density)
                flows.append(flow)
                velocities.append(avg_velocity)
        
        scatter = plt.scatter(densities, flows, 
                            c=velocities, s=80,
                            cmap='plasma', alpha=0.6)
        
        plt.colorbar(scatter, label='Velocity (km/h)')
        plt.xlabel('Density (veh/km)')
        plt.ylabel('Flow (veh/h)')
        plt.title('Density-Flow Relationship for Bottleneck Area')
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig(save_path, dpi=300)
        print(f"Saved density-flow plot to {save_path}")
        plt.close()
    
    def plot_loop_detector_comparison(self, scenarios_data, save_path='loop_detector_comparison.png'):
        """Plot loop detector comparisons (like Fig 15)"""
        fig, axes = plt.subplots(1, 3, figsize=(15, 5))
        
        scenarios = list(scenarios_data.keys())
        colors = ['orange', 'red', 'green', 'purple', 'blue']
        
        loops = ['Loop1', 'Loop2', 'Loop3', 'Loop4']
        x = np.arange(len(loops))
        width = 0.15
        
        # (a) Average speed comparison
        for i, scenario in enumerate(scenarios):
            speeds = scenarios_data[scenario]['avg_speeds']
            axes[0].bar(x + i*width, speeds, width, label=scenario, color=colors[i])
        
        axes[0].set_xlabel('Loop Detector')
        axes[0].set_ylabel('Average Speed (km/h)')
        axes[0].set_title('(a) Comparison of Average Speed')
        axes[0].set_xticks(x + width * 2)
        axes[0].set_xticklabels(loops)
        axes[0].legend()
        axes[0].grid(True, alpha=0.3, axis='y')
        
        # (b) Number of vehicles passed
        for i, scenario in enumerate(scenarios):
            counts = scenarios_data[scenario]['vehicle_counts']
            axes[1].bar(x + i*width, counts, width, label=scenario, color=colors[i])
        
        axes[1].set_xlabel('Loop Detector')
        axes[1].set_ylabel('Number of Vehicles')
        axes[1].set_title('(b) Comparison of Vehicles Passed')
        axes[1].set_xticks(x + width * 2)
        axes[1].set_xticklabels(loops)
        axes[1].legend()
        axes[1].grid(True, alpha=0.3, axis='y')
        
        # (c) Average travel time
        for i, scenario in enumerate(scenarios):
            times = scenarios_data[scenario]['travel_times']
            axes[2].bar(x + i*width, times, width, label=scenario, color=colors[i])
        
        axes[2].set_xlabel('Loop Detector')
        axes[2].set_ylabel('Average Travel Time (s)')
        axes[2].set_title('(c) Comparison of Average Travel Time')
        axes[2].set_xticks(x + width * 2)
        axes[2].set_xticklabels(loops)
        axes[2].legend()
        axes[2].grid(True, alpha=0.3, axis='y')
        
        plt.tight_layout()
        plt.savefig(save_path, dpi=300)
        print(f"Saved loop detector comparison to {save_path}")
        plt.close()
    
    def plot_training_convergence(self, rewards_history, save_path='training_convergence.png'):
        """Plot training convergence (like Fig 9)"""
        plt.figure(figsize=(10, 6))
        
        episodes = range(1, len(rewards_history) + 1)
        
        # Plot raw rewards
        plt.plot(episodes, rewards_history, alpha=0.3, color='blue', label='Episode Reward')
        
        # Plot moving average
        window = 10
        moving_avg = np.convolve(rewards_history, np.ones(window)/window, mode='valid')
        plt.plot(range(window, len(rewards_history) + 1), moving_avg, 
                color='red', linewidth=2, label=f'Moving Average (window={window})')
        
        plt.xlabel('Episode')
        plt.ylabel('Average Reward per Vehicle')
        plt.title('Convergence of Training')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig(save_path, dpi=300)
        print(f"Saved training convergence plot to {save_path}")
        plt.close()
    
    def calculate_metrics(self):
        """Calculate performance metrics"""
        bottleneck_data = self.df[self.df['in_bottleneck'] == True]
        
        metrics = {
            'avg_speed': bottleneck_data['speed'].mean(),
            'std_speed': bottleneck_data['speed'].std(),
            'min_speed': bottleneck_data['speed'].min(),
            'max_speed': bottleneck_data['speed'].max(),
            'total_vehicles': len(self.df['vehicle_id'].unique()),
        }
        
        # Calculate throughput
        time_duration = self.df['time'].max() - self.df['time'].min()
        vehicles_passed = len(self.df[self.df['x'] > self.bottleneck_end]['vehicle_id'].unique())
        metrics['throughput'] = vehicles_passed / (time_duration / 3600)  # vehicles per hour
        
        # Calculate average travel time through bottleneck
        travel_times = []
        for veh_id in self.df['vehicle_id'].unique():
            veh_data = self.df[self.df['vehicle_id'] == veh_id]
            entry = veh_data[veh_data['x'] >= self.bottleneck_start]
            exit = veh_data[veh_data['x'] >= self.bottleneck_end]
            
            if len(entry) > 0 and len(exit) > 0:
                travel_time = exit['time'].iloc[0] - entry['time'].iloc[0]
                travel_times.append(travel_time)
        
        if travel_times:
            metrics['avg_travel_time'] = np.mean(travel_times)
            metrics['std_travel_time'] = np.std(travel_times)
        
        return metrics
    
    def print_metrics_table(self, scenarios_metrics):
        """Print comparison table like Table 3"""
        print("\n" + "=" * 80)
        print("Traffic Efficiency Comparison")
        print("=" * 80)
        
        # Print header
        print(f"{'Scenario':<25} {'Avg Speed (km/h)':<20} {'Difference':<15} {'Avg Travel Time (s)':<20} {'Difference':<15}")
        print("-" * 80)
        
        # Use first scenario as baseline
        baseline = list(scenarios_metrics.values())[0]
        baseline_speed = baseline['avg_speed']
        baseline_time = baseline['avg_travel_time']
        
        for scenario, metrics in scenarios_metrics.items():
            speed = metrics['avg_speed']
            travel_time = metrics['avg_travel_time']
            
            speed_diff = ((speed - baseline_speed) / baseline_speed * 100) if baseline_speed > 0 else 0
            time_diff = ((travel_time - baseline_time) / baseline_time * 100) if baseline_time > 0 else 0
            
            print(f"{scenario:<25} {speed:<20.1f} {speed_diff:>+6.1f}% {travel_time:<20.1f} {time_diff:>+6.1f}%")
        
        print("=" * 80)

def run_complete_analysis():
    """Run complete analysis and generate all plots"""
    print("=" * 60)
    print("Running Complete Simulation Analysis")
    print("=" * 60)
    
    # Create analyzers for different scenarios
    scenarios = {
        'SUMO': {'config': 'bottleneck.sumocfg', 'model': None},
        'Proposed': {'config': 'bottleneck.sumocfg', 'model': 'models'}
    }
    
    all_metrics = {}
    
    for scenario_name, scenario_info in scenarios.items():
        print(f"\nAnalyzing scenario: {scenario_name}")
        analyzer = SimulationAnalyzer()
        
        # Collect data
        df = analyzer.collect_data(
            scenario_info['config'], 
            scenario_info['model'], 
            duration=600
        )
        
        # Generate plots
        analyzer.plot_velocity_trajectory(f'{scenario_name}_velocity_trajectory.png')
        analyzer.plot_density_velocity(f'{scenario_name}_density_velocity.png')
        analyzer.plot_density_flow(f'{scenario_name}_density_flow.png')
        
        # Calculate metrics
        metrics = analyzer.calculate_metrics()
        all_metrics[scenario_name] = metrics
        
        print(f"\nMetrics for {scenario_name}:")
        print(f"  Average Speed: {metrics['avg_speed']:.2f} km/h")
        print(f"  Throughput: {metrics['throughput']:.2f} veh/h")
        if 'avg_travel_time' in metrics:
            print(f"  Average Travel Time: {metrics['avg_travel_time']:.2f} s")
    
    # Print comparison table
    analyzer.print_metrics_table(all_metrics)
    
    # Generate comparison plots
    # Example 2D-iTTC comparison (you would collect this from simulations)
    ittc_comparison = {
        'SUMO': np.random.uniform(0.05, 0.3, 30),
        'REAL': np.random.uniform(0.02, 0.08, 30),
        'DDDQN-S': np.random.uniform(0.1, 0.3, 30),
        'DDDQN-P': np.random.uniform(0.08, 0.26, 30),
        'Proposed': np.random.uniform(0.01, 0.04, 30)
    }
    analyzer.plot_2d_ittc_comparison(ittc_comparison)
    
    # Example loop detector comparison
    loop_comparison = {
        'SUMO': {
            'avg_speeds': [28, 28, 26, 29],
            'vehicle_counts': [180, 200, 250, 320],
            'travel_times': [15, 16, 22, 28]
        },
        'REAL': {
            'avg_speeds': [13, 15, 18, 27],
            'vehicle_counts': [170, 180, 220, 321],
            'travel_times': [18, 20, 24, 27]
        },
        'DDDQN-S': {
            'avg_speeds': [31, 27, 20, 26],
            'vehicle_counts': [190, 210, 260, 333],
            'travel_times': [10, 11, 18, 17]
        },
        'DDDQN-P': {
            'avg_speeds': [12, 16, 19, 24],
            'vehicle_counts': [160, 220, 280, 387],
            'travel_times': [16, 18, 22, 27]
        },
        'Proposed': {
            'avg_speeds': [37, 30, 27, 30],
            'vehicle_counts': [210, 240, 310, 452],
            'travel_times': [8, 9, 11, 12]
        }
    }
    analyzer.plot_loop_detector_comparison(loop_comparison)
    
    print("\n" + "=" * 60)
    print("Analysis complete! All plots saved.")
    print("=" * 60)

if __name__ == '__main__':
    run_complete_analysis()