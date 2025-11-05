import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.distributions import Normal
import traci
import sys
import math
from collections import deque

# Neural Network for Actor (Policy)
class ActorNetwork(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_dim=128):
        super(ActorNetwork, self).__init__()
        self.fc1 = nn.Linear(state_dim, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.mean = nn.Linear(hidden_dim, action_dim)
        self.log_std = nn.Linear(hidden_dim, action_dim)
        
    def forward(self, state):
        x = torch.relu(self.fc1(state))
        x = torch.relu(self.fc2(x))
        mean = self.mean(x)
        log_std = self.log_std(x)
        log_std = torch.clamp(log_std, -20, 2)
        return mean, log_std
    
    def sample(self, state):
        mean, log_std = self.forward(state)
        std = log_std.exp()
        normal = Normal(mean, std)
        action = normal.sample()
        log_prob = normal.log_prob(action).sum(-1, keepdim=True)
        return action, log_prob

# Neural Network for Critic (Value Function)
class CriticNetwork(nn.Module):
    def __init__(self, state_dim, hidden_dim=128):
        super(CriticNetwork, self).__init__()
        self.fc1 = nn.Linear(state_dim, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.value = nn.Linear(hidden_dim, 1)
        
    def forward(self, state):
        x = torch.relu(self.fc1(state))
        x = torch.relu(self.fc2(x))
        value = self.value(x)
        return value

class CAVEnvironment:
    """Environment wrapper for SUMO simulation"""
    def __init__(self, sumo_config):
        self.sumo_config = sumo_config
        self.vehicle_ids = []
        self.step_count = 0
        self.max_steps = 3000
        
        # Bottleneck area definition (in meters)
        self.bottleneck_start = 60.0  # x-coordinate
        self.bottleneck_end = 210.0
        self.buffer_zone_start = 40.0
        self.buffer_zone_end = 60.0
        
    def start(self):
        """Start SUMO simulation"""
        traci.start(['sumo', '-c', self.sumo_config, '--no-warnings'])
        traci.simulationStep()
        self.vehicle_ids = traci.vehicle.getIDList()
        return self.get_observations()
    
    def get_observations(self):
        """Get state observations for all vehicles"""
        observations = {}
        
        for veh_id in traci.vehicle.getIDList():
            obs = self._get_vehicle_observation(veh_id)
            observations[veh_id] = obs
            
        return observations
    
    def _get_vehicle_observation(self, veh_id):
        """Get observation for a single vehicle (18 features as per paper)"""
        obs = np.zeros(18)
        
        try:
            # Vehicle's own state
            pos = traci.vehicle.getPosition(veh_id)
            speed = traci.vehicle.getSpeed(veh_id)
            angle = traci.vehicle.getAngle(veh_id)
            
            obs[0] = speed  # v_x (longitudinal speed)
            obs[1] = 0.0  # v_y (lateral speed, approximated)
            obs[2] = angle  # steering angle
            obs[3] = pos[0]  # x coordinate
            obs[4] = pos[1]  # y coordinate
            
            # Distance to obstacle
            obs[5] = self._get_distance_to_obstacle(pos)
            
            # Get surrounding vehicles info
            leader = traci.vehicle.getLeader(veh_id, dist=100)
            
            if leader:
                lead_id, lead_dist = leader
                lead_speed = traci.vehicle.getSpeed(lead_id)
                obs[6] = speed - lead_speed  # Δv_inner_lead
                obs[10] = lead_dist  # Δx_inner_lead
                obs[14] = self._compute_2d_ittc(veh_id, lead_id)
            else:
                obs[6] = 0.0
                obs[10] = 100.0
                obs[14] = 0.0
            
            # Simplified: set other surrounding vehicle features
            obs[7] = 0.0  # Δv_inner_follow
            obs[8] = 0.0  # Δv_outer_lead
            obs[9] = 0.0  # Δv_outer_follow
            obs[11] = 50.0  # Δx_inner_follow
            obs[12] = 50.0  # Δx_outer_lead
            obs[13] = 50.0  # Δx_outer_follow
            obs[15] = 0.0  # 2D-iTTC_inner_follow
            obs[16] = 0.0  # 2D-iTTC_outer_lead
            obs[17] = 0.0  # 2D-iTTC_outer_follow
            
        except traci.exceptions.TraCIException:
            pass
            
        return obs
    
    def _get_distance_to_obstacle(self, pos):
        """Calculate distance to bottleneck obstacle"""
        x, y = pos
        if self.bottleneck_start <= x <= self.bottleneck_end:
            return 0.0
        elif x < self.bottleneck_start:
            return self.bottleneck_start - x
        else:
            return 100.0
    
    def _compute_2d_ittc(self, veh_id, lead_id):
        """Compute 2D inverse time-to-collision"""
        try:
            pos = traci.vehicle.getPosition(veh_id)
            lead_pos = traci.vehicle.getPosition(lead_id)
            
            dx = lead_pos[0] - pos[0]
            dy = lead_pos[1] - pos[1]
            d_lead = math.sqrt(dx**2 + dy**2)
            
            if d_lead < 5.0:
                return 0.5  # Maximum threshold
            
            speed = traci.vehicle.getSpeed(veh_id)
            lead_speed = traci.vehicle.getSpeed(lead_id)
            
            angle = traci.vehicle.getAngle(veh_id) * math.pi / 180
            lead_angle = traci.vehicle.getAngle(lead_id) * math.pi / 180
            
            phi = math.atan2(dy, dx)
            
            v_rel = speed * math.cos(phi - angle) - lead_speed * math.cos(phi - lead_angle)
            
            if v_rel <= 0:
                return 0.0
            
            ittc = v_rel / (d_lead - 5.0)
            return min(ittc, 0.5)
            
        except:
            return 0.0
    
    def step(self, actions):
        """Execute actions and return next observations, rewards, done"""
        rewards = {}
        
        for veh_id, action in actions.items():
            if veh_id in traci.vehicle.getIDList():
                # Apply collision avoidance strategy
                safe_action = self._apply_collision_avoidance(veh_id, action)
                
                # Execute action (acceleration)
                accel_x, accel_y = safe_action
                current_speed = traci.vehicle.getSpeed(veh_id)
                new_speed = max(0, min(16.67, current_speed + accel_x * 0.1))
                
                try:
                    traci.vehicle.setSpeed(veh_id, new_speed)
                except:
                    pass
                
                # Calculate reward
                rewards[veh_id] = self._compute_reward(veh_id)
        
        traci.simulationStep()
        self.step_count += 1
        
        next_observations = self.get_observations()
        done = self.step_count >= self.max_steps
        
        return next_observations, rewards, done
    
    def _apply_collision_avoidance(self, veh_id, action):
        """Apply Gipps model-based collision avoidance"""
        accel_x, accel_y = action
        
        try:
            leader = traci.vehicle.getLeader(veh_id, dist=50)
            if leader is None:
                return action
            
            lead_id, gap = leader
            v = traci.vehicle.getSpeed(veh_id)
            v_lead = traci.vehicle.getSpeed(lead_id)
            
            # Gipps model safe speed
            T = 0.5  # reaction time
            a_tilde = 4.0  # desired acceleration
            b_tilde = -4.0  # desired deceleration
            b_hat = -4.0  # estimated lead deceleration
            S = 6.5  # safe distance
            
            # Free flow term
            term1 = v + 2.5 * a_tilde * T * (1 - v/16.67) * math.sqrt(0.025 + v/16.67)
            
            # Congested flow term
            term2_inner = (b_tilde * T)**2 - b_tilde * (2 * (gap - S) - v * T - v_lead**2 / b_hat)
            if term2_inner >= 0:
                term2 = b_tilde * T + math.sqrt(term2_inner)
            else:
                term2 = v
            
            safe_speed = min(term1, term2)
            
            # Check 2D-iTTC threshold
            ittc = self._compute_2d_ittc(veh_id, lead_id)
            
            if ittc > 0.5:  # Exceeds safety threshold
                # Limit acceleration
                max_accel = (safe_speed - v) / 0.1
                accel_x = min(accel_x, max_accel)
            
        except:
            pass
            
        return (accel_x, accel_y)
    
    def _compute_reward(self, veh_id):
        """Compute reward based on paper's reward function"""
        reward = 0.0
        
        try:
            pos = traci.vehicle.getPosition(veh_id)
            speed = traci.vehicle.getSpeed(veh_id)
            
            # 2D-iTTC reward
            leader = traci.vehicle.getLeader(veh_id)
            if leader:
                ittc = self._compute_2d_ittc(veh_id, leader[0])
                if ittc > 0.5:
                    reward += max(min(-ittc/0.5, 0), -1)
            
            # Speed reward
            reward += math.sqrt(speed**2) / 16.67
            
            # Distance keeping in buffer zone
            x = pos[0]
            if self.buffer_zone_start <= x <= self.buffer_zone_end:
                if leader and leader[1] > 7.0:
                    reward += 2.0
            
            # Lane-specific rewards
            y = pos[1]
            lane_id = traci.vehicle.getLaneID(veh_id)
            
            if 'outer' in veh_id:
                # Outer ring vehicles should move to inner ring
                if self.bottleneck_start <= x <= self.bottleneck_end:
                    if 'inner' in lane_id or y < 30:
                        reward += 2.0
            else:
                # Inner ring vehicles should stay centered
                if self.bottleneck_start <= x <= self.bottleneck_end:
                    if abs(y - 25) < 2:
                        reward += 2.0
            
        except:
            pass
            
        return reward
    
    def close(self):
        """Close SUMO simulation"""
        traci.close()

class IPPOAgent:
    """Independent PPO Agent for each vehicle"""
    def __init__(self, state_dim, action_dim, lr=3e-4):
        self.actor = ActorNetwork(state_dim, action_dim)
        self.critic = CriticNetwork(state_dim)
        self.actor_old = ActorNetwork(state_dim, action_dim)
        self.actor_old.load_state_dict(self.actor.state_dict())
        
        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=lr)
        self.critic_optimizer = optim.Adam(self.critic.parameters(), lr=lr)
        
        self.gamma = 0.99
        self.lambda_gae = 0.95
        self.epsilon = 0.2
        
        self.memory = []
        
    def select_action(self, state):
        """Select action using current policy"""
        state = torch.FloatTensor(state).unsqueeze(0)
        with torch.no_grad():
            action, log_prob = self.actor_old.sample(state)
        
        # Clip actions to valid range
        action = action.cpu().numpy()[0]
        action[0] = np.clip(action[0], -4.0, 4.0)  # longitudinal accel
        action[1] = np.clip(action[1], -1.0, 1.0)  # lateral accel
        
        return action, log_prob.cpu().numpy()[0]
    
    def store_transition(self, state, action, reward, log_prob):
        """Store transition in memory"""
        self.memory.append((state, action, reward, log_prob))
    
    def compute_gae(self, rewards, values, next_value):
        """Compute Generalized Advantage Estimation"""
        advantages = []
        gae = 0
        
        values = values + [next_value]
        
        for t in reversed(range(len(rewards))):
            delta = rewards[t] + self.gamma * values[t + 1] - values[t]
            gae = delta + self.gamma * self.lambda_gae * gae
            advantages.insert(0, gae)
        
        return advantages
    
    def update(self, batch_size=128, epochs=10):
        """Update policy using PPO"""
        if len(self.memory) < batch_size:
            return
        
        # Extract data from memory
        states = torch.FloatTensor([m[0] for m in self.memory])
        actions = torch.FloatTensor([m[1] for m in self.memory])
        rewards = [m[2] for m in self.memory]
        old_log_probs = torch.FloatTensor([m[3] for m in self.memory])
        
        # Compute values and advantages
        with torch.no_grad():
            values = self.critic(states).squeeze().tolist()
            next_value = 0  # Terminal value
        
        advantages = self.compute_gae(rewards, values, next_value)
        advantages = torch.FloatTensor(advantages)
        returns = advantages + torch.FloatTensor(values)
        
        # Normalize advantages
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)
        
        # PPO update
        for _ in range(epochs):
            for idx in range(0, len(states), batch_size):
                batch_states = states[idx:idx+batch_size]
                batch_actions = actions[idx:idx+batch_size]
                batch_old_log_probs = old_log_probs[idx:idx+batch_size]
                batch_advantages = advantages[idx:idx+batch_size]
                batch_returns = returns[idx:idx+batch_size]
                
                # Actor loss
                mean, log_std = self.actor(batch_states)
                std = log_std.exp()
                dist = Normal(mean, std)
                new_log_probs = dist.log_prob(batch_actions).sum(-1, keepdim=True)
                
                ratio = (new_log_probs - batch_old_log_probs).exp()
                surr1 = ratio * batch_advantages.unsqueeze(-1)
                surr2 = torch.clamp(ratio, 1-self.epsilon, 1+self.epsilon) * batch_advantages.unsqueeze(-1)
                actor_loss = -torch.min(surr1, surr2).mean()
                
                # Critic loss
                value_pred = self.critic(batch_states)
                critic_loss = nn.MSELoss()(value_pred, batch_returns.unsqueeze(-1))
                
                # Update networks
                self.actor_optimizer.zero_grad()
                actor_loss.backward()
                self.actor_optimizer.step()
                
                self.critic_optimizer.zero_grad()
                critic_loss.backward()
                self.critic_optimizer.step()
        
        # Update old policy
        self.actor_old.load_state_dict(self.actor.state_dict())
        
        # Clear memory
        self.memory = []

def train_marl():
    """Train multi-agent reinforcement learning system"""
    print("=" * 60)
    print("Training IPPO Multi-Agent System")
    print("=" * 60)
    
    # Initialize environment
    env = CAVEnvironment('bottleneck.sumocfg')
    
    # Initialize agents for each vehicle
    state_dim = 18
    action_dim = 2
    agents = {}
    
    num_episodes = 300
    
    for episode in range(num_episodes):
        print(f"\nEpisode {episode + 1}/{num_episodes}")
        
        # Start episode
        observations = env.start()
        
        # Initialize agents if first episode
        if episode == 0:
            for veh_id in observations.keys():
                agents[veh_id] = IPPOAgent(state_dim, action_dim)
        
        episode_reward = 0
        done = False
        step = 0
        
        while not done:
            actions = {}
            log_probs = {}
            
            # Select actions for all vehicles
            for veh_id, obs in observations.items():
                if veh_id in agents:
                    action, log_prob = agents[veh_id].select_action(obs)
                    actions[veh_id] = action
                    log_probs[veh_id] = log_prob
            
            # Execute actions
            next_observations, rewards, done = env.step(actions)
            
            # Store transitions
            for veh_id in actions.keys():
                if veh_id in agents and veh_id in rewards:
                    agents[veh_id].store_transition(
                        observations[veh_id],
                        actions[veh_id],
                        rewards[veh_id],
                        log_probs[veh_id]
                    )
                    episode_reward += rewards[veh_id]
            
            observations = next_observations
            step += 1
            
            if step % 100 == 0:
                print(f"  Step {step}, Avg Reward: {episode_reward / (step * len(agents)):.4f}")
        
        # Update all agents
        for veh_id, agent in agents.items():
            agent.update()
        
        print(f"Episode {episode + 1} finished. Total Reward: {episode_reward:.2f}")
        
        env.close()
        
        # Save models periodically
        if (episode + 1) % 50 == 0:
            for veh_id, agent in agents.items():
                torch.save(agent.actor.state_dict(), f'models/actor_{veh_id}_{episode+1}.pth')
                torch.save(agent.critic.state_dict(), f'models/critic_{veh_id}_{episode+1}.pth')
    
    print("\n" + "=" * 60)
    print("Training completed!")
    print("=" * 60)

if __name__ == '__main__':
    import os
    os.makedirs('models', exist_ok=True)
    train_marl()