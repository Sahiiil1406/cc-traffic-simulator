import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.distributions import Normal
import traci
import math
from collections import defaultdict
import os

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
        
        # Bottleneck area definition
        self.bottleneck_start = 60.0
        self.bottleneck_end = 210.0
        self.buffer_zone_start = 40.0
        self.buffer_zone_end = 60.0
        
    def start(self):
        """Start SUMO simulation"""
        try:
            traci.start(['sumo', '-c', self.sumo_config, '--no-warnings', '--start'])
        except:
            traci.close()
            traci.start(['sumo', '-c', self.sumo_config, '--no-warnings', '--start'])
        
        # Wait for vehicles to appear
        for _ in range(100):
            traci.simulationStep()
            if len(traci.vehicle.getIDList()) >= 20:
                break
        
        self.vehicle_ids = list(traci.vehicle.getIDList())
        self.step_count = 0
        return self.get_observations()
    
    def get_observations(self):
        """Get state observations for all vehicles"""
        observations = {}
        
        for veh_id in traci.vehicle.getIDList():
            try:
                obs = self._get_vehicle_observation(veh_id)
                observations[veh_id] = obs
            except:
                pass
            
        return observations
    
    def _get_vehicle_observation(self, veh_id):
        """Get observation for a single vehicle (18 features)"""
        obs = np.zeros(18)
        
        try:
            pos = traci.vehicle.getPosition(veh_id)
            speed = traci.vehicle.getSpeed(veh_id)
            angle = traci.vehicle.getAngle(veh_id)
            lane_id = traci.vehicle.getLaneID(veh_id)
            
            obs[0] = speed
            obs[1] = 0.0
            obs[2] = angle
            obs[3] = pos[0]
            obs[4] = pos[1]
            obs[5] = self._get_distance_to_obstacle(pos)
            
            # Get leader info
            leader = traci.vehicle.getLeader(veh_id, dist=100)
            if leader:
                lead_id, lead_dist = leader
                lead_speed = traci.vehicle.getSpeed(lead_id)
                obs[6] = speed - lead_speed
                obs[10] = lead_dist
                obs[14] = self._compute_2d_ittc(veh_id, lead_id)
            else:
                obs[6] = 0.0
                obs[10] = 100.0
                obs[14] = 0.0
            
            # Simplified surrounding vehicle features
            obs[7] = 0.0
            obs[8] = 0.0
            obs[9] = 0.0
            obs[11] = 50.0
            obs[12] = 50.0
            obs[13] = 50.0
            obs[15] = 0.0
            obs[16] = 0.0
            obs[17] = 0.0
            
        except:
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
                return 0.5
            
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
                try:
                    safe_action = self._apply_collision_avoidance(veh_id, action)
                    accel_x, accel_y = safe_action
                    current_speed = traci.vehicle.getSpeed(veh_id)
                    new_speed = max(0, min(16.67, current_speed + accel_x * 0.1))
                    
                    traci.vehicle.setSpeed(veh_id, new_speed)
                    rewards[veh_id] = self._compute_reward(veh_id)
                except:
                    rewards[veh_id] = -1.0
        
        try:
            traci.simulationStep()
        except:
            pass
        
        self.step_count += 1
        
        next_observations = self.get_observations()
        done = self.step_count >= self.max_steps or len(traci.vehicle.getIDList()) == 0
        
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
            
            T = 0.5
            a_tilde = 4.0
            b_tilde = -4.0
            b_hat = -4.0
            S = 6.5
            
            term1 = v + 2.5 * a_tilde * T * (1 - v/16.67) * math.sqrt(0.025 + v/16.67)
            
            term2_inner = (b_tilde * T)**2 - b_tilde * (2 * (gap - S) - v * T - v_lead**2 / b_hat)
            if term2_inner >= 0:
                term2 = b_tilde * T + math.sqrt(term2_inner)
            else:
                term2 = v
            
            safe_speed = min(term1, term2)
            ittc = self._compute_2d_ittc(veh_id, lead_id)
            
            if ittc > 0.5:
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
            lane_id = traci.vehicle.getLaneID(veh_id)
            
            # Speed reward
            reward += speed / 16.67
            
            # Safety reward
            leader = traci.vehicle.getLeader(veh_id)
            if leader:
                ittc = self._compute_2d_ittc(veh_id, leader[0])
                if ittc > 0.5:
                    reward -= ittc / 0.5
            
            # Lane-specific rewards
            x = pos[0]
            y = pos[1]
            
            if 'outer' in lane_id:
                # Outer ring vehicles should move to inner ring in bottleneck
                if self.bottleneck_start <= x <= self.bottleneck_end:
                    if y < 50:  # Moved to inner area
                        reward += 2.0
            else:
                # Inner ring vehicles should stay centered
                if self.bottleneck_start <= x <= self.bottleneck_end:
                    reward += 2.0
            
            # Distance keeping reward
            if self.buffer_zone_start <= x <= self.buffer_zone_end:
                if leader and leader[1] > 7.0:
                    reward += 2.0
            
        except:
            reward = -1.0
            
        return reward
    
    def close(self):
        """Close SUMO simulation"""
        try:
            traci.close()
        except:
            pass

class IPPOAgent:
    """Independent PPO Agent"""
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
        
        action = action.cpu().numpy()[0]
        action[0] = np.clip(action[0], -4.0, 4.0)
        action[1] = np.clip(action[1], -1.0, 1.0)
        
        return action, log_prob.cpu().numpy()[0]
    
    def store_transition(self, state, action, reward, log_prob):
        """Store transition in memory"""
        self.memory.append((state, action, reward, log_prob))
    
    def update(self, batch_size=128, epochs=5):
        """Update policy using PPO"""
        if len(self.memory) < batch_size:
            self.memory = []
            return
        
        states = torch.FloatTensor([m[0] for m in self.memory])
        actions = torch.FloatTensor([m[1] for m in self.memory])
        rewards = [m[2] for m in self.memory]
        old_log_probs = torch.FloatTensor([m[3] for m in self.memory])
        
        with torch.no_grad():
            values = self.critic(states).squeeze().tolist()
            if isinstance(values, float):
                values = [values]
        
        advantages = []
        gae = 0
        for t in reversed(range(len(rewards))):
            if t == len(rewards) - 1:
                next_value = 0
            else:
                next_value = values[t + 1]
            delta = rewards[t] + self.gamma * next_value - values[t]
            gae = delta + self.gamma * self.lambda_gae * gae
            advantages.insert(0, gae)
        
        advantages = torch.FloatTensor(advantages)
        returns = advantages + torch.FloatTensor(values)
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)
        
        for _ in range(epochs):
            mean, log_std = self.actor(states)
            std = log_std.exp()
            dist = Normal(mean, std)
            new_log_probs = dist.log_prob(actions).sum(-1, keepdim=True)
            
            ratio = (new_log_probs - old_log_probs).exp()
            surr1 = ratio * advantages.unsqueeze(-1)
            surr2 = torch.clamp(ratio, 1-self.epsilon, 1+self.epsilon) * advantages.unsqueeze(-1)
            actor_loss = -torch.min(surr1, surr2).mean()
            
            value_pred = self.critic(states)
            critic_loss = nn.MSELoss()(value_pred, returns.unsqueeze(-1))
            
            self.actor_optimizer.zero_grad()
            actor_loss.backward()
            self.actor_optimizer.step()
            
            self.critic_optimizer.zero_grad()
            critic_loss.backward()
            self.critic_optimizer.step()
        
        self.actor_old.load_state_dict(self.actor.state_dict())
        self.memory = []

def train_marl():
    """Train multi-agent reinforcement learning system"""
    print("=" * 60)
    print("Training IPPO Multi-Agent System")
    print("=" * 60)
    
    os.makedirs('models', exist_ok=True)
    
    env = CAVEnvironment('bottleneck.sumocfg')
    
    state_dim = 18
    action_dim = 2
    agents = {}
    
    num_episodes = 50  # Reduced for testing
    episode_rewards = []
    
    for episode in range(num_episodes):
        print(f"\nEpisode {episode + 1}/{num_episodes}")
        
        observations = env.start()
        
        # Initialize agents
        if episode == 0:
            for veh_id in observations.keys():
                agents[veh_id] = IPPOAgent(state_dim, action_dim)
        
        episode_reward = 0
        done = False
        step = 0
        
        while not done and step < 1500:
            actions = {}
            log_probs = {}
            
            for veh_id, obs in observations.items():
                if veh_id not in agents:
                    agents[veh_id] = IPPOAgent(state_dim, action_dim)
                
                action, log_prob = agents[veh_id].select_action(obs)
                actions[veh_id] = action
                log_probs[veh_id] = log_prob
            
            next_observations, rewards, done = env.step(actions)
            
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
        
        # Update agents
        for veh_id, agent in agents.items():
            agent.update()
        
        avg_reward = episode_reward / max(1, step * len(agents))
        episode_rewards.append(avg_reward)
        print(f"Episode {episode + 1} finished. Avg Reward: {avg_reward:.4f}")
        
        env.close()
        
        # Save models
        if (episode + 1) % 10 == 0:
            for veh_id, agent in list(agents.items())[:5]:  # Save first 5 agents
                torch.save(agent.actor.state_dict(), f'models/actor_{veh_id}.pth')
                torch.save(agent.critic.state_dict(), f'models/critic_{veh_id}.pth')
    
    print("\n" + "=" * 60)
    print("Training completed!")
    print("=" * 60)
    
    return episode_rewards

if __name__ == '__main__':
    rewards = train_marl()