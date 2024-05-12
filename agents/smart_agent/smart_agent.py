from smart_agent.smart_agent_env import TerraBotEnvironment
import torch
import torch.nn as nn
import torch.optim as optim

env = TerraBotEnvironment()

state_size = env.observation_space.shape[0]
action_size = env.action_space.n

# Define the policy network
class Policy(nn.Module):
    def __init__(self):
        super(Policy, self).__init__()
        self.fc1 = nn.Linear(state_size, 32)
        self.fc2 = nn.Linear(32, action_size)

    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = self.fc2(x)
        return torch.softmax(x, dim=-1)

policy = Policy()

loss_fn = nn.CrossEntropyLoss()
optimizer = optim.Adam(policy.parameters())

def update_policy(rewards, log_probs, optimizer):
    log_probs = torch.stack(log_probs)
    loss = -torch.mean(log_probs * sum(rewards))
    optimizer.zero_grad()
    loss.backward()
    optimizer.step()

for episode in range(10000):
	state, _ = env.reset()
	done = False
	rewards = []
	log_probs = []
	
	while not done:
		# Select action
		state = torch.tensor(state, dtype=torch.float32).reshape(1, -1)
		probs = policy(state)
		action = torch.multinomial(probs, 1).item()
		log_prob = torch.log(probs[0, action])

		# Take step
		next_state, reward, done, _, _ = env.step(action)
		rewards.append(reward)
		log_probs.append(log_prob)
		state = next_state
		
	# Update policy
	if episode % 1000 == 0:
		print(f"Episode {episode}: {sum(rewards)}")
	update_policy(rewards, log_probs, optimizer)
	rewards = []
	log_probs = []
