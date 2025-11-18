import random
from collections import defaultdict
from typing import Tuple, Dict, Any, List

# Class to generate 25x25 grid maze
class MazeEnv:
    """
    Agent has, position (x, y), orientation (0: up, 1: right, 2: down, 3: left), three proximity sensors (front, left, right) as blocked/not-blocked flags.
    Actions:
      0: forward
      1: backward
      2: hard left  (turn left 90deg, then move forward)
      3: hard right (turn right 90deg, then move forward)
    """
    ORI_TO_DELTA = {
        0: (-1, 0),  # up
        1: (0, 1),   # right
        2: (1, 0),   # down
        3: (0, -1),  # left
    }

    # init the parameters of the Class MazeEnv
    def __init__(self, size: int = 25, max_steps: int = 200, seed: int = 42):
        self.size = size
        self.max_steps = max_steps
        self.rng = random.Random(seed)

        self.maze = self._generate_maze()
        self.start_pos = (1, 1)
        self.goal_pos = (self.size - 2, self.size - 2)

        self.pos = self.start_pos
        self.ori = 0  # facing up
        self.steps_taken = 0

        # Reward parameters (we need to tune these and record as we go)
        self.step_reward = -0.05          # small time penalty
        self.invalid_penalty = -1.0       # hitting wall / boundary
        self.goal_reward = 10.0           # reaching the target cell
        self.timeout_penalty = -2.0       # failed to finish in time

    # Some MATLAB code translated to python idk if this will work
    def _generate_maze(self) -> List[List[int]]:
        """
        Create a simple maze with outer walls and some internal barriers.
        1 = wall, 0 = free.
        """
        n = self.size
        maze = [[0 for _ in range(n)] for _ in range(n)]

        # Outer walls
        for i in range(n):
            maze[0][i] = 1
            maze[n-1][i] = 1
            maze[i][0] = 1
            maze[i][n-1] = 1

        # Simple internal vertical bars with gaps (ensures some structure)
        for row in range(3, n-3, 4):
            gap_col = self.rng.randint(2, n-3)
            for col in range(2, n-2):
                # Don't block start/goal
                if (row, col) in [(1, 1), (n-2, n-2)]:
                    continue
                if col == gap_col:
                    continue
                maze[row][col] = 1

        # Ensure start and goal are free
        sx, sy = 1, 1
        gx, gy = n-2, n-2
        maze[sx][sy] = 0
        maze[gx][gy] = 0

        return maze

    def _is_wall(self, x: int, y: int) -> bool:
        # Boundaries are treated as walls
        if x < 0 or x >= self.size or y < 0 or y >= self.size:
            return True
        return self.maze[x][y] == 1

    def _get_sensor_readings(self) -> Tuple[int, int, int]:
        """
        Return blocked or non-blocked as 0/1 integers.
        This is a simplified approximation of 3pi+ distance readings.
        """
        x, y = self.pos
        ori = self.ori

        def blocked_in_direction(orientation: int) -> int:
            dx, dy = self.ORI_TO_DELTA[orientation]
            nx, ny = x + dx, y + dy
            return int(self._is_wall(nx, ny))

        front_blocked = blocked_in_direction(ori)
        left_blocked = blocked_in_direction((ori - 1) % 4)
        right_blocked = blocked_in_direction((ori + 1) % 4)

        return front_blocked, left_blocked, right_blocked

    def _get_state(self) -> Tuple[int, int, int, int, int, int]:
        """
        State = (x, y, orientation, front_blocked, left_blocked, right_blocked)
          - position
          - orientation
          - distance readings as blocked flags.
        """
        x, y = self.pos
        front_b, left_b, right_b = self._get_sensor_readings()
        return (x, y, self.ori, front_b, left_b, right_b)

    def reset(self) -> Tuple[int, int, int, int, int, int]:
        self.pos = self.start_pos
        self.ori = 0
        self.steps_taken = 0
        return self._get_state()

    def step(self, action: int) -> Tuple[Tuple[int, int, int, int, int, int], float, bool, Dict[str, Any]]:
        """
        Apply an action, return (next_state, reward, done, info).
        which really is just 
          - "invalid": bool (collision / boundary violation)
          - "success": bool (if episode terminated at goal)
        """
        assert action in [0, 1, 2, 3]
        self.steps_taken += 1
        info = {"invalid": False}
        reward = self.step_reward

        x, y = self.pos
        ori = self.ori

        # Compute new orientation & proposed move
        if action == 0:  # forward
            new_ori = ori
            dx, dy = self.ORI_TO_DELTA[new_ori]
            nx, ny = x + dx, y + dy
        elif action == 1:  # backward
            new_ori = ori
            dx, dy = self.ORI_TO_DELTA[new_ori]
            nx, ny = x - dx, y - dy
        elif action == 2:  # hard left: turn left then move forward
            new_ori = (ori - 1) % 4
            dx, dy = self.ORI_TO_DELTA[new_ori]
            nx, ny = x + dx, y + dy
        else:  # action == 3, hard right: turn right then move forward
            new_ori = (ori + 1) % 4
            dx, dy = self.ORI_TO_DELTA[new_ori]
            nx, ny = x + dx, y + dy

        # Check validity of move
        if self._is_wall(nx, ny):
            # Collision / boundary violation
            reward += self.invalid_penalty
            info["invalid"] = True
            # Stay in place, orientation unchanged
            nx, ny = x, y
            new_ori = ori
        else:
            # Move is valid
            self.pos = (nx, ny)
            self.ori = new_ori

        done = False

        # Goal check
        if self.pos == self.goal_pos:
            reward += self.goal_reward
            done = True
            info["success"] = True
        elif self.steps_taken >= self.max_steps:
            # Timeout
            reward += self.timeout_penalty
            done = True
            info["success"] = False

        next_state = self._get_state()
        return next_state, reward, done, info

    def render_ascii(self) -> None:
        """
        Hopefully this allows us to see what the hell the agent is doing becuase omg we need to prove this.
        'S' start, 'G' goal, 'A' agent, '#' wall, '.' free cell.
        """
        grid = [['#' if self.maze[i][j] == 1 else '.' for j in range(self.size)] for i in range(self.size)]
        sx, sy = self.start_pos
        gx, gy = self.goal_pos
        grid[sx][sy] = 'S'
        grid[gx][gy] = 'G'
        ax, ay = self.pos
        grid[ax][ay] = 'A'
        for row in grid:
            print(' '.join(row))
        print()

# Agent parameters (FIRST TIME MAKING THIS SHIT OMG THIS IS COMPLETELY DIFFERENT TO ML AND STATS)
class RLAgent:
    def __init__(
        self,
        num_actions: int = 4,
        alpha: float = 0.2,
        gamma: float = 0.99,
        eps_start: float = 1.0,
        eps_end: float = 0.05,
        eps_decay: float = 0.02,
        algo: str = "q_learning",
    ):
        """
        algo: "q_learning" or "sarsa"
        """
        assert algo in ("q_learning", "sarsa")
        self.num_actions = num_actions
        self.alpha = alpha
        self.gamma = gamma
        self.eps_start = eps_start
        self.eps_end = eps_end
        self.eps_decay = eps_decay
        self.algo = algo
        self.Q: Dict[Any, List[float]] = defaultdict(lambda: [0.0] * num_actions)

    def epsilon(self, episode: int) -> float:
        # Simple exponential decay
        eps = self.eps_end + (self.eps_start - self.eps_end) * (2.71828 ** (-self.eps_decay * episode))
        return max(self.eps_end, eps)

    def select_action(self, state, episode: int, greedy: bool = False) -> int:
        # Greedy-only mode for evaluation
        if not greedy and random.random() < self.epsilon(episode):
            return random.randint(0, self.num_actions - 1)

        q_vals = self.Q[state]
        max_q = max(q_vals)
        # Break ties randomly
        best_actions = [a for a, q in enumerate(q_vals) if q == max_q]
        return random.choice(best_actions)

    def update_q(self, state, action: int, reward: float, next_state, next_action: int = None, done: bool = False):
        current_q = self.Q[state][action]
        if self.algo == "q_learning":
            if done:
                target = reward
            else:
                max_next_q = max(self.Q[next_state])
                target = reward + self.gamma * max_next_q
        else:  # SARSA
            if done:
                target = reward
            else:
                assert next_action is not None
                target = reward + self.gamma * self.Q[next_state][next_action]

        self.Q[state][action] += self.alpha * (target - current_q)

# Training & Evaluation
def run_training(
    algo: str = "q_learning",
    episodes: int = 300,
    max_steps: int = 200,
    verbose: bool = True,
):
    env = MazeEnv(size=25, max_steps=max_steps, seed=42)
    agent = RLAgent(
        num_actions=4,
        alpha=0.2,
        gamma=0.99,
        eps_start=1.0,
        eps_end=0.05,
        eps_decay=0.02,
        algo=algo,
    )

    episode_rewards = []
    episode_lengths = []
    successes = []
    invalid_moves_per_ep = []

    for ep in range(episodes):
        state = env.reset()
        total_reward = 0.0
        invalid_moves = 0

        if algo == "sarsa":
            action = agent.select_action(state, episode=ep)

        for t in range(max_steps):
            if algo == "q_learning":
                action = agent.select_action(state, episode=ep)

            next_state, reward, done, info = env.step(action)
            total_reward += reward
            if info.get("invalid", False):
                invalid_moves += 1

            if algo == "q_learning":
                agent.update_q(state, action, reward, next_state, done=done)
                state = next_state
            else:  # SARSA
                next_action = agent.select_action(next_state, episode=ep)
                agent.update_q(state, action, reward, next_state, next_action=next_action, done=done)
                state, action = next_state, next_action

            if done:
                break

        episode_rewards.append(total_reward)
        episode_lengths.append(t + 1)
        invalid_moves_per_ep.append(invalid_moves)
        successes.append(1 if info.get("success", False) else 0)

        if verbose and (ep + 1) % 10 == 0:
            avg_len = sum(episode_lengths[-10:]) / len(episode_lengths[-10:])
            avg_rew = sum(episode_rewards[-10:]) / len(episode_rewards[-10:])
            avg_succ = sum(successes[-10:]) / len(successes[-10:]) * 100.0
            avg_invalid = sum(invalid_moves_per_ep[-10:]) / len(invalid_moves_per_ep[-10:])
            print(
                f"[{algo}] Episode {ep+1}/{episodes} | "
                f"avg_len (last10)={avg_len:.1f}, "
                f"avg_rew={avg_rew:.2f}, "
                f"success%={avg_succ:.1f}, "
                f"avg_invalid={avg_invalid:.1f}"
            )

    # Evaluation with greedy policy (no exploration)
    eval_episodes = 50
    success_eval = 0
    total_time = 0
    total_invalid_eval = 0

    for ep in range(eval_episodes):
        state = env.reset()
        for t in range(max_steps):
            action = agent.select_action(state, episode=episodes, greedy=True)
            next_state, reward, done, info = env.step(action)
            if info.get("invalid", False):
                total_invalid_eval += 1
            state = next_state
            if done:
                if info.get("success", False):
                    success_eval += 1
                    total_time += t + 1
                break

    success_rate = success_eval / eval_episodes if eval_episodes > 0 else 0.0
    avg_time = total_time / max(success_eval, 1)

    print("\n=== Evaluation Results ===")
    print(f"Algorithm: {algo}")
    print(f"Success rate: {success_rate*100:.1f}% over {eval_episodes} eval episodes")
    print(f"Average time-to-goal (successful episodes): {avg_time:.1f} steps")
    print(f"Total invalid moves during evaluation: {total_invalid_eval}")

    # Rough proxy against project targets
    target_success = 0.9
    if success_rate >= target_success:
        print("Target success rate (~90%) achieved or exceeded.")
    else:
        print("Target success rate NOT achieved yet. Tune hyperparameters or training longer.")

    return agent, env, {
        "episode_rewards": episode_rewards,
        "episode_lengths": episode_lengths,
        "successes": successes,
        "invalid_moves": invalid_moves_per_ep,
    }


if __name__ == "__main__":
    # Run both algorithms for comparison
    print("Training Q-learning agent...")
    q_agent, q_env, q_stats = run_training(algo="q_learning", episodes=300, max_steps=200)

    print("\n\nTraining SARSA agent...")
    sarsa_agent, sarsa_env, sarsa_stats = run_training(algo="sarsa", episodes=300, max_steps=200)
