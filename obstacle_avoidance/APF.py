import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class APF:
    def __init__(self, K_attractive=0.02, K_repulsive=3.0, obstacle_range=2):
        self.K_att = K_attractive
        self.K_rep = K_repulsive
        self.obs_range = obstacle_range
    
    def calculate_attractive_force(self, pos_robot, pos_goal):
        robot2goal = np.array(pos_goal) - np.array(pos_robot)
        distance = np.linalg.norm(robot2goal)
        if distance == 0:
            return np.array([0.0, 0.0])
        
        F_attractive = self.K_att * robot2goal
        
        return F_attractive
    
    def calculate_repulsive_force(self, pos_robot, obstacles):
        total_repulsive_force = np.array([0.0,0.0], dtype=float)
        for obstacle in obstacles:
            robot2obs = np.array(obstacle) - np.array(pos_robot)
            distance = np.linalg.norm(robot2obs)
            
            if distance < self.obs_range and distance > 0:
                robot2obs_unit = robot2obs/distance
                # F_rep = K_rep * (1/distance - 1/obs_range) * (1/d^2) * (-unit vector) : 장애물 반대방향으로 힘이 적용되어야 함
                repulsive_magnitude = self.K_rep*(1/distance - 1/self.obs_range)*(1/(distance**2))
                F_repulsive = -repulsive_magnitude*robot2obs_unit
                total_repulsive_force += F_repulsive
        
        return total_repulsive_force
    
    def calculate_total_force(self, pos_robot, pos_goal, obstacles):
        F_att = self.calculate_attractive_force(pos_robot, pos_goal)
        F_rep = self.calculate_repulsive_force(pos_robot, obstacles)
        return F_att + F_rep


class APFSimulator:
    def __init__(self):
        self.apf = APF(K_attractive=0.02, K_repulsive=30, obstacle_range=2)

        self.robot_pos = np.array([0.0, 0.0])
        self.goal_pos = np.array([30.0, 30.0])
        self.obstacles = np.array([[7, 6],
                                   [14, 16],
                                   [23, 22]], dtype=float)

        self.max_speed = 0.8
        self.goal_error = 1.0
        self.max_iters = 2000

        self.trace = [self.robot_pos.copy()]
        self.iters = 0
        self.done = False

        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self._init_plot()

    def _init_plot(self):
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.set_xlim(-5, 35)
        self.ax.set_ylim(-5, 35)
        self.ax.grid(True)
        self.ax.set_title('Artificial Potential Field Simulation')
        
        self.ax.plot(self.obstacles[:, 0], self.obstacles[:, 1], 'ro', 
                     markersize=15, label='obstacles')
        for obs in self.obstacles:
            circle = plt.Circle((obs[0], obs[1]), self.apf.obs_range, 
                              color='red', fill=False, linestyle='--', alpha=0.3, linewidth=2)
            self.ax.add_patch(circle)

        self.ax.plot(self.goal_pos[0], self.goal_pos[1], 'g*', 
                     markersize=20, label='goal', zorder=5)
        
        self.robot_plot, = self.ax.plot([], [], 'bo', markersize=10, label='robot')
        self.trace_plot, = self.ax.plot([], [], 'b-', linewidth=2.5, alpha=0.7, label='trace')

        self.ax.legend(loc='upper right', fontsize=11)
        

    def init_animation(self):
        """애니메이션 초기화 함수"""
        self.robot_plot.set_data([], [])
        self.trace_plot.set_data([], [])
        return self.robot_plot, self.trace_plot

    def update(self, frame):
        """애니메이션 업데이트 함수"""
        if self.done:
            return self.robot_plot, self.trace_plot

        self.iters += 1

        F = self.apf.calculate_total_force(
            self.robot_pos,
            self.goal_pos,
            self.obstacles
        )

        norm = np.linalg.norm(F)
        if norm > self.max_speed and norm > 0:
            F = F / norm * self.max_speed

        self.robot_pos += F
        self.trace.append(self.robot_pos.copy())

        self.robot_plot.set_data([self.robot_pos[0]], [self.robot_pos[1]])
        
        tr = np.array(self.trace)
        self.trace_plot.set_data(tr[:, 0], tr[:, 1])

        error = np.linalg.norm(self.goal_pos - self.robot_pos)
        if error < self.goal_error:
            self.done = True
            print(f"\n✓ Goal reached!")
            print(f"  Iterations: {self.iters}")
            print(f"  Final error: {error:.2f}")
        elif self.iters >= self.max_iters:
            self.done = True
            print(f"Max iterations reached. Final error: {error:.2f}")

        return self.robot_plot, self.trace_plot

    def run(self):
        """시뮬레이션 실행"""
        self.ani = FuncAnimation(
            self.fig,
            self.update,
            init_func=self.init_animation,
            frames=self.max_iters,
            interval=30,  # 50ms 
            blit=True,
            repeat=False
        )
        plt.show()

        
if __name__=="__main__":
    sim = APFSimulator()
    sim.run()
        