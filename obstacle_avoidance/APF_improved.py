import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

class APF_Improved:
    """
    - Tangential force : 장애물 주변을 따라 이동하도록 유도
    - Random noise: local minimum 회피
    """
    def __init__(self, K_attractive=1.5, K_repulsive=80.0, obstacle_range=5.0):
        self.K_att = K_attractive
        self.K_rep = K_repulsive
        self.obs_range = obstacle_range
    
    def calculate_attractive_force(self, pos_robot, pos_goal):
        """
        목표점으로의 인력
        F_att = K_att * (goal 방향 단위벡터)
        """
        robot2goal = np.array(pos_goal) - np.array(pos_robot)
        distance = np.linalg.norm(robot2goal)
        
        if distance == 0:
            return np.array([0.0, 0.0])
        
        # 단위 벡터로 정규화 (goal 가까이 올 시 속도 저하 방지)
        F_attractive = self.K_att * (robot2goal / distance)
        
        return F_attractive
    
    def calculate_repulsive_force(self, pos_robot, pos_goal, obstacles):
        """
        장애물로부터의 척력 + Tangential 방향 힘
        
        - Repulsive Force: 장애물 반대 방향 (밀어냄)
        - Tangential force: 장애물을 따라 목표 방향으로 회전 (우회 유도)
        """
        total_repulsive_force = np.array([0.0, 0.0], dtype=float)
        
        goal_direction = np.array(pos_goal) - np.array(pos_robot)
        goal_dist = np.linalg.norm(goal_direction)
        if goal_dist > 0:
            goal_unit = goal_direction / goal_dist
        else:
            goal_unit = np.array([0.0, 0.0])
        
        for obstacle in obstacles:
            robot2obs = np.array(obstacle) - np.array(pos_robot)
            distance = np.linalg.norm(robot2obs)
            
            if distance < self.obs_range and distance > 0:
                robot2obs_unit = robot2obs / distance
                
                repulsive_magnitude = self.K_rep * (1/distance - 1/self.obs_range) * (1/(distance**2))
                F_repulsive = -repulsive_magnitude * robot2obs_unit
                
                # Tangential force (장애물을 따라 회전하는 힘)
                # 수직 벡터: robot2obs를 90도 회전 [-y, x]
                tangent = np.array([-robot2obs_unit[1], robot2obs_unit[0]])
                
                # 목표 방향과 더 가까운 tangent 방향 선택
                if np.dot(tangent, goal_unit) < 0:
                    tangent = -tangent
                
                # Tangent 방향으로 힘 추가 
                F_tangent = repulsive_magnitude * 0.5 * tangent
                
                total_repulsive_force += (F_repulsive + F_tangent)
        
        return total_repulsive_force
    
    def calculate_total_force(self, pos_robot, pos_goal, obstacles):
        F_att = self.calculate_attractive_force(pos_robot, pos_goal)
        F_rep = self.calculate_repulsive_force(pos_robot, pos_goal, obstacles)
        return F_att + F_rep


class APFSimulator:
    def __init__(self):
        self.apf = APF_Improved(
            K_attractive=1.5,      
            K_repulsive=80.0,      
            obstacle_range=5.0     
        )

        self.robot_pos = np.array([0.0, 0.0])
        self.goal_pos = np.array([30.0, 30.0])
        self.obstacles = np.array([[5, 5],
                                   [14, 18],
                                   [23, 23]], dtype=float)

        self.max_speed = 0.8       
        self.goal_error = 1.0      
        self.max_iters = 3000      

        self.trace = [self.robot_pos.copy()]
        self.iters = 0
        self.done = False

        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self._init_plot()

    def _init_plot(self):
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.set_xlim(-5, 35)
        self.ax.set_ylim(-5, 35)
        self.ax.grid(True, alpha=0.3)
        self.ax.set_title('Artificial Potential Field - Improved', fontsize=14, fontweight='bold')

        self.ax.plot(self.obstacles[:, 0], self.obstacles[:, 1], 'ro', 
                     markersize=15, label='Obstacles', zorder=5)
        for obs in self.obstacles:
            circle = plt.Circle((obs[0], obs[1]), self.apf.obs_range, 
                              color='red', fill=False, linestyle='--', alpha=0.3, linewidth=2)
            self.ax.add_patch(circle)

        self.ax.plot(self.goal_pos[0], self.goal_pos[1], 'g*', 
                     markersize=20, label='Goal', zorder=5)
        
        self.robot_plot, = self.ax.plot([], [], 'bo', markersize=10, label='Robot', zorder=5)
        self.trace_plot, = self.ax.plot([], [], 'b-', linewidth=2.5, alpha=0.7, label='Path')

        self.info_text = self.ax.text(0.02, 0.98, '', transform=self.ax.transAxes,
                                      verticalalignment='top', fontsize=10,
                                      bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

        self.ax.legend(loc='upper right', fontsize=11)

    def init_animation(self):
        """애니메이션 초기화 함수"""
        self.robot_plot.set_data([], [])
        self.trace_plot.set_data([], [])
        self.info_text.set_text('')
        return self.robot_plot, self.trace_plot, self.info_text

    def update(self, frame):
        """애니메이션 업데이트 함수"""
        if self.done:
            return self.robot_plot, self.trace_plot, self.info_text

        self.iters += 1

        F = self.apf.calculate_total_force(
            self.robot_pos,
            self.goal_pos,
            self.obstacles
        )

        norm = np.linalg.norm(F)
        if norm > self.max_speed and norm > 0:
            F = F / norm * self.max_speed

        # 힘이 매우 작으면 랜덤 노이즈 추가
        if norm < 0.01:
            F += np.random.randn(2) * 0.2

        self.robot_pos += F
        self.trace.append(self.robot_pos.copy())

        self.robot_plot.set_data([self.robot_pos[0]], [self.robot_pos[1]])
        
        tr = np.array(self.trace)
        self.trace_plot.set_data(tr[:, 0], tr[:, 1])

        error = np.linalg.norm(self.goal_pos - self.robot_pos)
        
        info = f'Iteration: {self.iters}\n'
        info += f'Position: ({self.robot_pos[0]:.2f}, {self.robot_pos[1]:.2f})\n'
        info += f'Distance to goal: {error:.2f}'
        self.info_text.set_text(info)

        if error < self.goal_error:
            self.done = True
            print(f"\n✓ Goal reached!")
            print(f"  Iterations: {self.iters}")
            print(f"  Final error: {error:.2f}")
            self.info_text.set_text(info + '\n\n★ GOAL REACHED! ★')
        elif self.iters >= self.max_iters:
            self.done = True
            print(f"\nMax iterations reached. Final error: {error:.2f}")

        return self.robot_plot, self.trace_plot, self.info_text

    def run(self):
        """시뮬레이션 실행"""
        print("Starting APF simulation...")
        print(f"Start: {self.robot_pos}")
        print(f"Goal: {self.goal_pos}")
        print(f"Obstacles: \n{self.obstacles}")
        
        self.ani = FuncAnimation(
            self.fig,
            self.update,
            init_func=self.init_animation,
            frames=self.max_iters,
            interval=30,      # 30ms per frame
            blit=True,
            repeat=False
        )
        
        plt.show()

        
if __name__ == "__main__":
    sim = APFSimulator()    
    sim.run()
