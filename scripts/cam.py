import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rcParams
from enum import Enum


class moveType(Enum):
    SIN_ACCELERATION = 1
    COS_ACCELERATION = 2
    LINEAR = 3

class Cam:
    def __init__(self, h, phi_1, alpha_1, phi_2, alpha_2, phi_s1, phi_s2, omega_1, e=10, r_0=10, 
                 rise_move_type=moveType.SIN_ACCELERATION, return_move_type=moveType.COS_ACCELERATION):
        self.h = h
        self.phi_1 = phi_1
        self.alpha_1 = alpha_1
        self.phi_2 = phi_2
        self.alpha_2 = alpha_2
        self.phi_s1 = phi_s1
        self.phi_s2 = phi_s2
        self.omega_1 = omega_1
        self.e = e
        self.r_0 = r_0
        self.rise_move_type = rise_move_type
        self.return_move_type = return_move_type

    def calculate(self):
        varphi_range = np.arange(0, 361, 1)
        num_points = len(varphi_range)
        
        s_points = np.zeros(num_points)
        v_points = np.zeros(num_points)
        a_points = np.zeros(num_points)
        ds_dvarphi_points = np.zeros(num_points)
        r_0_points = np.zeros(num_points)
        alpha_points = np.zeros(num_points)
        rho_points = np.zeros(num_points)
        x_points = np.zeros(num_points)
        y_points = np.zeros(num_points)
        dx_points = np.zeros(num_points)
        dy_points = np.zeros(num_points)
        x_1_points = np.zeros(num_points)
        y_1_points = np.zeros(num_points)

        for idx in range(num_points):
            varphi = varphi_range[idx]
            
            if varphi <= self.phi_1:
                if self.rise_move_type == moveType.SIN_ACCELERATION:
                    s = self.h * (varphi / self.phi_1 - np.sin(2 * np.pi * varphi / self.phi_1) / (2 * np.pi))
                    v = self.h * self.omega_1 * (1 - np.cos(2 * np.pi * varphi / self.phi_1)) / self.phi_1
                    a = 2 * np.pi * self.h * self.omega_1**2 * np.sin(2 * np.pi * varphi / self.phi_1) / self.phi_1**2
                elif self.rise_move_type == moveType.COS_ACCELERATION:
                    s = self.h / 2 * (1 - np.cos(np.pi * varphi / self.phi_1))
                    v = np.pi * self.h * self.omega_1 / 2 / self.phi_1 * np.sin(np.pi * varphi / self.phi_1)
                    a = np.pi ** 2 * self.h * self.omega_1**2 / 2 / self.phi_1**2 * np.cos(np.pi * varphi / self.phi_1)
                
            elif varphi >= self.phi_1 + self.phi_s1 and varphi <= self.phi_1 + self.phi_s1 + self.phi_2:

                if self.return_move_type == moveType.SIN_ACCELERATION:
                    T = varphi - (self.phi_1 + self.phi_s1)
                    s = self.h * (1 - (T / self.phi_2) + np.sin(np.pi * 2 * T / self.phi_1) / 2 / np.pi) 
                    v = -1 * self.h * self.omega_1 / self.phi_2 * (1 - np.cos(np.pi * 2 * T / self.phi_2))
                    a = -2 * np.pi * self.h * self.omega_1**2 / self.phi_2**2 * np.sin(np.pi * 2 * T / self.phi_2)
                elif self.return_move_type == moveType.COS_ACCELERATION:
                    s = self.h * (1 + np.cos(np.pi * (varphi - (self.phi_1 + self.phi_s1)) / self.phi_2)) / 2
                    v = -1 * np.pi * self.h * self.omega_1 / 2 / self.phi_2 * np.sin(np.pi * (varphi - (self.phi_1 + self.phi_s1)) / self.phi_2)
                    a = -1 * np.pi * np.pi * self.h * self.omega_1**2 / 2 / self.phi_2**2 * np.cos(np.pi * (varphi - (self.phi_1 + self.phi_s1)) / self.phi_2)

            elif varphi > self.phi_1 and varphi < self.phi_1 + self.phi_s1:
                s = self.h
                v = 0
                a = 0
            elif varphi > self.phi_1 + self.phi_s1 + self.phi_2 and varphi <= 360:
                s = 0
                v = 0
                a = 0

            s_points[idx] = s
            v_points[idx] = v
            a_points[idx] = a

            ds_dvarphi = v / self.omega_1
            ds_dvarphi_points[idx] = ds_dvarphi

            if varphi <= self.phi_1:
                alpha = self.alpha_1
            elif varphi >= self.phi_1 + self.phi_s1 and varphi <= self.phi_1 + self.phi_s1 + self.phi_2:
                alpha = self.alpha_2

            alpha_points[idx] = alpha

            compare = 0
            temp_r_0 = self.e + 1  # 使用临时变量存储 r_0
            while compare >= 0:
                s_0 = np.sqrt(temp_r_0**2 - self.e**2)
                compare = abs(ds_dvarphi - self.e) / (s_0 + s) - np.tan(np.deg2rad(alpha))
                temp_r_0 += 1
            r_0_points[idx] = temp_r_0

        self.r_0 = np.max(r_0_points)
        s_0 = np.sqrt(self.r_0**2 - self.e**2)

        for idx in range(num_points):
            varphi = varphi_range[idx]
            s = s_points[idx]
            ds_dvarphi = ds_dvarphi_points[idx]
            dds_ddvarphi = a_points[idx] / self.omega_1

            alpha = np.degrees(np.arctan(abs(ds_dvarphi - self.e) / (s_0 + s)))
            alpha_points[idx] = alpha

            x = -(s_0 + s) * np.sin(np.radians(varphi)) - self.e * np.cos(np.radians(varphi))
            y = (s_0 + s) * np.cos(np.radians(varphi)) - self.e * np.sin(np.radians(varphi))
            dx = -(s_0 + s) * np.cos(np.radians(varphi)) - (ds_dvarphi - self.e) * np.sin(np.radians(varphi))
            dy = -(s_0 + s) * np.sin(np.radians(varphi)) + (ds_dvarphi - self.e) * np.cos(np.radians(varphi))
            ddx = -(2 * ds_dvarphi - self.e) * np.cos(np.radians(varphi)) - (dds_ddvarphi - s_0 - s) * np.sin(np.radians(varphi))
            ddy = (dds_ddvarphi - s_0 - s) * np.cos(np.radians(varphi)) - (2 * ds_dvarphi - self.e) * np.sin(np.radians(varphi))
            rho = (dx**2 + dy**2)**1.5 / (dx * ddy - dy * ddx)
            rho_points[idx] = rho

            x_points[idx] = x
            y_points[idx] = y
            dx_points[idx] = dx
            dy_points[idx] = dy

        rho = np.min(rho_points)
        r_r = np.floor(rho / 2)

        for idx in range(num_points):
            varphi = varphi_range[idx]
            x = x_points[idx]
            y = y_points[idx]
            dx = dx_points[idx]
            dy = dy_points[idx]
            x_1 = x + r_r * dy / np.sqrt(dx**2 + dy**2)
            y_1 = y - r_r * dx / np.sqrt(dx**2 + dy**2)
            x_1_points[idx] = x_1
            y_1_points[idx] = y_1

        return {
            'varphi_range': varphi_range,
            's_points': s_points,
            'v_points': v_points,
            'a_points': a_points,
            'ds_dvarphi_points': ds_dvarphi_points,
            'alpha_points': alpha_points,
            'rho_points': rho_points,
            'x_points': x_points,
            'y_points': y_points,
            'x_1_points': x_1_points,
            'y_1_points': y_1_points
        }

    def plot_results(self, results):
        varphi_range = results['varphi_range']
        s_points = results['s_points']
        v_points = results['v_points']
        a_points = results['a_points']
        ds_dvarphi_points = results['ds_dvarphi_points']
        alpha_points = results['alpha_points']
        rho_points = results['rho_points']
        x_points = results['x_points']
        y_points = results['y_points']
        x_1_points = results['x_1_points']
        y_1_points = results['y_1_points']

        fig, axs = plt.subplots(4, 2, figsize=(12, 16))

        # 绘制位移曲线
        axs[0, 0].plot(varphi_range, s_points, 'r-', linewidth=1.5)
        axs[0, 0].set_xlabel(r'varphi (°)', fontsize=10)
        axs[0, 0].set_ylabel('s (mm)', fontsize=10)
        axs[0, 0].set_title('s(varphi)', fontsize=12)
        axs[0, 0].grid(True)

        # 绘制速度曲线
        axs[0, 1].plot(varphi_range, v_points, 'r-', linewidth=1.5)
        axs[0, 1].set_xlabel(r'varphi (°)', fontsize=10)
        axs[0, 1].set_ylabel('v (mm/s)', fontsize=10)
        axs[0, 1].set_title('v(varphi)', fontsize=12)
        axs[0, 1].grid(True)

        # 绘制加速度曲线
        axs[1, 0].plot(varphi_range, a_points, 'r-', linewidth=1.5)
        axs[1, 0].set_xlabel(r'varphi (°)', fontsize=10)
        axs[1, 0].set_ylabel('a (mm²/s)', fontsize=10)
        axs[1, 0].set_title('a(varphi)', fontsize=12)
        axs[1, 0].grid(True)

        # 绘制位移导数曲线
        axs[1, 1].plot(s_points, ds_dvarphi_points, 'r-', linewidth=1.5)
        axs[1, 1].set_xlabel('s (mm)', fontsize=10)
        axs[1, 1].set_ylabel('ds/dvarphi (mm/°)', fontsize=10)
        axs[1, 1].set_title('ds/dvarphi(s)', fontsize=12)
        axs[1, 1].grid(True)

        # 绘制压力角曲线
        axs[2, 0].plot(varphi_range, alpha_points, 'r-', linewidth=1.5)
        axs[2, 0].set_xlabel(r'varphi (°)', fontsize=10)
        axs[2, 0].set_ylabel('alpha (°)', fontsize=10)
        axs[2, 0].set_title('alpha(varphi)', fontsize=12)
        axs[2, 0].grid(True)

        # 绘制曲率半径曲线
        axs[2, 1].plot(varphi_range, rho_points, 'r-', linewidth=1.5)
        axs[2, 1].set_xlabel(r'varphi (°)', fontsize=10)
        axs[2, 1].set_ylabel('rho (mm)', fontsize=10)
        axs[2, 1].set_title('rho(varphi)', fontsize=12)
        axs[2, 1].grid(True)

        # 绘制理论轮廓曲线
        axs[3, 0].plot(x_points, y_points, 'r-', linewidth=1.5)
        axs[3, 0].set_xlabel('x (mm)', fontsize=10)
        axs[3, 0].set_ylabel('y (mm)', fontsize=10)
        axs[3, 0].set_title('theoretical profile', fontsize=12)
        axs[3, 0].grid(True)

        # 绘制实际轮廓曲线
        axs[3, 1].plot(x_1_points, y_1_points, 'r-', linewidth=1.5)
        axs[3, 1].set_xlabel('x (mm)', fontsize=10)
        axs[3, 1].set_ylabel('y (mm)', fontsize=10)
        axs[3, 1].set_title('actual profile', fontsize=12)
        axs[3, 1].grid(True)

        plt.tight_layout()
        plt.show()

