import matplotlib.pyplot as plt
from cam import Cam, moveType
def main():
    cam = Cam(
        h=60,
        phi_1=125,
        alpha_1=30,
        phi_2=125,
        alpha_2=65,
        phi_s1=60,
        phi_s2=50,
        omega_1=30,
        e=10,
        r_0=10,
        rise_move_type=moveType.SIN_ACCELERATION,  # 推程选择正弦加速度
        return_move_type=moveType.SIN_ACCELERATION  # 回程选择余弦加速度
    )
    results = cam.calculate()
    cam.plot_results(results)
if __name__ == "__main__":
    main()