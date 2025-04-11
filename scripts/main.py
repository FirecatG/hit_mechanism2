import matplotlib.pyplot as plt
from cam import Cam, moveType
import numpy as np
def main():
    cam = Cam(
        h=55,
        phi_1=130,
        alpha_1=30,
        phi_2=135,
        alpha_2=65,
        phi_s1=25,
        phi_s2=70,
        omega_1=30,
        e=10,
        r_0=10, 
        rise_move_type=moveType.CONSTANT,  
        return_move_type=moveType.COS_ACCELERATION  
    )
    results = cam.calculate()
    cam.plot_results(results)

if __name__ == "__main__":
    main()
    