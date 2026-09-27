"""
Python dynamic simulation script for the 2-DOF Robotic Manipulator Arm.
Computes open-loop and closed-loop poles, state-feedback pole placement, and simulates step responses.
"""

import sys
if sys.platform.startswith("win"):
    try:
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
        sys.stderr.reconfigure(encoding='utf-8', errors='replace')
    except Exception:
        pass

from pathlib import Path
import numpy as np
import scipy.signal as signal
from scipy.linalg import solve_continuous_are
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# State-Space System Matrices
A = np.array([
    [0.0,      0.0,      1.0, 0.0],
    [0.0,      0.0,      0.0, 1.0],
    [-0.4568, -0.6196,   0.0, 0.0],
    [0.2485,  -6.6174,   0.0, 0.0]
])

B = np.array([
    [0.0,      0.0],
    [0.0,      0.0],
    [0.7870,  -0.0426],
    [0.04285,  0.1349]
])

C = np.array([
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0]
])

D = np.zeros((2, 2))


def analyze_and_simulate(output_dir: Path):
    output_dir.mkdir(parents=True, exist_ok=True)
    print("=" * 65)
    print("   2-DOF Robotic Manipulator Arm: Linear Control Simulation")
    print("=" * 65)

    # 1. Open-loop poles & eigenvalues
    eigenvalues = np.linalg.eigvals(A)
    print("\n[+] Open-Loop System Eigenvalues (Poles):")
    for ev in eigenvalues:
        print(f"    lambda = {ev.real:+.4f} {ev.imag:+.4f}j")

    # Controllability check
    ctrb_matrix = np.hstack([B, A @ B, A @ A @ B, A @ A @ A @ B])
    rank_ctrb = np.linalg.matrix_rank(ctrb_matrix)
    print(f"\n[+] Controllability Matrix Rank: {rank_ctrb} / 4 (System is {'Controllable' if rank_ctrb == 4 else 'Uncontrollable'})")

    # 2. State-Feedback Pole Placement
    # Target poles: -5 +/- 5j, -30, -35
    # Using LQR formulation for robust multi-input gain computation
    Q = np.diag([100.0, 100.0, 10.0, 10.0])
    R = np.eye(2) * 0.1
    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ (B.T @ P)

    print("\n[+] Computed State-Feedback Gain Matrix K:")
    print(np.array2string(K, precision=4, suppress_small=True))

    A_cl = A - B @ K
    cl_poles = np.linalg.eigvals(A_cl)
    print("\n[+] Closed-Loop Poles:")
    for p in cl_poles:
        print(f"    lambda = {p.real:+.4f} {p.imag:+.4f}j")

    # 3. Step Response Simulation
    time_span = np.linspace(0, 5, 1000)
    sys_cl_u1 = signal.StateSpace(A_cl, B[:, [0]], C, D[:, [0]])
    sys_cl_u2 = signal.StateSpace(A_cl, B[:, [1]], C, D[:, [1]])

    t1, y1 = signal.step(sys_cl_u1, T=time_span)
    t2, y2 = signal.step(sys_cl_u2, T=time_span)

    # Plotting Step Response
    fig, axes = plt.subplots(2, 2, figsize=(11, 7))

    axes[0, 0].plot(t1, y1[:, 0], lw=2, color="#1f77b4")
    axes[0, 0].set_title("Input 1 -> Joint 1 Angle (theta_1)")
    axes[0, 0].set_ylabel("Angle (rad)")
    axes[0, 0].grid(True, alpha=0.3)

    axes[0, 1].plot(t1, y1[:, 1], lw=2, color="#ff7f0e")
    axes[0, 1].set_title("Input 1 -> Joint 2 Angle (theta_2) [Cross-Coupling]")
    axes[0, 1].set_ylabel("Angle (rad)")
    axes[0, 1].grid(True, alpha=0.3)

    axes[1, 0].plot(t2, y2[:, 0], lw=2, color="#2ca02c")
    axes[1, 0].set_title("Input 2 -> Joint 1 Angle (theta_1) [Cross-Coupling]")
    axes[1, 0].set_xlabel("Time (s)")
    axes[1, 0].set_ylabel("Angle (rad)")
    axes[1, 0].grid(True, alpha=0.3)

    axes[1, 1].plot(t2, y2[:, 1], lw=2, color="#d62728")
    axes[1, 1].set_title("Input 2 -> Joint 2 Angle (theta_2)")
    axes[1, 1].set_xlabel("Time (s)")
    axes[1, 1].set_ylabel("Angle (rad)")
    axes[1, 1].grid(True, alpha=0.3)

    plt.suptitle("Closed-Loop MIMO Step Response: 2-DOF Robotic Manipulator", fontsize=14, fontweight="bold")
    plt.tight_layout()

    save_fig = output_dir / "closed_loop_step_response.png"
    plt.savefig(save_fig, dpi=300)
    plt.close()
    print(f"\n[+] Closed-loop step response plot exported to: {save_fig.resolve()}")


if __name__ == "__main__":
    out = Path(__file__).parent / "results"
    analyze_and_simulate(out)
