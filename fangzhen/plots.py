from __future__ import annotations

from typing import Dict
import numpy as np
import matplotlib.pyplot as plt


def plot_results(logs: Dict[str, np.ndarray]) -> None:
    t = logs["t"]
    angles = logs["angles"]
    angles_sp = logs["angles_sp"]
    pos = logs["pos"]
    z_sp = logs["z_sp"]
    thrust = logs["thrust_cmd"]
    tau = logs["tau_cmd"]

    fig, axs = plt.subplots(4, 1, figsize=(10, 12), sharex=True)

    axs[0].plot(t, np.rad2deg(angles[:, 0]), label="phi")
    axs[0].plot(t, np.rad2deg(angles[:, 1]), label="theta")
    axs[0].plot(t, np.rad2deg(angles[:, 2]), label="psi")
    axs[0].plot(t, np.rad2deg(angles_sp[:, 0]), "--", label="phi_sp")
    axs[0].plot(t, np.rad2deg(angles_sp[:, 1]), "--", label="theta_sp")
    axs[0].plot(t, np.rad2deg(angles_sp[:, 2]), "--", label="psi_sp")
    axs[0].set_ylabel("Angles (deg)")
    axs[0].legend(loc="best")
    axs[0].grid(True, alpha=0.3)

    axs[1].plot(t, pos[:, 2], label="z")
    axs[1].plot(t, z_sp, "--", label="z_sp")
    axs[1].set_ylabel("Altitude z (m)")
    axs[1].legend(loc="best")
    axs[1].grid(True, alpha=0.3)

    axs[2].plot(t, thrust, label="Total Thrust (N)")
    axs[2].set_ylabel("Thrust (N)")
    axs[2].legend(loc="best")
    axs[2].grid(True, alpha=0.3)

    axs[3].plot(t, tau[:, 0], label="tau_x")
    axs[3].plot(t, tau[:, 1], label="tau_y")
    axs[3].plot(t, tau[:, 2], label="tau_z")
    axs[3].set_ylabel("Torques (N*m)")
    axs[3].set_xlabel("Time (s)")
    axs[3].legend(loc="best")
    axs[3].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()


