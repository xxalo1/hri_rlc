import numpy as np
import matplotlib.pyplot as plt
import torch
import math
from . import numpy_util as npu
from . import pytorch_util as ptu
FloatArray = npu.FloatArray

class Plotter():

    def __init__(self):
        self._log = {}


    def log(self, 
        key: str, 
        value: torch.Tensor | FloatArray
    ) -> None:
        """Log a value under the specified key."""
        if key in self._log:
            if isinstance(value, np.ndarray):
                self._log[key].append(value.copy())
            elif isinstance(value, torch.Tensor):
                value = ptu.to_numpy(value)
                self._log[key].append(value)
            else: 
                self._log[key].append(value)
        else:
            if isinstance(value, np.ndarray):
                self._log[key] = [value.copy()]
            elif isinstance(value, torch.Tensor):
                value = ptu.to_numpy(value)
                self._log[key] = [value]
            else: 
                self._log[key] = [value]


    def get_log(self) -> dict[str, FloatArray]:
        """Retrieve the logged data as numpy arrays."""
        return {key: np.array(values) for key, values in self._log.items()}


    def clear_log(self) -> None:
        """Clear all recorded data."""
        for k in self._log:
            self._log[k].clear()


    def draw_plots(self):
        log = self.get_log()
        t    = log["t"]
        q_des   = log["q_des"]
        q    = log["q"]
        qd_des  = log["qd_des"]
        qd   = log["qd"]
        e    = log["e"]
        de   = log["de"]
        v = log["v"]
        taumj = log["taumj"]
        qdd_des = log["qdd_des"]
        qdd = log["qdd"]
        tau_rnea = log.get("tau_rnea", None)
        tau_diff = log.get("tau_diff", None)
        tau_diff_per = log.get("tau_diff_per", None)
        N, n = q.shape

        plots = [
            ("Joint Positions",       [(q_des, "q_des"), (q, "q"), (qd, "qd"), (qd_des, "qd_des")]),
            ("tracking error e and dt with v", [(e, "e"), (de, "de"), (v, "v"), (qdd_des, "qdd_des")]),
            ("Torque difference", [(tau_diff, "tau_diff")]),
            ("Torque difference percentage", [(tau_diff_per, "tau_diff_per")]),
            ("Joint Accelerations", [(qdd, "qdd"), (qdd_des, "qdd_des"), (v, "v")]),
        ]

        cols = 3                                # change to 2 if you want
        rows = math.ceil(n / cols)              # auto rows based on n

        for title, series in plots:
            fig, axes = plt.subplots(
                rows, cols,
                figsize=(cols*5, rows*3),
                sharex=True
            )

            axes = axes.flatten()               # make indexing simple
            fig.suptitle(title)

            for j in range(n):
                ax = axes[j]
                for arr, label in series:

                    if label:
                        ax.plot(t, arr[:, j], label=label)
                    else:
                        ax.plot(t, arr[:, j])
                ax.set_ylabel(f"J{j+1}")
                if any(label is not None for _, label in series):
                    ax.legend()

            # turn off any unused subplot
            for k in range(n, len(axes)):
                axes[k].axis("off")

            axes[-1].set_xlabel("time [s]")

        plt.show()

