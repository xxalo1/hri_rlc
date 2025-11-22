import numpy as np
import matplotlib.pyplot as plt
import torch
import math
from ..utils import numpy_util as npu
from ..utils import pytorch_util as ptu
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
        qt   = log["qt"]
        q    = log["q"]
        dqt  = log["dqt"]
        dq   = log["dq"]
        e    = log["e"]
        de   = log["de"]
        v = log["v"]
        taumj = log["taumj"]
        ddqt = log["ddqt"]
        ddq = log["ddq"]
        tau_rnea = log.get("tau_rnea", None)
        tau_diff = log.get("tau_diff", None)
        tau_diff_per = log.get("tau_diff_per", None)
        N, n = q.shape

        plots = [
            ("Joint Positions",       [(qt, "qt"), (q, "q"), (dq, "dq"), (dqt, "dqt")]),
            ("tracking error e and dt with v", [(e, "e"), (de, "de"), (v, "v"), (ddqt, "ddqt")]),
            ("Torque difference", [(tau_diff, "tau_diff")]),
            ("Torque difference percentage", [(tau_diff_per, "tau_diff_per")]),
            ("Joint Accelerations", [(ddq, "ddq"), (ddqt, "ddqt"), (v, "v")]),
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

