from pathlib import Path
import logging

from ..utils import pytorch_util as ptu
from ..utils import numpy_util as npu
from .app_gen3 import Gen3App

ptu.init_gpu(use_gpu=False)

HERE = Path(__file__).parent
XML_PATH = HERE / "../world/world.xml"
XML_PATH_2 = HERE / "../world/scene.xml"

# setup .log file logging
LOG_DIR = HERE / "logs"
LOG_DIR.mkdir(exist_ok=True)
log_path = LOG_DIR / f"env_run.log"
logger = logging.getLogger("env_run")
logger.setLevel(logging.INFO)
# avoid duplicate handlers if main() re-runs
logger.handlers.clear()
fh = logging.FileHandler(log_path, mode="w")
fh.setFormatter(logging.Formatter("%(asctime)s %(message)s"))
logger.addHandler(fh)
logger.propagate = False


app = Gen3App(XML_PATH, logger)
app.run()