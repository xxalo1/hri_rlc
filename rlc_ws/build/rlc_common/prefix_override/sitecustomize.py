import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rwolv/projects/hri_rlc/rlc_ws/install/rlc_common'
