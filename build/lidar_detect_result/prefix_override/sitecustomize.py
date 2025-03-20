import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/wudi/python_files/lidar_detect_result/install/lidar_detect_result'
