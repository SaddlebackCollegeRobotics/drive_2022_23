import os
from ament_index_python.packages import get_package_share_directory

package_name = 'driver'

configFile = os.path.dirname(os.path.realpath(__file__)) + "/gamepads.config"
print(configFile)


# print(os.path.join(get_package_share_directory(package_name)))