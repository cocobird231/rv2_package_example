# This is an optional script that will be executed under `create-service` command in the `rv2_startup` script.
# The script will be executed before the `runfile.sh` is created, before the service file is created.

#!/bin/bash

echo "[custom]"

# The argument $1 should be the system.yaml file path.
# The argument $2 should be the repo path under ROS2 workspace source directory.
echo ${@}
