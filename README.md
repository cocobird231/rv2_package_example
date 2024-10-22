# RV2 Package Example
This is an example package for RV2 Package Launcher Management.

## File Tree
```plaintext
rv2_package_example
├── include/rv2_package_example
│   └── ...
├── launch
│   └── launch.py
├── params
│   ├── params.yaml
│   └── system.yaml (optional)
├── scripts
│   ├── custom.sh (optional)
│   ├── script_after_build.sh (optional)
│   ├── script_before_build.sh (optional)
│   └── source_env.sh (optional)
├── src
│   └── ...
├── CMakeLists.txt
├── package.xml
├── requirements_apt.txt (optional)
├── requirements_pip.txt (optional)
└── ...
```

## `launch` Directory
This directory contains launch files for the package. The launch file should contains the `params_file` or `config_file` argument to load the parameters.

## `params` Directory
This directory contains the parameters for the package. The parameters are stored in YAML format and can be loaded using the `params_file` or `config_file` argument in the launch file.

The files under this directory will be copied to the package launcher directory.

- `params.yaml`: The parameters file for the package.
    ```yaml
    /**:
        ros__parameters:
            param1: "val1"
    ```
    - `/**`: The namespace for the parameters.
    - `ros__parameters`: The ROS parameters format.
    - `param1`: The parameter name and value.

    **NOTE**: The file can be directly loaded using `ros2 run` command.

- `system.yaml`: The system parameters to create the service file.
    ```yaml
    launch:
        params: "params.yaml"
        launch: "launch.py"

    network:
        interface: "eno1"
        internet_required: false

    custom:
        arg1: "val1"
    ```
    - `launch`: The required parameters to create `runfile.sh` for systemd service.
        - `params`: The name of the parameters file under `params` directory.
        - `launch`: The name of the launch file under `launch` directory.

    - `network`: The network parameters to create `runfile.sh` and service file.
        - `interface`: The network interface to use. The parameter will be added to the service file.
        - `internet_required`: The flag to check the internet connection. If `true`, the internet connection check will be added to the `runfile.sh`.

    - `custom`: The custom parameters for `custom.sh` script.

## `scripts` Directory
This directory contains scripts that can be used to create service, runfile and build the package.

- `custom.sh`: Custom script that can be run under `create-service` command before runfile.sh created, before service file created.

    The `custom.sh` will be run with sudo permission, and passes the `system.yaml` file path and `package path` as arguments.

    **NOTE**: The `system.yaml` file path indicates the file under package launcher directory.

    **NOTE**: The `package path` is the path of the package under the ROS2 workspace.

- `script_after_build.sh`: Script that can be run after the `build` command.

    The `script_after_build.sh` will be run with sudo permission, and passes the `package path` as an argument.

- `script_before_build.sh`: Script that can be run before the `build` command.

    The `script_before_build.sh` will be run with sudo permission, and passes the `package path` as an argument.

- `source_env.sh`: Script that will be added to the `runfile.sh` to source the environment.

    **NOTE**: The script will be added before the `ros2 launch` command.

## Requirements Files
The requirements files are used to install the dependencies for the package.

- `requirements_apt.txt`: The APT packages to install using `apt install` command.

- `requirements_pip.txt`: The Python packages to install using `python3 -m pip install` command.
