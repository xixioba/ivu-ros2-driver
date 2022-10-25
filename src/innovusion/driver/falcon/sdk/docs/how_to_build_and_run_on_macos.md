# How to build and run on macOS 

> This document is a manual for building and running falcon sdk from a fresh MacBook Pro 14

## Environment

- Apple M1 Pro with 10-core CPU, 16-core GPU, 16-core Neural Engine
- macOS Monterey 12.3+

## Build Dependencies

- clang 13.1.6 (clang-1316.0.21.2.5)+
- Homebrew 3.5.6
  - **Boost** stable 1.79.0_1+
  - **Openssl** stable 3.0.5+
  - **Eigen** stable 3.4.0+
  - gnu-tar
  - rsync

## Build Procedure

### Install Dependencies

> All dependencies will be installed in this section, if you have already installed you can skip this section

#### xcode-select

> Refer to  https://mac.install.guide/commandlinetools/4.html

```bash
xcode-select –install
```

#### Homebrew

> Refer to https://brew.sh/

1. Paste in a macOS Terminal 

```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

2. Add **brew** to your **PATH**

```bash
echo 'eval "$(/opt/homebrew/bin/brew shellenv)"' >> /Users/[your-user-name]/.zprofile
eval "$(/opt/homebrew/bin/brew shellenv)"
```

#### Falcon SDK dependencies

> **boost, openssl, eigen** are required，gnu-tar and rsync are tools for packaging related

```bash
brew install boost openssl eigen gnu-tar rsync
```

### Check and Compile

> In this section, the build environment and dependencies will be checked, and then the entire sdk will be compiled

#### Check build-macos-clang.bash

Please check **falcon-lidar-sdk/build/build-macos-clang.bash**, most of the time, we only need to focus on the code enclosed in `##!` blocks, especially

```bash
export BOOST_DIR="/opt/homebrew/Cellar/boost/1.79.0_1"
export OPENSSL_DIR="/opt/homebrew/Cellar/openssl@3/3.0.5"
export EIGEN_DIR="/opt/homebrew/Cellar/eigen/3.4.0_1/include/eigen3"
```

We can use **brew** to see the actual library path on your computer, like

```bash
brew info boost
boost: stable 1.79.0 (bottled), HEAD
Collection of portable C++ source libraries
https://www.boost.org/
/opt/homebrew/Cellar/boost/1.79.0_1 (15,462 files, 465.6MB) *
  Poured from bottle on 2022-07-27 at 21:32:04
From: https://github.com/Homebrew/homebrew-core/blob/HEAD/Formula/boost.rb
License: BSL-1.0
==> Dependencies
Required: icu4c ✔, xz ✔, zstd ✔
==> Options
--HEAD
	Install HEAD version
==> Analytics
install: 152,140 (30 days), 297,441 (90 days), 1,012,540 (365 days)
install-on-request: 36,863 (30 days), 71,101 (90 days), 224,648 (365 days)
build-error: 31 (30 days)
```

If there is an update, we need to match `export BOOST_DIR` and `/opt/homebrew/Cellar/boost/1.79.0_1`, `openssl` and `eigen` are also similar.

#### Compile SDK

> Victory is here, we just need one step!

**Execute the following command(For users and regular developers)**

```bash
./build/build-macos-clang.bash
```

*For those with code repository access

> Once executed, you will see the **innovusion-build-working-dir**  in your user directory.The **source** directory contains the compilation intermediate process files, executable files and link libraries, as well as **inno_lidar_sdk_public.tgz**, which is a clean compressed package that can be moved at will.

```bash
./build/build-macos-clang.bash pipeline public
```

*For those with code repository access and want to **SDK source code delivered**

```bash
./build/build-macos-clang.bash pipeline internal
```

## Run Procedure

> **inno_lidar_sdk_public.tgz** contains executable files and library files, and is generally the basis for anyone to read lidar data and develop software

#### SDK Review

> Let's review inno_lidar_sdk_public, necessary comments have been added

```bash
➜  inno_lidar_sdk_public tree . -L 2
.
├── MAKE_ENV_SETUP.txt
├── Makefile
├── README.md
├── SDK_VERSION
├── apps ###executable files like pcs/inno_pc_server
│   ├── example
│   ├── lidar_util
│   ├── parse
│   └── pcs
├── build
│   └── cpplint.py
├── lib ###application dependent libraries
│   ├── libinnolidarsdk.so -> libinnolidarsdk.so.0
│   ├── libinnolidarsdk.so.0 -> libinnolidarsdk.so.0.0.0
│   ├── libinnolidarsdk.so.0.0.0
│   ├── libinnolidarsdkclient.a
│   ├── libinnolidarsdkclient.so -> libinnolidarsdkclient.so.0
│   ├── libinnolidarsdkclient.so.0 -> libinnolidarsdkclient.so.0.0.0
│   ├── libinnolidarsdkclient.so.0.0.0
│   ├── libinnolidarsdkcommon.a
│   ├── libinnolidarsdkcommon.so -> libinnolidarsdkcommon.so.0
│   ├── libinnolidarsdkcommon.so.0 -> libinnolidarsdkcommon.so.0.0.0
│   ├── libinnolidarsdkcommon.so.0.0.0
│   ├── libinnolidarutils.a
│   ├── libinnolidarutils.so -> libinnolidarutils.so.0
│   ├── libinnolidarutils.so.0 -> libinnolidarutils.so.0.0.0
│   ├── libinnolidarutils.so.0.0.0
│   ├── libinnolidarwsutils.a
│   ├── libinnolidarwsutils.so -> libinnolidarwsutils.so.0
│   ├── libinnolidarwsutils.so.0 -> libinnolidarwsutils.so.0.0.0
│   └── libinnolidarwsutils.so.0.0.0
└── src ###some lidar source and header files
    ├── Makefile
    ├── sdk_client
    ├── sdk_common
    ├── thirdparty
    ├── utils
    └── ws_utils

13 directories, 25 files
```

#### Run inno_pc_server

> inno_pc_server connects and reads the data of one lidar. If you need to read multiple lidars at the same time, you also need to run multiple inno_pc_servers

Terminal One:

> Commands in **[**  **]** are not required

```bash
./inno_pc_server --lidar-ip 172.168.1.10 --lidar-port 8002 [--udp-ip 127.0.0.1 --udp-port 8010 --udp-port-status 8010 --udp-port-message 8010 --udp-port-raw 8010 --udp-port-status-local 8010 --tcp-port 8010]
```

Terminal Two if has another lidar:

> Commands in **[**  **]** are not required

```bash
./inno_pc_server --lidar-ip 172.168.1.9 --lidar-port 8002 [--udp-ip 127.0.0.1 --udp-port 8011 --udp-port-status 8011 --udp-port-message 8011 --udp-port-raw 8011 --udp-port-status-local 8011 --tcp-port 8011]
```

More lidar just require more terminals and modify commands

For more command options

```bash
➜  pcs ./inno_pc_server -h
2022-07-27 16:23:50.288 [ INFO] 0 pcs_main.cpp:26 VERSION: devbuild
2022-07-27 16:23:50.289 [ INFO] 0 pcs_main.cpp:27 BUILD_TAG: LOCAL-BUILD
2022-07-27 16:23:50.289 [ INFO] 0 pcs_main.cpp:28 BUILD_TIME: 11:53:19 Jul 27 2022
2022-07-27 16:23:50.289 [ INFO] 0 pcs_main.cpp:29 API: DEV-internal
2022-07-27 16:23:50.289 [ INFO] 0 pcs_main.cpp:30 API_BUILD_TAG: LOCAL-BUILD
2022-07-27 16:23:50.289 [ INFO] 0 pcs_main.cpp:31 API_BUILD_TIME: 11:53:08 Jul 27 2022
usage: ./inno_pc_server 
	{--lidar-ip <INPUT_LIDAR_IP> [--lidar-port <INPUT_LIDAR_TCP_PORT> --lidar-udp-port <INPUT_LIDAR_UDP_PORT>] [--retry <RETRY_COUNT>] |
	 --file <INPUT_DATA_FILE> [--yaml <INPUT_YAML_FILE>] [--speed <PLAY_SPEED>] [--rewind <REWIND_TIMES>] [--skip <SKIP_IN_MB>]}
	[--falcon-eye <X>,<Y>]
	[--reflectance <REFLECTANCE_MODE>]
	[--multireturn <MULTI_RETURN_MODE>]
	[--lidar-id <LIDAR_ID>]
	[--udp-ip <UDP_DEST_IP> [--udp-port <DATA_UDP_DEST_PORT>]
	  [--udp-port-status <STATUS_UDP_DEST_PORT>]
	  [--udp-port-message <MESSAGE_UDP_DEST_PORT>]]
	  [--udp-port-raw <RAW_UDP_DEST_PORT>]]
	[--udp-port-status-local <LOCAL_STATUS_UDP_DEST_PORT>]
	[--tcp-port <TCP_LISTEN_PORT>]
	[--status-interval-ms <INTERVAL_IN_MS>]
	[--record-inno-pc-filename <RECORD_INNO_PC_FILE>
	  [--record-inno-pc-size-in-m <RECORD_INNO_PC_FILE_SIZE>]
	  [--inno-pc-record-npy]]
	[--record-png-filename <RECORD_PNG_FILE>
	[--record-rosbag-filename <RECORD_ROSBAG_FILE>
	  [--record-rosbag-size-in-m <RECORD_ROSBAG_SIZE>]]
	[--record-raw-filename <RECORD_RAW_FILE>
	  [--record-raw-size-in-m <RECORD_RAW_FILE_SIZE>]]
	[--config <CONFIG_FILE>]
	[--config2 <CONFIG_FILE2>]
	[--dtc <DTC_FILE>]
	[--out-format <OUT_FORMAT>]
	[--processed]
	[--debug <DEBUG_LEVEL>] [-v] [-h] [--quiet]
	[--log-filename <LOG_FILE>] [--log-file-rotate-number <ROTATE_NUMBER>] [--log-file-max-size-k <MAX_LOG_FILE_SIZE_IN_K>]
	[--error-log-filename <ERROR_LOG_FILE>] [--error-log-file-rotate-number <ROTATE_NUMBER>] [--error-log-file-max-size-k <MAX_ERROR_LOG_FILE_SIZE_IN_K>]
	[--show-viewer]
For the --falcon-eye option,
	x: ROI horizontal center (in [-60, 60] degrees)
	y: ROI vertical center (in [-25, 25], degrees)
	[--test-command <TEST_COMMAND>] [--run-time-s <RUN_TIME_IN_SECONDS>] [--test-command-interval-ms <TEST_COMMAND_INTERVAL_IN_MS>]
```

## Performance overview

> This section gives a basic cpu usage. Since the lidar can be used as a server and parse the original data by itself, it can be selected as server or client on the macos side

| Mode              | Server(port 8002, lidar server disabled) | Client(default 8010) |
| :---------------- | :--------------------------------------: | :------------------: |
| Cpu (single core) |                   <50%                   |         <30%         |

## Debug in vscode

### launch.json

```c
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "(gdb) start",
            "type": "lldb",
            "request": "launch",
            "program": "***path***/pcs/inno_pc_server",
            "args": [
                "--lidar-ip",
                "172.168.1.10",
                "--lidar-port",
                "8002"
            ],
            "cwd": "${workspaceFolder}",
        }
    ]
}
```
