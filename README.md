### OPENVR PLUGIN FOR AMBF
Finally run AMBF in SteamVR. Current configuration supports Meta Quest Headsets, ALVR, and Ubuntu 22.04 or 24.04 (because of ALVR support). More information coming soon!!!

## Compile and Build
```bash
cd <path to this directory/>
mkdir build && cd build
cmake ..
make
```

The make command will produce two library (.so) file, one is an object plugin and the other is a simulator plugin.

## Run
The OpenVR Plugin is already added to the `camera.yaml` file. Please check the file to inspect its contents.

Make sure `ambf_simulator` is aliased, or head to the directory where `ambf_simulator` is located. On Linux, this is `ambf/bin/lin-x86_64/`. Replace the `<path to this directory>` in the commands below with the **absolute** or **relative** location of this directory.

```bash
ambf_simulator -a <path to this directory>/ADF/camera.yaml -l1
```

The `-l1` flag can be changed to load any other ADF file. 
