# PRL's Spot ROS Driver Docker File

This Dockerfile builds a docker image aimed at running on Spot Core for access to the robot through ROS.

## Accessing Spot Core

Turn on Spot and connect to its WiFi network. Then open a terminal and type:

```console
$ ssh -4 -p 20022 spot@192.168.80.3 -L 21000:127.0.0.1:21000 -fN
$ vncviewer localhost:21000
```
When promted, type the password for the VNC server (you can probably guess it). This will log you in Spot Core.

## Building

We will use the Makefile included with this repo for this:

1. Go to the directory where the Makefile is stored
2. Type 

```console
make .pull 
```
This will create a new directory under your `HOME` directory called `prl_spot/src`.

3. Go to `prl_spot/src`:

```console
cd ~/prl_spot/src`
```

4. Clone the PRL's Spot ROS driver repo:

```console
git clone https://github.com/ImperialCollegeLondon/spot_prl.git
```
5. Open `spot_prl/spot_driver/launch/spot_interface.launch` and update the `username` and `password` fields.

6. Go back to the directory where the Makefile is
7. Run:

```console
make .build
```

## Usage

* Go to the directory where the Makefile is stored.

* Run:

```console
 make body_driver
 ```

Wait until the driver finished loading. You need to enable the motors from Spot first for this to work.

* In a new terminal run 

```console
make keyboard
``` 
Follow the instractions on the screen to control Spot using the keyboars.