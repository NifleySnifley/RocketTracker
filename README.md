# Seeker GUI

The seeker GUI for the rocket tracker is written in Rust and mainly designed for running on a raspberry pi



## Building

Due to the fact that Raspberry Pis aren't especially powerul and the large amount of dependencies this project uses, its much more convenient to cross-compile the application on a development machine, and deploy it to the "target" Pi remotely.

### Prerequisites:

- Linux/Unix system (or WSL)
  
  - `rsync`
  
  - `ssh`
  
  - GNU Make
  
  - `cmake` (required for building the protobuf compiler)

- Rustup + Cargo (Rust toolchain)

- `cross` crate
  
  - Can be installed simply by running
    
    ```bash
    cargo install cross --git https://github.com/cross-rs/cross`
    ```

- Docker (used by `cross` as a build environment)

### Compilation & Deployment

Compilation and deployment is handled by the makefile, which provides 5 targets:

- `all` - Builds and deploys the application to the target

- `dev` - Builds, deploys, and runs the application on the target

- `build` - Compiles the source using `cross` for `arm-unknown-linux-gnueabihf`

- `exec_remote` - Runs the application on the target
  
  - The xorg display can be configured in [remotescript.sh](./remotescript.sh), which can be useful for X11 forwarding during development

- `deploy` - Copies the compiled executable (and [remotescript.sh](./remotescript.sh)) to the target

 When deploying code, the IP address of the target machine can be specified along with the make command using `TARGET_HOST` ex:`make all TARGET_HOST=pi@192.168.0.0`. It's often also neccesary to specify the copying destination using `TARGET_PATH` (such as `make all TARGET_HOST=pi@192.168.0.0 TARGET_PATH=/home/pi/Documents`). It is reccomended to setup key-only SSH authentication for the target machine (for more seamless automated deployment)


