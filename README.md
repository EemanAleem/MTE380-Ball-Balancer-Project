# MTE380-Ball-Balancer-Project

## Setup
This setup assumes the IDE being used is the VSCode IDE to run this program.

### Windows:
1. Download a CMake binary distribution from here: https://cmake.org/download/. 
   Click "Add to PATH" upon installation.
2. Download OpenCV from here: https://opencv.org/releases/.
3. Copy the path to your opencv/build folder. As an example: `C:/Users/Eeman/opencv/build`. Go to "Edit environment variables on your account" and add a 'OpenCV_DIR' to it with the path to your opencv/build folder.

### Linux (Debian/Ubuntu)
Install the compiler, CMake and OpenCV:
```bash
sudo apt update
sudo apt install build-essential cmake libopencv-dev
```
OpenCV installs to a standard location, so no `OpenCV_DIR` is needed.
(Other distros: Fedora uses `sudo dnf install gcc-c++ cmake opencv-devel`.)

### Macbook:
1. Install the Xcode command line tools (provides the compiler):
   ```bash
   xcode-select --install
   ```
2. Install Homebrew if you don't have it: https://brew.sh
3. Install CMake and OpenCV:
   ```bash
   brew install cmake opencv
   ```
4. If CMake can't find OpenCV, set `OpenCV_DIR` and add the line to `~/.zshrc`
   so it persists:
   ```bash
   export OpenCV_DIR="$(brew --prefix opencv)/lib/cmake/opencv4"
   ```
   Then restart VSCode and any open terminals.
5. The first time the program accesses the camera, macOS will ask for permission.
   Grant it to the app you launch from (VSCode or Terminal). If you missed the prompt,
   enable it under System Settings > Privacy & Security > Camera.

### All Systems:
1. Install the C/C++ VSCode extension if you don't already have it.
2. Install the CMake Tools VSCode extension.
3. When prompted to select a kit, choose your compiler
   (Visual Studio on Windows, GCC on Linux, Clang on macOS).

## Building

In VSCode, open the Command Palette and run `CMake: Configure`, then `CMake: Build`.

Or from a terminal in the project root:
```bash
cmake --build build --config Release
```

If OpenCV isn't found, pass the path explicitly instead of using the environment variable:
```bash
cmake -S . -B build -DOpenCV_DIR=/path/to/opencv/build
```

## Running

Windows:
```
build\Release\ball_balancer.exe
```
Linux / macOS:
```bash
./build/ball_balancer
```

The serial port name differs per platform: `COM3` on Windows, `/dev/ttyUSB0` or
`/dev/ttyACM0` on Linux, and `/dev/cu.usbmodem*` on macOS.
