# esp32-s3-flip-fluid-simulation
FLIP (+ PIC) fluid simulation on ESP32-S3 with 8x8 LED matrix display and accelerometer-controlled gravity.

Main objective is to implement a particle-based fluid dynamics simulation using the FLIP method on embedded hardware, exploring the challenges of real-time physics calculations with limited computational resources while creating an interactive, visually appealing display.

## Table of Contents
- [Hardware Components](#hardware-components)
- [Assembly](#assembly)
- [Pinout Configuration](#pinout-configuration)
- [Physical Setup](#physical-setup)
- [Key Features](#key-features)
  - [FLIP Fluid Simulation](#flip-fluid-simulation)
  - [Incompressibility Solver](#incompressibility-solver)
  - [Particle Dynamics Pipeline](#particle-dynamics-pipeline)
  - [Spatial Hash Grid](#spatial-hash-grid)
  - [Bilinear Interpolation](#bilinear-interpolation)
  - [Button Control System](#button-control-system)
  - [Color Presets](#color-presets)
  - [Visualization](#visualization)
  - [Accelerometer Integration](#accelerometer-integration)
- [Technical Specifications](#technical-specifications)
  - [Performance](#performance)
  - [Memory Usage](#memory-usage)
  - [Compiler Optimizations](#compiler-optimizations)
- [Algorithm Details](#algorithm-details)
  - [FLIP Method](#flip-method)
  - [Pressure Projection](#pressure-projection)
  - [Particle Collision](#particle-collision)
- [Build Environment](#build-environment)
- [Configuration Options](#configuration-options)
- [Usage](#usage)
- [Credit](#credit)
- [License](#license)

## Hardware Components
- Waveshare ESP32-S3-Matrix Development Board with 8x8 led matrix and QMI8658 6-Axis IMU (Accelerometer + Gyroscope) integrated
- Momentary Push Button
- Small switch
- TP4057 1A
- 220mAh Lipo pouch battery
<img width="650" height="310" alt="hardwareComponents" src="https://github.com/user-attachments/assets/90149b72-0045-4879-8765-851080bced2d" />

## Assembly

### Solder de components together using AWG30 wires
<img width="478" height="350" alt="guts" src="https://github.com/user-attachments/assets/23cf3dd8-04e4-4dfe-a456-4fdc2e495380" />

### Design and print the casing
<img width="482" height="350" alt="casing" src="https://github.com/user-attachments/assets/28c9bfaa-b1ef-40e2-9b1b-5ab7b9efb6e8" />

### Place the components in casing
<img width="350" height="455" alt="gutsincasing" src="https://github.com/user-attachments/assets/609cc651-fc70-4232-86ad-caa702ecf82c" />


### Finished product
<img width="1135" height="1500" alt="finished2" src="https://github.com/user-attachments/assets/a118e2cc-514e-43a4-b093-3f5e72528e4d" />


## Pinout Configuration

### NeoPixel LED Matrix
```cpp
LED_PIN = GPIO 14   // WS2812B data line
NUM_LEDS = 64       // 8x8 matrix
```

### QMI8658 IMU (I2C)
```cpp
SDA_PIN = GPIO 11   // I2C data
SCL_PIN = GPIO 12   // I2C clock
QMI8658_ADDR = 0x6B // I2C device address
I2C_FREQ = 400kHz   // fast mode
```

### Control Button
```cpp
BUTTON_PIN = GPIO 5  // user input (with internal pullup)
```

Button is active low (pressed = LOW, released = HIGH)

## Physical Setup
The simulation uses a 500mm x 500mm virtual container. Particles are constrained inside the solid simulation boundary, and the reachable fluid area is mapped across the full 8x8 LED grid for display. The LED matrix serves as a low-resolution display. The QMI8658 accelerometer continuously updates the gravity direction, allowing the fluid to respond naturally to device orientation.

## Key Features

### FLIP Fluid Simulation

(* means configurable as parameter in config.h)
Particle-In-Cell hybrid method combining Lagrangian particles with Eulerian grid:
- **400 Particles:** hexagonally packed for optimal density, radius = 0.3 * cell_size(*)
- **20x20 Simulation Grid:** provides velocity field for incompressibility solving(*)
- **FLIP/PIC Blending:** 94% FLIP + 6% PIC for stability while preserving momentum(*)
- **Frame Interval:** 23ms real-time (43.5 fps target)(*)
- **Physics Timestep:** 27.6ms simulated time per frame (1.2x speed multiplier snappier fluid motion)(*)
- **Gravity Magnitude:** 9.81 m/s^2, direction updated from accelerometer each frame(*)

### Incompressibility Solver
Iterative pressure projection using Gauss-Seidel with successive over-relaxation:
- **15 Iterations:** balances accuracy vs. computation time (~1.5ms solve time)(*)
- **Over-Relaxation Factor:** 1.9x to accelerate convergence(*)
- **Density Correction:** compensates for particle drift using K-factor = 1.0(configurable)(*)
- **Grid Classification:** cells marked as LIQUID, AIR, or WALL based on particle occupancy

### Particle Dynamics Pipeline
Nine-stage FLIP cycle executed each frame:
1. **Integration:** apply gravity, update positions based on velocity
2. **Separation:** push overlapping particles apart (2 iterations, spatial hash grid)
3. **Collision:** clamp particles inside boundaries with restitution/friction
4. **Grid Preparation:** reset cell types, mark liquid cells, save previous velocities
5. **Particle → Grid Transfer:** scatter particle velocities to grid using bilinear weights
6. **Normalization:** divide accumulated velocities by weights, restore wall velocities
7. **Density Calculation:** compute particle density per cell for drift compensation
8. **Incompressibility Solve:** iteratively adjust velocities to minimize divergence
9. **Grid → Particle Transfer:** interpolate corrected velocities back to particles with FLIP/PIC blending

### Spatial Hash Grid
Efficient particle separation using uniform grid acceleration structure:
- **Cell Spacing:** 2.2 × particle_radius for optimal bucket size
- **Initial Packing:** particles initialized at 1.8 × radius spacing (tighter than 2.0) to increase rest density
- **Collision Detection:** only checks particles in same/adjacent cells (9 cells total)
- **Separation Force:** pushes particles to minimum separation distance
- **Memory Layout:** flat arrays (num_cell_particles, first_cell_particle, cell_particle_ids)
- Significantly faster than O(n²) all-pairs check for 400 particles

### Bilinear Interpolation
Smooth velocity transfers between particles and staggered MAC grid:
- **Staggered Grid:** vx stored on vertical faces, vy on horizontal faces
- **4-Point Weights:** precomputed for each particle based on position
- **Sub-Cell Accuracy:** particles can be anywhere within cell, not just at centers
- **Weight Accumulation:** prevents velocity discontinuities at cell boundaries
- Computed once per particle per transfer, reused for both velocity components

### Button Control System
State machine with debouncing for three interaction types:
- **Single Tap (< 600ms):** cycle through 7 brightness levels (3-13 intensity)(*)
- **Double Tap (< 250ms between):** cycle through 17 color presets (blues, greens, reds, yellows, purples, pinks, white)(*)
- **Long Press (≥ 600ms):** cycle through 5 foam threshold levels (0.4-3.0 LED intensity)(*)
- **Debouncing:** 25ms settling time eliminates mechanical noise(*)
- **NVS Persistence:** settings saved to flash immediately after change, restored on boot

### Color Presets
17 carefully tuned color combinations optimized for LED visibility:
- **Blues/Cyans (4):** pure blue, cyan, sky blue, aqua
- **Greens (2):** lime green, yellow-green
- **Reds (2):** pure red, crimson
- **Oranges/Yellows (3):** orange, golden yellow, amber
- **Purples (3):** violet, lavender, deep purple
- **Pinks/Magentas (2):** hot pink, magenta
- **White (1):** full spectrum white

Each preset is scaled by current brightness setting (0-255 range)

### Visualization
Particle density mapping to LED colors with foam effect:
- **Liquid Rendering:** LEDs with intensity above 2.0 show fluid color at current brightness(*)
- **Foam Effect:** LEDs above the current foam threshold display white at 30% brightness(*)
- **Brightness Scaling:** base color RGB values multiplied by brightness factor (3-13)(*)
- **LED Smoothing:** particles are bilinearly splatted into neighboring LEDs with 0.60 frame persistence(*)
- **Update Rate:** 23ms target frame interval (~43.5 fps), with measured compute headroom up to ~122 fps
- Physical-to-pixel mapping: reachable particle region (meters) -> LED coordinates (0-7)

### Accelerometer Integration
Real-time gravity direction from QMI8658 6-axis IMU:
- **Sensor Mode:** accelerometer only (gyroscope disabled for efficiency)
- **Update Frequency:** every frame (~43 Hz)
- **Range:** ±4g with 8192 LSB/g sensitivity
- **Axis Mapping:** device Y-axis → simulation X (horizontal), device X-axis → simulation Y (vertical)
- **Magnitude:** constant 9.81 m/s², direction varies with tilt
- **Response:** fluid immediately reacts to device orientation changes
- **Calibration:** automatic via sensor hardware, no user calibration needed

## Technical Specifications

### Performance
- **Simulation Grid:** 20x20 cells (400 total)
- **Display Grid:** 8x8 LEDs (64 total, low-resolution fluid projection)
- **Particle Count:** 400 active particles
- **Frame Interval:** 23ms target (43.5 fps)
- **Measured Compute Headroom:** 8.16ms total frame time (~122.5 fps theoretical max)
- **Frame Breakdown:**
  - Integration: ~0.02ms
  - Separation: ~2.40ms
  - Collision: ~0.11ms
  - Grid Prep: ~0.18ms
  - P->G Transfer: ~0.55ms
  - Density: ~0.21ms
  - Solver: ~1.53ms
  - G->P Transfer: ~1.00ms
  - Visualization: ~2.16ms
  - **Total:** ~8.16ms per frame

### Memory Usage
- **Particle Array:** 400 * 16 bytes = 6.4 KB
- **Grid Velocities:** 4 * 400 * 4 bytes = 6.4 KB (current + previous)
- **Weight Accumulators:** 2 * 400 * 4 bytes = 3.2 KB
- **Cell Types:** 400 cells
- **Spatial Hash:** ~9 KB (particle bucketing)
- **Density Array:** 400 * 4 bytes = 1.6 KB
- **Liquid Cell List:** 400 * 4 bytes = 1.6 KB
- **Total:** ~30 KB for the main simulation arrays

### Compiler Optimizations
Aggressive optimization flags for maximum performance:
- **-O3:** maximum optimization level
- **-ffast-math:** aggressive floating-point optimizations (IEEE compliance relaxed)
- **-funroll-loops:** unroll loops for better pipelining
- **-fomit-frame-pointer:** free up register for computations
- **CPU Frequency:** 240 MHz (maximum ESP32-S3 speed)
- **Flash Frequency:** 80 MHz for fast code fetching

## Algorithm Details

Implemented following Ten Minute Physics tutorial closely (https://www.youtube.com/watch?v=XmzBREkK8kY)

### FLIP Method
- **Particles:** carry velocity and position, no pressure field
- **Grid:** solves for incompressibility, enforces boundary conditions
- **Transfer P→G:** particles scatter velocities to grid with bilinear interpolation
- **Solve Pressure:** grid velocities adjusted to satisfy â‹…u = 0
- **Transfer G→P:** particles updated with FLIP/PIC blend:
  - **PIC (6%):** vnew = vgrid (dissipative, stable)
  - **FLIP (94%):** vnew = vold + (vgrid - vgrid_old) (preserves momentum)

### Pressure Projection
- **Divergence Calculation:** sum of velocity flux through cell faces
- **Pressure Gradient:** distributes correction to neighboring cells
- **Over-Relaxation:** scales correction by 1.9x to accelerate convergence
- **Boundary Conditions:** solid walls have zero normal velocity
- **Density Compensation:** adds drift correction term based on particle deficit/surplus

### Particle Collision
Handles boundaries with configurable restitution and friction:
- **Wall Detection:** clamp position one simulation cell plus particle_radius inside the domain
- **Restitution Factor:** 0.2 (20% velocity preserved on bounce)(*)
- **Friction Factor:** 0.2 (20% tangential velocity preserved)(*)
- **Clamping:** prevents particles from penetrating walls
- Applied after position update, before grid transfer

## Build Environment

### PlatformIO Configuration
```ini
[env:esp32-s3-matrix]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino

; CRITICAL: Flash configuration for ESP32-S3-Matrix
board_build.flash_mode = dio
board_build.flash_size = 4MB
board_build.partitions = default.csv
board_upload.flash_size = 4MB

; 1. Hardware Accelerations
board_build.f_cpu = 240000000L
board_build.f_flash = 80000000L

build_flags = 
    -DARDUINO_USB_CDC_ON_BOOT=1
    ; compiler optimizations
    -O3
    -ffast-math
    -funroll-loops
    -fomit-frame-pointer
    ; disable debug overhead
    -DCORE_DEBUG_LEVEL=0
    
monitor_speed = 115200
upload_speed = 921600

lib_deps = 
    adafruit/Adafruit NeoPixel@^1.12.0
```

Flash mode and size are pinned explicitly since this board's flash chip isn't always autodetected correctly; the partition table uses PlatformIO's standard `default.csv` scheme.

## Configuration Options

All simulation parameters can be adjusted in `config.h`:

### Physics Constants
```cpp
GRAVITY_MAGNITUDE = 9.81f          // m/s²
FRAME_INTERVAL = 0.023f            // base timestep
SPEED_MULTIPLIER = 1.2f            // time acceleration
FLIP_RATIO = 0.94f                 // FLIP vs PIC blend
INCOMPRESSIBILITY_ITERATIONS = 15  // solver iterations
OVERRELAXATION = 1.9f              // convergence acceleration
```

### Particle Settings
```cpp
NUM_PARTICLES = 400                // total particle count
RESTITUTION_FACTOR = 0.2f          // bounce coefficient
FRICTION_FACTOR = 0.2f             // wall friction
```

### Grid Resolution
```cpp
GRID_SIZE_X = 20                   // simulation cells
GRID_SIZE_Y = 20
PHYSICAL_WIDTH = 0.5f              // meters
PHYSICAL_HEIGHT = 0.5f
```

### Visualization
```cpp
PARTICLE_THRESHOLD = 2.0f          // min LED intensity for liquid color
LED_PERSISTENCE = 0.60f            // display smoothing decay
// Foam thresholds: 3.0, 1.6, 1.2, 0.8, 0.4  // configured in button presets
```

## Usage

### First Boot
1. Device initializes with default settings (blue color, low brightness, low foam)
2. Particles appear in block formation at bottom of display
3. Tilt device to control gravity direction - fluid responds immediately
### Test


https://github.com/user-attachments/assets/9a86d0c9-9984-4224-b226-0375ce9b3e63



### Button Controls
- **Single Tap:** cycle brightness (3 → 4 → 5 → 6 → 9 → 11 → 13 → 3...)
- **Double Tap:** cycle colors (blue → cyan → green → red → orange → purple → pink → white → blue...)
- **Long Press:** cycle foam threshold (none → low → medium → high → max → none...)
### Button funcionality test


https://github.com/user-attachments/assets/5af7af76-c64d-4482-b536-27835820dfac



### Settings Persistence
All button settings automatically save to flash and restore on next power-up.


## Credit

- MAtthias Müller´s Ten Minute Physics: "18 - How to write a FLIP water / fluid simulation running in your browser"
- https://www.youtube.com/watch?v=XmzBREkK8kY

## License
MIT License - feel free to use, modify, and distribute.

