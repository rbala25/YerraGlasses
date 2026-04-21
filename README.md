# YerraGlasses
 
Audio-augmented smart glasses that act as a wearable guide dog — real-time object detection converted to spatial audio feedback, all running on an ESP32-S3 tucked inside a glasses frame.
 
---
 
## Highlights
 
**Embedded ML Object Detection** — Neural network running on a **Luckfox RV1103 NPU** detects objects from live visual input and converts them into real-time auditory cues. Designed to assist blind and visually impaired users without any cloud dependency.
 
**Dual I2S Class-D Audio** — Two Class-D amplifiers driven over I2S deliver low-latency directional audio feedback. Custom DAC and speaker amplification stages built discrete, with 26-gauge wire crimped and routed by hand to fit the form factor.
 
**Space-Constrained PCB** — Custom board designed to fit inside a glasses frame. Integrates USB-C input, rail regulation, I2C sensor communication, and the full I2S audio pipeline — all in a footprint small enough to wear.
 
**Power System** — Robust battery management with USB-C charging, regulated power rails, and firmware-level power state detection to maximize battery life in a miniature form factor.
 
---
 
## Stack
 
| Layer | Tech |
|---|---|
| MCU | ESP32-S3 |
| Language | C, bare-metal |
| ML Inference | Luckfox RV1103 NPU (object detection) |
| Audio | Dual I2S Class-D amps + custom DAC |
| Communication | I2C |
| Power | USB-C + regulated rails + battery monitoring |
| Hardware | Custom PCB, 26-gauge hand-crimped wiring |
