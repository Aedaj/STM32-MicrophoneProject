# STM32 Microphone Processing System 🎙️

[![GitHub](https://img.shields.io/badge/Repository-Public-blue)](https://github.com/AedanJaesonEngineeringProjects/STM32-MicrophoneProject)  
![Platform](https://img.shields.io/badge/Platform-STM32-blue) ![Language](https://img.shields.io/badge/Language-C%2B%2B-orange)  

This project showcases a **real-time audio processing system** using the **STM32F411RE microcontroller**, designed to capture, filter, and enhance audio signals. It features dual modes for normal and privacy-enhanced (robot-voice) audio output and highlights **digital signal processing** with **frequency spectrum analysis**.

---

##  Overview

This system uses **MEMS microphone data acquisition**, **analog-to-digital conversion (ADC)**, and **real-time audio processing** to demonstrate embedded audio signal processing on the STM32 platform. Key capabilities include:

- High-pass filtering to remove low-frequency noise
- Real-time robot voice effect using delayed signal modification
- Fast Fourier Transform (FFT) analysis for frequency spectrum visualization
- SD card audio storage in `.wav` format (both raw and processed audio)
- LCD display for dominant frequency and system status

---

## ⚙️ System Architecture

**Core components:**
- **STM32F411RE Microcontroller:** Main control and processing unit  
- **MEMS Microphone Module:** Audio input source  
- **I2C LCD Display:** Visual feedback on frequency spectrum  
- **MicroSD Card:** Audio data storage (via SPI and FATFS)  
- **SerialPlot Tool:** Real-time visualization on connected PC  

---

##  Key Features

- **Real-Time Processing:** DMA-based ADC ensures continuous audio sampling without data loss.
- **Privacy Mode:** Robot voice effect for privacy-oriented audio masking.
- **High-Pass Filtering:** Eliminates low-frequency background noise (cutoff at 250 Hz).
- **FFT Frequency Spectrum Analysis:** Visualizes dominant frequencies on an LCD.
- **Dual Audio Storage:** Records both original and processed audio to SD card in `.wav` format.
  
---

##  Technologies & Tools

- **STM32CubeIDE:** Development environment  
- **CMSIS DSP Library:** Optimized DSP functions  
- **FATFS:** File system management for SD card  
- **HAL Libraries:** STM32 peripheral abstraction  
- **SerialPlot:** Graphical audio data visualization  

---

##  Setup and Configuration

1. **Clone the Repository:**
   ```bash
   git clone https://github.com/AedanJaesonEngineeringProjects/STM32-MicrophoneProject.git
   cd STM32-MicrophoneProject
