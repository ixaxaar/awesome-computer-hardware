# Awesome Computer Hardware [![Awesome](https://awesome.re/badge.svg)](https://awesome.re)

> A list of computer hardware technologies, architectures, specifications, and standards

- [Awesome Computer Hardware ](#awesome-computer-hardware-)
    - [Processor Technologies](#processor-technologies)
        - [CPU Architectures](#cpu-architectures)
            - [x86 Family](#x86-family)
            - [ARM Family](#arm-family)
            - [RISC Architectures](#risc-architectures)
            - [Embedded \& Microcontroller Architectures](#embedded--microcontroller-architectures)
            - [Historic Architectures](#historic-architectures)
        - [CPU Microarchitectures](#cpu-microarchitectures)
            - [Intel Microarchitectures](#intel-microarchitectures)
            - [AMD Microarchitectures](#amd-microarchitectures)
            - [ARM Microarchitectures](#arm-microarchitectures)
        - [Instruction Set Extensions](#instruction-set-extensions)
            - [x86 SIMD Extensions](#x86-simd-extensions)
            - [x86 Specialized Extensions](#x86-specialized-extensions)
            - [ARM SIMD Extensions](#arm-simd-extensions)
            - [PowerPC SIMD Extensions](#powerpc-simd-extensions)
            - [RISC-V Vector Extensions](#risc-v-vector-extensions)
            - [Other Architecture Extensions](#other-architecture-extensions)
        - [Virtualization Technologies](#virtualization-technologies)
        - [Security Technologies](#security-technologies)
    - [Platform-Specific Hardware](#platform-specific-hardware)
        - [Embedded Systems](#embedded-systems)
            - [Microcontrollers (MCU)](#microcontrollers-mcu)
            - [System-on-Chip (SoC)](#system-on-chip-soc)
            - [Single Board Computers](#single-board-computers)
            - [IoT Platforms](#iot-platforms)
            - [Real-Time Operating System Hardware](#real-time-operating-system-hardware)
        - [Desktop \& Laptop Hardware](#desktop--laptop-hardware)
            - [Form Factors](#form-factors)
            - [Motherboards](#motherboards)
            - [Mobile-Specific Components](#mobile-specific-components)
        - [Server Hardware](#server-hardware)
            - [Server Form Factors](#server-form-factors)
            - [Server Processors](#server-processors)
            - [Server Memory](#server-memory)
            - [RAID Controllers](#raid-controllers)
            - [Management Controllers](#management-controllers)
        - [HPC \& Supercomputing](#hpc--supercomputing)
            - [HPC Interconnects](#hpc-interconnects)
            - [Accelerator Technologies](#accelerator-technologies)
            - [Supercomputer Architectures](#supercomputer-architectures)
        - [Mobile Devices](#mobile-devices)
            - [Mobile SoCs](#mobile-socs)
            - [Mobile Memory](#mobile-memory)
            - [Mobile Display Technologies](#mobile-display-technologies)
        - [Edge Computing](#edge-computing)
            - [Edge Processors](#edge-processors)
            - [Edge AI Accelerators](#edge-ai-accelerators)
    - [Industry-Specific Hardware](#industry-specific-hardware)
        - [Automotive Electronics](#automotive-electronics)
            - [Automotive Processors](#automotive-processors)
            - [ADAS \& Autonomous Driving](#adas--autonomous-driving)
            - [Vehicle Networks](#vehicle-networks)
            - [Automotive Sensors](#automotive-sensors)
        - [Industrial Electronics](#industrial-electronics)
            - [Industrial Controllers](#industrial-controllers)
            - [Industrial Communication](#industrial-communication)
            - [Industrial Sensors](#industrial-sensors)
        - [Aerospace \& Defense](#aerospace--defense)
            - [Radiation-Hardened Components](#radiation-hardened-components)
            - [Avionics Systems](#avionics-systems)
            - [Military Standards](#military-standards)
        - [Medical Devices](#medical-devices)
            - [Medical Imaging Hardware](#medical-imaging-hardware)
            - [Patient Monitoring](#patient-monitoring)
            - [Medical Safety Standards](#medical-safety-standards)
        - [Telecommunications](#telecommunications)
            - [Baseband Processors](#baseband-processors)
            - [RF Components](#rf-components)
            - [Telecom Standards](#telecom-standards)
    - [Fabrication Processes](#fabrication-processes)
        - [Current Process Nodes](#current-process-nodes)
        - [Mature Process Nodes](#mature-process-nodes)
        - [Future Process Nodes](#future-process-nodes)
        - [Manufacturing Technologies](#manufacturing-technologies)
        - [Foundries](#foundries)
        - [Packaging Technologies](#packaging-technologies)
    - [Memory Technologies](#memory-technologies)
        - [Volatile Memory](#volatile-memory)
            - [DRAM Types](#dram-types)
            - [SRAM Types](#sram-types)
        - [Non-Volatile Memory](#non-volatile-memory)
            - [Flash Memory](#flash-memory)
            - [Emerging NVM](#emerging-nvm)
        - [Cache Architectures](#cache-architectures)
    - [Storage Technologies](#storage-technologies)
        - [Storage Media](#storage-media)
            - [Solid State](#solid-state)
            - [Magnetic](#magnetic)
            - [Optical](#optical)
            - [Tape](#tape)
        - [Storage Interfaces](#storage-interfaces)
        - [Storage Protocols](#storage-protocols)
        - [Storage Controllers](#storage-controllers)
    - [Interconnect Technologies](#interconnect-technologies)
        - [System Buses](#system-buses)
        - [High-Speed Interconnects](#high-speed-interconnects)
        - [CPU Interconnects](#cpu-interconnects)
        - [Memory Interfaces](#memory-interfaces)
        - [On-Chip Interconnects](#on-chip-interconnects)
    - [Graphics Technologies](#graphics-technologies)
        - [GPU Architectures](#gpu-architectures)
        - [Graphics APIs](#graphics-apis)
        - [Ray Tracing Technologies](#ray-tracing-technologies)
        - [Video Encode/Decode](#video-encodedecode)
        - [Display Technologies](#display-technologies)
    - [Network Technologies](#network-technologies)
        - [Ethernet Standards](#ethernet-standards)
        - [Wireless Technologies](#wireless-technologies)
        - [Network Protocols](#network-protocols)
        - [Network Processors](#network-processors)
    - [I/O Technologies](#io-technologies)
        - [Universal Serial Bus (USB)](#universal-serial-bus-usb)
        - [Display Interfaces](#display-interfaces)
        - [Audio Interfaces](#audio-interfaces)
        - [Legacy I/O](#legacy-io)
    - [Input Devices](#input-devices)
        - [Keyboards](#keyboards)
        - [Pointing Devices](#pointing-devices)
        - [Touchscreens](#touchscreens)
        - [Game Controllers](#game-controllers)
        - [Biometric Readers](#biometric-readers)
    - [Output Devices](#output-devices)
        - [Displays](#displays)
            - [Display Panel Types](#display-panel-types)
            - [Display Resolutions](#display-resolutions)
        - [Printers](#printers)
        - [Projectors](#projectors)
        - [Audio Output](#audio-output)
    - [Sensors \& Data Acquisition](#sensors--data-acquisition)
        - [Environmental Sensors](#environmental-sensors)
        - [Motion Sensors](#motion-sensors)
        - [Optical Sensors](#optical-sensors)
        - [Position \& Navigation](#position--navigation)
        - [ADC/DAC](#adcdac)
    - [Chipset Technologies](#chipset-technologies)
        - [Intel Chipsets](#intel-chipsets)
        - [AMD Chipsets](#amd-chipsets)
        - [Mobile Chipsets](#mobile-chipsets)
    - [Power Management](#power-management)
        - [Power Standards](#power-standards)
        - [CPU Power Technologies](#cpu-power-technologies)
        - [Battery Technologies](#battery-technologies)
        - [Power Delivery](#power-delivery)
        - [Voltage Regulators](#voltage-regulators)
    - [Clock \& Timing](#clock--timing)
        - [Crystal Oscillators](#crystal-oscillators)
        - [Clock Generators](#clock-generators)
        - [Timing Standards](#timing-standards)
    - [Firmware Technologies](#firmware-technologies)
        - [Boot Technologies](#boot-technologies)
        - [Management Technologies](#management-technologies)
        - [Embedded Firmware](#embedded-firmware)
    - [Cooling Technologies](#cooling-technologies)
        - [Thermal Management](#thermal-management)
        - [Active Cooling](#active-cooling)
        - [Passive Cooling](#passive-cooling)
    - [Connectors \& Cables](#connectors--cables)
        - [Power Connectors](#power-connectors)
        - [Data Connectors](#data-connectors)
        - [Board-to-Board Connectors](#board-to-board-connectors)
        - [Cable Types](#cable-types)
    - [Mechanical Components](#mechanical-components)
        - [Computer Cases](#computer-cases)
        - [Server Racks](#server-racks)
        - [Enclosures](#enclosures)
        - [Mounting Hardware](#mounting-hardware)
    - [Hardware Design \& Manufacturing](#hardware-design--manufacturing)
        - [PCB Design](#pcb-design)
        - [IC Packaging](#ic-packaging)
        - [Assembly Technologies](#assembly-technologies)
        - [Design Tools](#design-tools)
    - [Emerging Technologies](#emerging-technologies)
        - [Quantum Computing](#quantum-computing)
        - [Neuromorphic Computing](#neuromorphic-computing)
        - [Optical Computing](#optical-computing)
        - [DNA Storage](#dna-storage)
        - [Carbon Nanotube Electronics](#carbon-nanotube-electronics)
        - [Spintronics](#spintronics)
    - [Specialized Computing](#specialized-computing)
        - [AI/ML Accelerators](#aiml-accelerators)
        - [FPGA Technologies](#fpga-technologies)
        - [DSP Technologies](#dsp-technologies)
        - [Cryptographic Accelerators](#cryptographic-accelerators)
        - [Network Accelerators](#network-accelerators)
        - [Storage Accelerators](#storage-accelerators)
    - [Standards Organizations](#standards-organizations)
        - [Industry Consortia](#industry-consortia)
        - [Standards Bodies](#standards-bodies)
    - [Testing and Validation](#testing-and-validation)
        - [Test Standards](#test-standards)
        - [Reliability Testing](#reliability-testing)
        - [Certification Programs](#certification-programs)


## Processor Technologies

### CPU Architectures

#### x86 Family
- **x86** - [Wikipedia](https://en.wikipedia.org/wiki/X86) | [Intel x86 Manuals](https://www.intel.com/content/www/us/en/developer/articles/technical/intel-sdm.html)
- **x86-64 (AMD64)** - [Wikipedia](https://en.wikipedia.org/wiki/X86-64) | [AMD64 Spec](https://developer.amd.com/resources/developer-guides-manuals/)
- **IA-32** - [Wikipedia](https://en.wikipedia.org/wiki/IA-32)
- **Itanium (IA-64)** - [Wikipedia](https://en.wikipedia.org/wiki/Itanium) | [Intel Specs](https://www.intel.com/content/www/us/en/processors/itanium/)

#### ARM Family
- **ARM** - [Wikipedia](https://en.wikipedia.org/wiki/ARM_architecture_family) | [ARM Specs](https://developer.arm.com/documentation)
- **AArch32** - [Wikipedia](https://en.wikipedia.org/wiki/ARM_architecture_family#AArch32)
- **AArch64** - [Wikipedia](https://en.wikipedia.org/wiki/AArch64) | [ARM AArch64](https://developer.arm.com/architectures/cpu-architecture/a-profile)
- **Thumb** - [Wikipedia](https://en.wikipedia.org/wiki/ARM_architecture_family#Thumb)
- **Thumb-2** - [Wikipedia](https://en.wikipedia.org/wiki/ARM_architecture_family#Thumb-2)

#### RISC Architectures
- **RISC-V** - [Wikipedia](https://en.wikipedia.org/wiki/RISC-V) | [RISC-V Spec](https://riscv.org/technical/specifications/)
- **MIPS** - [Wikipedia](https://en.wikipedia.org/wiki/MIPS_architecture) | [MIPS Specs](https://www.mips.com/products/)
- **PowerPC** - [Wikipedia](https://en.wikipedia.org/wiki/PowerPC) | [Power ISA](https://openpowerfoundation.org/)
- **Power ISA** - [Wikipedia](https://en.wikipedia.org/wiki/Power_ISA)
- **SPARC** - [Wikipedia](https://en.wikipedia.org/wiki/SPARC) | [Oracle SPARC](https://www.oracle.com/servers/sparc/)
- **Alpha** - [Wikipedia](https://en.wikipedia.org/wiki/DEC_Alpha)
- **PA-RISC** - [Wikipedia](https://en.wikipedia.org/wiki/PA-RISC)

#### Embedded & Microcontroller Architectures
- **AVR** - [Wikipedia](https://en.wikipedia.org/wiki/AVR_microcontrollers) | [Microchip AVR](https://www.microchip.com/en-us/products/microcontrollers-and-microprocessors/8-bit-mcus/avr-mcus)
- **PIC** - [Wikipedia](https://en.wikipedia.org/wiki/PIC_microcontrollers) | [Microchip PIC](https://www.microchip.com/en-us/products/microcontrollers-and-microprocessors/8-bit-mcus/pic-mcus)
- **8051** - [Wikipedia](https://en.wikipedia.org/wiki/Intel_MCS-51)
- **68HC11** - [Wikipedia](https://en.wikipedia.org/wiki/68HC11)
- **MSP430** - [Wikipedia](https://en.wikipedia.org/wiki/MSP430) | [TI MSP430](https://www.ti.com/microcontrollers-mcus-processors/msp430-microcontrollers/overview.html)

#### Historic Architectures
- **6502** - [Wikipedia](https://en.wikipedia.org/wiki/MOS_Technology_6502)
- **Z80** - [Wikipedia](https://en.wikipedia.org/wiki/Zilog_Z80)
- **8080** - [Wikipedia](https://en.wikipedia.org/wiki/Intel_8080)
- **68000** - [Wikipedia](https://en.wikipedia.org/wiki/Motorola_68000)
- **VAX** - [Wikipedia](https://en.wikipedia.org/wiki/VAX)
- **PDP-11** - [Wikipedia](https://en.wikipedia.org/wiki/PDP-11)

### CPU Microarchitectures

#### Intel Microarchitectures
- **Raptor Lake** - [Wikipedia](https://en.wikipedia.org/wiki/Raptor_Lake) | [Intel Specs](https://www.intel.com/content/www/us/en/products/details/processors/core.html)
- **Alder Lake** - [Wikipedia](https://en.wikipedia.org/wiki/Alder_Lake) | [Intel Specs](https://www.intel.com/content/www/us/en/products/details/processors/core/12th-gen.html)
- **Rocket Lake** - [Wikipedia](https://en.wikipedia.org/wiki/Rocket_Lake)
- **Tiger Lake** - [Wikipedia](https://en.wikipedia.org/wiki/Tiger_Lake)
- **Ice Lake** - [Wikipedia](https://en.wikipedia.org/wiki/Ice_Lake_(microprocessor))
- **Comet Lake** - [Wikipedia](https://en.wikipedia.org/wiki/Comet_Lake)
- **Coffee Lake** - [Wikipedia](https://en.wikipedia.org/wiki/Coffee_Lake)
- **Kaby Lake** - [Wikipedia](https://en.wikipedia.org/wiki/Kaby_Lake)
- **Skylake** - [Wikipedia](https://en.wikipedia.org/wiki/Skylake_(microarchitecture))
- **Broadwell** - [Wikipedia](https://en.wikipedia.org/wiki/Broadwell_(microarchitecture))
- **Haswell** - [Wikipedia](https://en.wikipedia.org/wiki/Haswell_(microarchitecture))
- **Ivy Bridge** - [Wikipedia](https://en.wikipedia.org/wiki/Ivy_Bridge_(microarchitecture))
- **Sandy Bridge** - [Wikipedia](https://en.wikipedia.org/wiki/Sandy_Bridge)
- **Nehalem** - [Wikipedia](https://en.wikipedia.org/wiki/Nehalem_(microarchitecture))
- **Core** - [Wikipedia](https://en.wikipedia.org/wiki/Intel_Core_(microarchitecture))
- **NetBurst (Pentium 4)** - [Wikipedia](https://en.wikipedia.org/wiki/NetBurst_(microarchitecture))
- **P6 (Pentium Pro/II/III)** - [Wikipedia](https://en.wikipedia.org/wiki/P6_(microarchitecture))
- **Intel Atom (Tremont, Gracemont)** - [Wikipedia](https://en.wikipedia.org/wiki/Intel_Atom) | [Intel Atom](https://www.intel.com/content/www/us/en/products/details/processors/atom.html)

#### AMD Microarchitectures
- **Zen 4** - [Wikipedia](https://en.wikipedia.org/wiki/Zen_4) | [AMD Ryzen 7000](https://www.amd.com/en/products/processors/desktops/ryzen.html)
- **Zen 3** - [Wikipedia](https://en.wikipedia.org/wiki/Zen_3) | [AMD Ryzen 5000](https://www.amd.com/en/products/processors/desktops/ryzen.html)
- **Zen 2** - [Wikipedia](https://en.wikipedia.org/wiki/Zen_2)
- **Zen+** - [Wikipedia](https://en.wikipedia.org/wiki/Zen%2B)
- **Zen** - [Wikipedia](https://en.wikipedia.org/wiki/Zen_(microarchitecture))
- **Excavator** - [Wikipedia](https://en.wikipedia.org/wiki/Excavator_(microarchitecture))
- **Steamroller** - [Wikipedia](https://en.wikipedia.org/wiki/Steamroller_(microarchitecture))
- **Piledriver** - [Wikipedia](https://en.wikipedia.org/wiki/Piledriver_(microarchitecture))
- **Bulldozer** - [Wikipedia](https://en.wikipedia.org/wiki/Bulldozer_(microarchitecture))
- **K10** - [Wikipedia](https://en.wikipedia.org/wiki/AMD_K10)
- **K8** - [Wikipedia](https://en.wikipedia.org/wiki/AMD_K8)
- **RDNA (GPU)** - [Wikipedia](https://en.wikipedia.org/wiki/RDNA_(microarchitecture))
- **GCN (GPU)** - [Wikipedia](https://en.wikipedia.org/wiki/Graphics_Core_Next)

#### ARM Microarchitectures
- **Cortex-X4** - [Wikipedia](https://en.wikipedia.org/wiki/ARM_Cortex-X) | [ARM Cortex-X](https://www.arm.com/products/silicon-ip-cpu/cortex-x)
- **Cortex-X3** - [Wikipedia](https://en.wikipedia.org/wiki/ARM_Cortex-X)
- **Cortex-A715** - [Wikipedia](https://en.wikipedia.org/wiki/ARM_Cortex-A#Cortex-A700_series) | [ARM Cortex-A](https://www.arm.com/products/silicon-ip-cpu/cortex-a)
- **Cortex-A78** - [Wikipedia](https://en.wikipedia.org/wiki/ARM_Cortex-A#Cortex-A78)
- **Cortex-A77** - [Wikipedia](https://en.wikipedia.org/wiki/ARM_Cortex-A#Cortex-A77)
- **Cortex-A76** - [Wikipedia](https://en.wikipedia.org/wiki/ARM_Cortex-A#Cortex-A76)
- **Cortex-A75** - [Wikipedia](https://en.wikipedia.org/wiki/ARM_Cortex-A#Cortex-A75)
- **Cortex-A73** - [Wikipedia](https://en.wikipedia.org/wiki/ARM_Cortex-A#Cortex-A73)
- **Cortex-A72** - [Wikipedia](https://en.wikipedia.org/wiki/ARM_Cortex-A#Cortex-A72)
- **Cortex-A57** - [Wikipedia](https://en.wikipedia.org/wiki/ARM_Cortex-A#Cortex-A57)
- **Cortex-A53** - [Wikipedia](https://en.wikipedia.org/wiki/ARM_Cortex-A#Cortex-A53)
- **Cortex-A9** - [Wikipedia](https://en.wikipedia.org/wiki/ARM_Cortex-A#Cortex-A9)
- **Cortex-A8** - [Wikipedia](https://en.wikipedia.org/wiki/ARM_Cortex-A#Cortex-A8)
- **Cortex-R** - [Wikipedia](https://en.wikipedia.org/wiki/ARM_Cortex-R) | [ARM Cortex-R](https://www.arm.com/products/silicon-ip-cpu/cortex-r)
- **Cortex-M** - [Wikipedia](https://en.wikipedia.org/wiki/ARM_Cortex-M) | [ARM Cortex-M](https://www.arm.com/products/silicon-ip-cpu/cortex-m)
- **Neoverse** - [Wikipedia](https://en.wikipedia.org/wiki/ARM_Neoverse) | [ARM Neoverse](https://www.arm.com/products/silicon-ip-cpu/neoverse)
- **Apple Silicon (M-series)** - [Wikipedia](https://en.wikipedia.org/wiki/Apple_silicon) | [Apple M-series](https://www.apple.com/mac/m1/)

### Instruction Set Extensions

#### x86 SIMD Extensions
- **MMX** - [Wikipedia](https://en.wikipedia.org/wiki/MMX_(instruction_set)) | [Intel MMX](https://www.intel.com/content/www/us/en/architecture-and-technology/mmx/mmx-technology-general-information.html)
- **SSE (Streaming SIMD Extensions)** - [Wikipedia](https://en.wikipedia.org/wiki/Streaming_SIMD_Extensions) | [Intel Intrinsics Guide](https://software.intel.com/sites/landingpage/IntrinsicsGuide/)
- **SSE2** - [Wikipedia](https://en.wikipedia.org/wiki/SSE2)
- **SSE3** - [Wikipedia](https://en.wikipedia.org/wiki/SSE3)
- **SSSE3** - [Wikipedia](https://en.wikipedia.org/wiki/SSSE3)
- **SSE4** - [Wikipedia](https://en.wikipedia.org/wiki/SSE4)
- **SSE4.1** - [Wikipedia](https://en.wikipedia.org/wiki/SSE4#SSE4.1)
- **SSE4.2** - [Wikipedia](https://en.wikipedia.org/wiki/SSE4#SSE4.2)
- **AVX (Advanced Vector Extensions)** - [Wikipedia](https://en.wikipedia.org/wiki/Advanced_Vector_Extensions)
- **AVX2** - [Wikipedia](https://en.wikipedia.org/wiki/Advanced_Vector_Extensions#AVX2)
- **AVX-512** - [Wikipedia](https://en.wikipedia.org/wiki/AVX-512)
- **FMA (Fused Multiply-Add)** - [Wikipedia](https://en.wikipedia.org/wiki/FMA_instruction_set)
- **FMA3** - [Wikipedia](https://en.wikipedia.org/wiki/FMA_instruction_set#FMA3_instruction_set)
- **FMA4** - [Wikipedia](https://en.wikipedia.org/wiki/FMA_instruction_set#FMA4_instruction_set)
- **XOP (eXtended Operations)** - [Wikipedia](https://en.wikipedia.org/wiki/XOP_instruction_set)
- **F16C** - [Wikipedia](https://en.wikipedia.org/wiki/F16C)

#### x86 Specialized Extensions
- **AES-NI** - [Wikipedia](https://en.wikipedia.org/wiki/AES_instruction_set)
- **CLMUL** - [Wikipedia](https://en.wikipedia.org/wiki/CLMUL_instruction_set)
- **SHA Extensions** - [Wikipedia](https://en.wikipedia.org/wiki/Intel_SHA_extensions)
- **BMI (Bit Manipulation Instructions)** - [Wikipedia](https://en.wikipedia.org/wiki/Bit_manipulation_instruction_set)
- **BMI2** - [Wikipedia](https://en.wikipedia.org/wiki/Bit_manipulation_instruction_set#BMI2)
- **ADX (Multi-Precision Add-Carry)** - [Wikipedia](https://en.wikipedia.org/wiki/Intel_ADX)
- **RDSEED** - [Wikipedia](https://en.wikipedia.org/wiki/RDRAND#RDSEED)
- **RDRAND** - [Wikipedia](https://en.wikipedia.org/wiki/RDRAND)
- **POPCNT** - [Wikipedia](https://en.wikipedia.org/wiki/SSE4#POPCNT_and_LZCNT)
- **LZCNT** - [Wikipedia](https://en.wikipedia.org/wiki/SSE4#POPCNT_and_LZCNT)
- **MOVBE** - [Wikipedia](https://en.wikipedia.org/wiki/SSE4#MOVBE)
- **PREFETCH** - [Wikipedia](https://en.wikipedia.org/wiki/Cache_prefetching)

#### ARM SIMD Extensions
- **NEON (Advanced SIMD)** - [Wikipedia](https://en.wikipedia.org/wiki/ARM_architecture_family#Advanced_SIMD_(Neon)) | [ARM NEON](https://developer.arm.com/architectures/instruction-sets/simd-isas/neon)
- **SVE (Scalable Vector Extension)** - [Wikipedia](https://en.wikipedia.org/wiki/AArch64#Scalable_Vector_Extension_(SVE)) | [ARM SVE](https://developer.arm.com/architectures/instruction-sets/simd-isas/sve)
- **SVE2** - [Wikipedia](https://en.wikipedia.org/wiki/AArch64#Scalable_Vector_Extension_2_(SVE2))
- **Helium (M-Profile Vector Extension)** - [Wikipedia](https://en.wikipedia.org/wiki/ARM_Cortex-M#Helium) | [ARM Helium](https://developer.arm.com/architectures/instruction-sets/simd-isas/helium)

#### PowerPC SIMD Extensions
- **AltiVec** - [Wikipedia](https://en.wikipedia.org/wiki/AltiVec)
- **VSX (Vector Scalar Extension)** - [Wikipedia](https://en.wikipedia.org/wiki/VSX)

#### RISC-V Vector Extensions
- **RVV (RISC-V Vector Extension)** - [Wikipedia](https://en.wikipedia.org/wiki/RISC-V#Vector_extensions) | [RISC-V Vector Spec](https://github.com/riscv/riscv-v-spec)

#### Other Architecture Extensions
- **SPARC VIS** - [Wikipedia](https://en.wikipedia.org/wiki/Visual_Instruction_Set)
- **MIPS SIMD** - [Wikipedia](https://en.wikipedia.org/wiki/MIPS_architecture#SIMD_extensions)

### Virtualization Technologies
- **Intel VT-x** - [Wikipedia](https://en.wikipedia.org/wiki/X86_virtualization#Intel_VT-x) | [Intel VT-x Spec](https://www.intel.com/content/www/us/en/virtualization/virtualization-technology/intel-virtualization-technology.html)
- **AMD-V (SVM)** - [Wikipedia](https://en.wikipedia.org/wiki/X86_virtualization#AMD-V) | [AMD-V Spec](https://developer.amd.com/resources/developer-guides-manuals/)
- **Intel VT-d** - [Wikipedia](https://en.wikipedia.org/wiki/X86_virtualization#Intel_VT-d) | [Intel VT-d Spec](https://www.intel.com/content/www/us/en/virtualization/virtualization-technology/intel-virtualization-technology.html)
- **AMD-Vi (IOMMU)** - [Wikipedia](https://en.wikipedia.org/wiki/Input%E2%80%93output_memory_management_unit#AMD-Vi)
- **ARM Virtualization Extensions** - [Wikipedia](https://en.wikipedia.org/wiki/ARM_architecture_family#Virtualization_extensions)
- **Nested Page Tables (NPT)** - [Wikipedia](https://en.wikipedia.org/wiki/Second_Level_Address_Translation)
- **Extended Page Tables (EPT)** - [Wikipedia](https://en.wikipedia.org/wiki/Second_Level_Address_Translation)

### Security Technologies
- **Intel TXT (Trusted Execution Technology)** - [Wikipedia](https://en.wikipedia.org/wiki/Trusted_Execution_Technology)
- **AMD Secure Memory Encryption (SME)** - [Wikipedia](https://en.wikipedia.org/wiki/Zen_(microarchitecture)#Security_features)
- **Intel SGX (Software Guard Extensions)** - [Wikipedia](https://en.wikipedia.org/wiki/Software_Guard_Extensions)
- **ARM TrustZone** - [Wikipedia](https://en.wikipedia.org/wiki/ARM_TrustZone)
- **Intel CET (Control-flow Enforcement Technology)** - [Wikipedia](https://en.wikipedia.org/wiki/Intel_CET)
- **ARM Pointer Authentication** - [Wikipedia](https://en.wikipedia.org/wiki/ARM_architecture_family#Pointer_authentication)

## Platform-Specific Hardware

### Embedded Systems

#### Microcontrollers (MCU)
- **STM32 Family** - [Wikipedia](https://en.wikipedia.org/wiki/STM32) | [STMicroelectronics](https://www.st.com/en/microcontrollers-microprocessors/stm32-32-bit-arm-cortex-mcus.html)
- **ESP32** - [Wikipedia](https://en.wikipedia.org/wiki/ESP32) | [Espressif](https://www.espressif.com/en/products/socs/esp32)
- **ESP8266** - [Wikipedia](https://en.wikipedia.org/wiki/ESP8266)
- **Arduino** - [Wikipedia](https://en.wikipedia.org/wiki/Arduino) | [Arduino](https://www.arduino.cc/)
- **Teensy** - [Wikipedia](https://en.wikipedia.org/wiki/Teensy) | [PJRC](https://www.pjrc.com/teensy/)
- **nRF Series (Nordic)** - [Wikipedia](https://en.wikipedia.org/wiki/Nordic_Semiconductor) | [Nordic nRF](https://www.nordicsemi.com/Products)
- **RP2040 (Raspberry Pi)** - [Wikipedia](https://en.wikipedia.org/wiki/RP2040) | [Raspberry Pi Pico](https://www.raspberrypi.com/products/rp2040/)
- **SAM Family (Microchip)** - [Microchip SAM](https://www.microchip.com/en-us/products/microcontrollers-and-microprocessors/32-bit-mcus)
- **Kinetis (NXP)** - [NXP Kinetis](https://www.nxp.com/products/processors-and-microcontrollers/arm-microcontrollers/general-purpose-mcus/k-series-cortex-m4:KINETIS_K_SERIES)
- **LPC Series (NXP)** - [NXP LPC](https://www.nxp.com/products/processors-and-microcontrollers/arm-microcontrollers/general-purpose-mcus/lpc-cortex-m-mcus:LPC-CORTEX-M-MCUS)
- **CC Series (Texas Instruments)** - [TI SimpleLink](https://www.ti.com/wireless-connectivity/overview.html)
- **PSoC (Infineon)** - [Wikipedia](https://en.wikipedia.org/wiki/Programmable_system-on-chip) | [Infineon PSoC](https://www.infineon.com/cms/en/product/microcontroller/32-bit-psoc-arm-cortex-microcontroller/psoc-6-32-bit-arm-cortex-m4-mcu/)
- **RISC-V MCUs** - [SiFive](https://www.sifive.com/cores) | [GigaDevice](https://www.gigadevice.com/products/microcontrollers/gd32/risc-v/)

#### System-on-Chip (SoC)
- **Raspberry Pi SoCs (BCM2xxx)** - [Wikipedia](https://en.wikipedia.org/wiki/Raspberry_Pi) | [Raspberry Pi](https://www.raspberrypi.com/)
- **Qualcomm Snapdragon** - [Wikipedia](https://en.wikipedia.org/wiki/Qualcomm_Snapdragon) | [Qualcomm](https://www.qualcomm.com/products/mobile/snapdragon)
- **MediaTek Dimensity** - [Wikipedia](https://en.wikipedia.org/wiki/MediaTek) | [MediaTek](https://www.mediatek.com/products/smartphones/mediatek-dimensity)
- **Samsung Exynos** - [Wikipedia](https://en.wikipedia.org/wiki/Exynos) | [Samsung](https://www.samsung.com/semiconductor/minisite/exynos/)
- **Apple A-series** - [Wikipedia](https://en.wikipedia.org/wiki/Apple_A-series) | [Apple](https://www.apple.com/iphone/)
- **NVIDIA Tegra** - [Wikipedia](https://en.wikipedia.org/wiki/Tegra) | [NVIDIA Tegra](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/)
- **NXP i.MX** - [Wikipedia](https://en.wikipedia.org/wiki/I.MX) | [NXP i.MX](https://www.nxp.com/products/processors-and-microcontrollers/arm-processors/i-mx-applications-processors:IMX_HOME)
- **TI Sitara** - [Wikipedia](https://en.wikipedia.org/wiki/Sitara) | [TI Sitara](https://www.ti.com/processors/sitara-arm/overview.html)
- **Allwinner** - [Wikipedia](https://en.wikipedia.org/wiki/Allwinner_Technology) | [Allwinner](http://www.allwinnertech.com/)
- **Rockchip** - [Wikipedia](https://en.wikipedia.org/wiki/Rockchip) | [Rockchip](http://www.rock-chips.com/)
- **Amlogic** - [Wikipedia](https://en.wikipedia.org/wiki/Amlogic) | [Amlogic](https://www.amlogic.com/)
- **Broadcom SoCs** - [Wikipedia](https://en.wikipedia.org/wiki/Broadcom) | [Broadcom](https://www.broadcom.com/)

#### Single Board Computers
- **Raspberry Pi** - [Wikipedia](https://en.wikipedia.org/wiki/Raspberry_Pi) | [Official Site](https://www.raspberrypi.org/)
- **BeagleBone** - [Wikipedia](https://en.wikipedia.org/wiki/BeagleBoard#BeagleBone) | [BeagleBoard](https://beagleboard.org/)
- **Odroid** - [Wikipedia](https://en.wikipedia.org/wiki/ODROID) | [Hardkernel](https://www.hardkernel.com/)
- **Orange Pi** - [Orange Pi](http://www.orangepi.org/)
- **Banana Pi** - [Wikipedia](https://en.wikipedia.org/wiki/Banana_Pi) | [Banana Pi](https://www.banana-pi.org/)
- **NVIDIA Jetson** - [Wikipedia](https://en.wikipedia.org/wiki/Nvidia_Jetson) | [NVIDIA Jetson](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/)
- **Intel NUC** - [Wikipedia](https://en.wikipedia.org/wiki/Next_Unit_of_Computing) | [Intel NUC](https://www.intel.com/content/www/us/en/products/details/nuc.html)
- **LattePanda** - [LattePanda](https://www.lattepanda.com/)
- **Pine64** - [Wikipedia](https://en.wikipedia.org/wiki/Pine64) | [Pine64](https://www.pine64.org/)
- **Rock Pi** - [Radxa](https://rockpi.org/)

#### IoT Platforms
- **ESP-IDF** - [Espressif IoT](https://www.espressif.com/en/products/sdks/esp-idf)
- **Zephyr RTOS Hardware** - [Wikipedia](https://en.wikipedia.org/wiki/Zephyr_(operating_system)) | [Zephyr Project](https://www.zephyrproject.org/)
- **Amazon FreeRTOS Hardware** - [AWS FreeRTOS](https://aws.amazon.com/freertos/)
- **Particle IoT** - [Particle](https://www.particle.io/)
- **Arduino IoT Cloud** - [Arduino IoT](https://www.arduino.cc/en/IoT/HomePage)
- **Adafruit Feather** - [Adafruit Feather](https://www.adafruit.com/category/943)
- **Seeed Studio** - [Seeed Studio](https://www.seeedstudio.com/)

#### Real-Time Operating System Hardware
- **RTOS-Compatible MCUs** - Specialized microcontrollers for real-time applications
- **QNX-Compatible Hardware** - [QNX](https://blackberry.qnx.com/)
- **VxWorks Hardware** - [Wind River VxWorks](https://www.windriver.com/products/vxworks)
- **SafeRTOS Hardware** - [WITTENSTEIN SafeRTOS](https://www.highintegritysystems.com/safertos/)

### Desktop & Laptop Hardware

#### Form Factors
- **ATX** - [Wikipedia](https://en.wikipedia.org/wiki/ATX) | [ATX Spec](https://www.intel.com/content/www/us/en/support/articles/000005796/boards-and-kits.html)
- **microATX (mATX)** - [Wikipedia](https://en.wikipedia.org/wiki/MicroATX)
- **Mini-ITX** - [Wikipedia](https://en.wikipedia.org/wiki/Mini-ITX)
- **Extended ATX (EATX)** - [Wikipedia](https://en.wikipedia.org/wiki/Extended_ATX)
- **Nano-ITX** - [Wikipedia](https://en.wikipedia.org/wiki/Mini-ITX#Nano-ITX)
- **Pico-ITX** - [Wikipedia](https://en.wikipedia.org/wiki/Mini-ITX#Pico-ITX)
- **BTX** - [Wikipedia](https://en.wikipedia.org/wiki/BTX_(form_factor))
- **Laptop Form Factors** - [Wikipedia](https://en.wikipedia.org/wiki/Laptop#Form_factors)

#### Motherboards
- **Desktop Motherboards** - [Wikipedia](https://en.wikipedia.org/wiki/Motherboard)
- **Gaming Motherboards** - High-performance consumer boards
- **Workstation Motherboards** - Professional-grade boards
- **Server Motherboards** - Enterprise boards with ECC support

#### Mobile-Specific Components
- **Laptop Batteries** - [Wikipedia](https://en.wikipedia.org/wiki/Laptop#Power_supply)
- **Laptop Displays (eDP)** - [Wikipedia](https://en.wikipedia.org/wiki/DisplayPort#eDP)
- **Laptop Keyboards** - [Wikipedia](https://en.wikipedia.org/wiki/Laptop#Keyboard)
- **TrackPads/Pointing Sticks** - [Wikipedia](https://en.wikipedia.org/wiki/Touchpad)
- **Webcams** - [Wikipedia](https://en.wikipedia.org/wiki/Webcam)
- **MXM Graphics Modules** - [Wikipedia](https://en.wikipedia.org/wiki/Mobile_PCI_Express_Module)
- **SO-DIMM Memory** - [Wikipedia](https://en.wikipedia.org/wiki/SO-DIMM)

### Server Hardware

#### Server Form Factors
- **Rack-Mount Servers** - [Wikipedia](https://en.wikipedia.org/wiki/19-inch_rack)
  - **1U Server** - [Wikipedia](https://en.wikipedia.org/wiki/Rack_unit)
  - **2U Server** - [Wikipedia](https://en.wikipedia.org/wiki/Rack_unit)
  - **4U Server** - [Wikipedia](https://en.wikipedia.org/wiki/Rack_unit)
- **Blade Servers** - [Wikipedia](https://en.wikipedia.org/wiki/Blade_server)
- **Tower Servers** - [Wikipedia](https://en.wikipedia.org/wiki/Server_(computing)#Tower_server)
- **Micro Servers** - [Wikipedia](https://en.wikipedia.org/wiki/Microserver)
- **Dense Servers** - High-density computing platforms

#### Server Processors
- **Intel Xeon Scalable** - [Wikipedia](https://en.wikipedia.org/wiki/Xeon) | [Intel Xeon](https://www.intel.com/content/www/us/en/products/details/processors/xeon.html)
  - **Xeon Platinum** - [Intel Xeon Platinum](https://www.intel.com/content/www/us/en/products/details/processors/xeon/scalable.html)
  - **Xeon Gold** - [Intel Xeon Gold](https://www.intel.com/content/www/us/en/products/details/processors/xeon/scalable.html)
  - **Xeon Silver** - [Intel Xeon Silver](https://www.intel.com/content/www/us/en/products/details/processors/xeon/scalable.html)
  - **Xeon Bronze** - [Intel Xeon Bronze](https://www.intel.com/content/www/us/en/products/details/processors/xeon/scalable.html)
- **AMD EPYC** - [Wikipedia](https://en.wikipedia.org/wiki/Epyc) | [AMD EPYC](https://www.amd.com/en/products/server-processors)
  - **EPYC 9004 Series (Genoa)** - [AMD EPYC Genoa](https://www.amd.com/en/processors/epyc-9004-series)
  - **EPYC 7004 Series (Genoa-X)** - 3D V-Cache variants
  - **EPYC 8004 Series (Siena)** - Edge-optimized processors
- **Ampere Altra** - [Wikipedia](https://en.wikipedia.org/wiki/Ampere_Computing) | [Ampere](https://amperecomputing.com/products/processors)
- **ARM Neoverse** - [ARM Neoverse](https://www.arm.com/products/silicon-ip-cpu/neoverse)
- **IBM POWER10** - [Wikipedia](https://en.wikipedia.org/wiki/POWER10) | [IBM POWER](https://www.ibm.com/products/power10)
- **Oracle SPARC M8** - [Wikipedia](https://en.wikipedia.org/wiki/SPARC_M8) | [Oracle SPARC](https://www.oracle.com/servers/sparc/)

#### Server Memory
- **ECC RDIMM** - [Wikipedia](https://en.wikipedia.org/wiki/Registered_memory)
- **ECC LRDIMM** - [Wikipedia](https://en.wikipedia.org/wiki/Load-reduced_DIMM)
- **NVDIMM** - [Wikipedia](https://en.wikipedia.org/wiki/NVDIMM) | [JEDEC NVDIMM](https://www.jedec.org/standards-documents/docs/jesd245b)
- **Persistent Memory (PMem)** - [Wikipedia](https://en.wikipedia.org/wiki/Persistent_memory) | [Intel Optane PMem](https://www.intel.com/content/www/us/en/products/details/memory-storage/optane-persistent-memory.html)
- **CXL Memory** - [Compute Express Link](https://www.computeexpresslink.org/)

#### RAID Controllers
- **Hardware RAID** - [Wikipedia](https://en.wikipedia.org/wiki/RAID#Hardware_RAID)
- **LSI MegaRAID** - [Broadcom MegaRAID](https://www.broadcom.com/products/storage/raid-controllers)
- **Adaptec RAID** - [Microchip Adaptec](https://www.microchip.com/en-us/products/storage/raid-adapters)
- **Dell PERC** - [Dell PERC](https://www.dell.com/en-us/dt/storage/perc.htm)
- **HPE Smart Array** - [HPE Smart Array](https://www.hpe.com/us/en/servers/smart-array.html)

#### Management Controllers
- **BMC (Baseboard Management Controller)** - [Wikipedia](https://en.wikipedia.org/wiki/Intelligent_Platform_Management_Interface#Baseboard_management_controller)
- **iDRAC (Dell)** - [Wikipedia](https://en.wikipedia.org/wiki/Dell_DRAC) | [Dell iDRAC](https://www.dell.com/support/kbdoc/en-us/000134867/what-is-the-idrac)
- **iLO (HPE)** - [Wikipedia](https://en.wikipedia.org/wiki/HP_Integrated_Lights-Out) | [HPE iLO](https://www.hpe.com/us/en/servers/integrated-lights-out-ilo.html)
- **IMM (Lenovo)** - [Lenovo XClarity](https://www.lenovo.com/us/en/servers-storage/solutions/systems-management/xclarity/)

### HPC & Supercomputing

#### HPC Interconnects
- **InfiniBand HDR** - [Wikipedia](https://en.wikipedia.org/wiki/InfiniBand#HDR) | [InfiniBand Trade Association](https://www.infinibandta.org/)
- **InfiniBand NDR** - Next-generation 400Gb/s InfiniBand
- **Intel Omni-Path** - [Wikipedia](https://en.wikipedia.org/wiki/Omni-Path) | [Intel Omni-Path](https://www.intel.com/content/www/us/en/products/details/fabric/omni-path.html)
- **NVIDIA NVLink** - [Wikipedia](https://en.wikipedia.org/wiki/NVLink) | [NVIDIA NVLink](https://www.nvidia.com/en-us/data-center/nvlink/)
- **NVSwitch** - [NVIDIA NVSwitch](https://www.nvidia.com/en-us/data-center/nvswitch/)
- **Slingshot (HPE)** - [HPE Slingshot](https://www.hpe.com/us/en/compute/hpc/slingshot-interconnect.html)
- **Tofu Interconnect (Fujitsu)** - [Fujitsu Tofu](https://www.fujitsu.com/global/about/resources/news/press-releases/2019/0506-01.html)

#### Accelerator Technologies
- **NVIDIA A100** - [Wikipedia](https://en.wikipedia.org/wiki/Ampere_(microarchitecture)#GPUs) | [NVIDIA A100](https://www.nvidia.com/en-us/data-center/a100/)
- **NVIDIA H100** - [NVIDIA H100](https://www.nvidia.com/en-us/data-center/h100/)
- **NVIDIA GH200** - Grace Hopper Superchip | [NVIDIA GH200](https://www.nvidia.com/en-us/data-center/grace-hopper-superchip/)
- **AMD Instinct MI300** - [AMD Instinct](https://www.amd.com/en/products/accelerators/instinct/mi300)
- **AMD Instinct MI250X** - [AMD MI250X](https://www.amd.com/en/products/server-accelerators/instinct-mi250x)
- **Intel Data Center GPU Max** - [Intel Max Series](https://www.intel.com/content/www/us/en/products/details/discrete-gpus/data-center-gpu.html)
- **Graphcore Bow IPU** - [Graphcore](https://www.graphcore.ai/products/ipu)
- **Cerebras WSE-3** - [Cerebras Wafer-Scale Engine](https://www.cerebras.net/product-chip/)
- **SambaNova DataScale** - [SambaNova](https://sambanova.ai/products/datascale/)

#### Supercomputer Architectures
- **Cray Supercomputers** - [Wikipedia](https://en.wikipedia.org/wiki/Cray) | [HPE Cray](https://www.hpe.com/us/en/compute/hpc/supercomputing.html)
- **Fugaku** - [Wikipedia](https://en.wikipedia.org/wiki/Fugaku_(supercomputer)) | [RIKEN Fugaku](https://www.r-ccs.riken.jp/en/fugaku)
- **Frontier (AMD-based)** - [Wikipedia](https://en.wikipedia.org/wiki/Frontier_(supercomputer))
- **Aurora (Intel-based)** - [Wikipedia](https://en.wikipedia.org/wiki/Aurora_(supercomputer))
- **Summit** - [Wikipedia](https://en.wikipedia.org/wiki/Summit_(supercomputer))
- **Sierra** - [Wikipedia](https://en.wikipedia.org/wiki/Sierra_(supercomputer))
- **TOP500 Systems** - [TOP500](https://www.top500.org/)

### Mobile Devices

#### Mobile SoCs
- **Apple A17 Pro** - [Wikipedia](https://en.wikipedia.org/wiki/Apple_A17) | [Apple iPhone](https://www.apple.com/iphone/)
- **Apple M3** - [Wikipedia](https://en.wikipedia.org/wiki/Apple_M3) | [Apple Mac](https://www.apple.com/mac/)
- **Qualcomm Snapdragon 8 Gen 3** - [Qualcomm](https://www.qualcomm.com/products/mobile/snapdragon/smartphones)
- **MediaTek Dimensity 9300** - [MediaTek](https://www.mediatek.com/products/smartphones/mediatek-dimensity-9300)
- **Samsung Exynos 2400** - [Samsung Exynos](https://www.samsung.com/semiconductor/minisite/exynos/)
- **Google Tensor G3** - [Wikipedia](https://en.wikipedia.org/wiki/Google_Tensor) | [Google Pixel](https://store.google.com/product/pixel_8)
- **Huawei Kirin** - [Wikipedia](https://en.wikipedia.org/wiki/HiSilicon#Kirin_application_processors)

#### Mobile Memory
- **LPDDR5X** - [Wikipedia](https://en.wikipedia.org/wiki/LPDDR#LPDDR5X) | [JEDEC LPDDR5X](https://www.jedec.org/standards-documents/docs/jesd209-5b)
- **UFS 4.0** - [Wikipedia](https://en.wikipedia.org/wiki/Universal_Flash_Storage) | [JEDEC UFS 4.0](https://www.jedec.org/standards-documents/docs/jesd220e-01)
- **eMMC** - [Wikipedia](https://en.wikipedia.org/wiki/MultiMediaCard#eMMC) | [JEDEC eMMC](https://www.jedec.org/standards-documents/docs/jesd84-b51)

#### Mobile Display Technologies
- **AMOLED** - [Wikipedia](https://en.wikipedia.org/wiki/AMOLED)
- **LTPO (Low-Temperature Polycrystalline Oxide)** - [Wikipedia](https://en.wikipedia.org/wiki/LTPO)
- **ProMotion (Apple)** - Adaptive refresh rate displays
- **Gorilla Glass** - [Wikipedia](https://en.wikipedia.org/wiki/Gorilla_Glass) | [Corning Gorilla Glass](https://www.corning.com/gorillaglass/)
- **Ceramic Shield** - [Apple Ceramic Shield](https://www.apple.com/newsroom/2020/10/apple-introduces-iphone-12-pro-and-iphone-12-pro-max-with-5g/)

### Edge Computing

#### Edge Processors
- **Intel Atom (Edge)** - [Intel Atom](https://www.intel.com/content/www/us/en/products/details/processors/atom.html)
- **AMD Ryzen Embedded** - [Wikipedia](https://en.wikipedia.org/wiki/List_of_AMD_Ryzen_processors#Ryzen_Embedded) | [AMD Embedded](https://www.amd.com/en/products/embedded)
- **NXP Layerscape** - [NXP Layerscape](https://www.nxp.com/products/processors-and-microcontrollers/arm-processors/layerscape-processors:LAYERSCAPE)
- **Marvell OCTEON** - [Marvell OCTEON](https://www.marvell.com/products/infrastructure-processors.html)
- **Xilinx Zynq** - [Wikipedia](https://en.wikipedia.org/wiki/Xilinx_Zynq) | [AMD Xilinx Zynq](https://www.xilinx.com/products/silicon-devices/soc/zynq-7000.html)

#### Edge AI Accelerators
- **Google Coral** - [Wikipedia](https://en.wikipedia.org/wiki/Tensor_Processing_Unit#Edge_TPU) | [Google Coral](https://coral.ai/)
- **Intel Movidius** - [Wikipedia](https://en.wikipedia.org/wiki/Movidius) | [Intel Movidius](https://www.intel.com/content/www/us/en/products/details/processors/movidius-vpu.html)
- **NVIDIA Jetson Orin** - [NVIDIA Jetson Orin](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/)
- **Hailo AI Accelerators** - [Hailo](https://hailo.ai/)
- **Kneron AI Chips** - [Kneron](https://www.kneron.com/)

## Industry-Specific Hardware

### Automotive Electronics

#### Automotive Processors
- **NVIDIA DRIVE** - [Wikipedia](https://en.wikipedia.org/wiki/Nvidia_Drive) | [NVIDIA DRIVE](https://www.nvidia.com/en-us/self-driving-cars/)
- **Qualcomm Snapdragon Ride** - [Qualcomm Automotive](https://www.qualcomm.com/products/automotive/snapdragon-ride-platform)
- **Tesla FSD Computer** - [Wikipedia](https://en.wikipedia.org/wiki/Tesla_Autopilot#Hardware_3)
- **Mobileye EyeQ** - [Wikipedia](https://en.wikipedia.org/wiki/Mobileye) | [Mobileye EyeQ](https://www.mobileye.com/our-technology/evolution-eyeq-chip/)
- **Renesas R-Car** - [Wikipedia](https://en.wikipedia.org/wiki/Renesas_Electronics) | [Renesas R-Car](https://www.renesas.com/us/en/products/automotive-products/automotive-system-chips-socs/r-car-automotive-socs)
- **NXP S32** - [NXP S32](https://www.nxp.com/products/processors-and-microcontrollers/s32-automotive-platform:S32)
- **Infineon AURIX** - [Infineon AURIX](https://www.infineon.com/cms/en/product/microcontroller/32-bit-tricore-microcontroller/32-bit-tricore-aurix-tc4x/)
- **TI Jacinto** - [TI Jacinto](https://www.ti.com/processors/automotive-processors/overview.html)

#### ADAS & Autonomous Driving
- **LiDAR Processors** - [Wikipedia](https://en.wikipedia.org/wiki/Lidar)
- **Radar SoCs** - Automotive radar processing
- **Vision Processing Units** - Camera ISP and neural processing
- **Sensor Fusion Processors** - Multi-sensor data fusion

#### Vehicle Networks
- **CAN Bus** - [Wikipedia](https://en.wikipedia.org/wiki/CAN_bus) | [CAN in Automation](https://www.can-cia.org/)
- **CAN FD** - [Wikipedia](https://en.wikipedia.org/wiki/CAN_FD)
- **FlexRay** - [Wikipedia](https://en.wikipedia.org/wiki/FlexRay) | [FlexRay Consortium](https://www.flexray.com/)
- **LIN Bus** - [Wikipedia](https://en.wikipedia.org/wiki/Local_Interconnect_Network)
- **MOST** - [Wikipedia](https://en.wikipedia.org/wiki/Media_Oriented_Systems_Transport)
- **Automotive Ethernet** - [Wikipedia](https://en.wikipedia.org/wiki/Automotive_Ethernet)

#### Automotive Sensors
- **Camera Modules** - Automotive-grade image sensors
- **Ultrasonic Sensors** - Parking and proximity detection
- **Pressure Sensors** - TPMS and engine monitoring
- **IMU (Inertial Measurement Unit)** - [Wikipedia](https://en.wikipedia.org/wiki/Inertial_measurement_unit)

### Industrial Electronics

#### Industrial Controllers
- **PLC (Programmable Logic Controller)** - [Wikipedia](https://en.wikipedia.org/wiki/Programmable_logic_controller)
- **PAC (Programmable Automation Controller)** - [Wikipedia](https://en.wikipedia.org/wiki/Programmable_automation_controller)
- **DCS (Distributed Control System)** - [Wikipedia](https://en.wikipedia.org/wiki/Distributed_control_system)
- **SCADA Systems** - [Wikipedia](https://en.wikipedia.org/wiki/SCADA)
- **Siemens SIMATIC** - [Siemens SIMATIC](https://new.siemens.com/global/en/products/automation/systems/industrial.html)
- **Allen-Bradley** - [Rockwell Automation](https://www.rockwellautomation.com/)
- **Schneider Electric Modicon** - [Schneider Electric](https://www.se.com/ww/en/)

#### Industrial Communication
- **PROFINET** - [Wikipedia](https://en.wikipedia.org/wiki/PROFINET) | [PROFIBUS](https://www.profibus.com/)
- **EtherCAT** - [Wikipedia](https://en.wikipedia.org/wiki/EtherCAT) | [EtherCAT Technology Group](https://www.ethercat.org/)
- **Modbus** - [Wikipedia](https://en.wikipedia.org/wiki/Modbus) | [Modbus Organization](https://modbus.org/)
- **CC-Link** - [Wikipedia](https://en.wikipedia.org/wiki/CC-Link) | [CC-Link Partner Association](https://www.cc-link.org/)
- **POWERLINK** - [Wikipedia](https://en.wikipedia.org/wiki/POWERLINK) | [Ethernet POWERLINK](https://www.ethernet-powerlink.org/)
- **SERCOS III** - [Wikipedia](https://en.wikipedia.org/wiki/SERCOS_III)
- **IO-Link** - [Wikipedia](https://en.wikipedia.org/wiki/IO-Link) | [IO-Link Community](https://io-link.com/)

#### Industrial Sensors
- **Proximity Sensors** - Inductive, capacitive, photoelectric
- **Temperature Sensors** - Thermocouples, RTDs, thermistors
- **Flow Sensors** - Mass flow, volumetric flow
- **Level Sensors** - Ultrasonic, radar, capacitive
- **Vision Systems** - Industrial cameras and processors

### Aerospace & Defense

#### Radiation-Hardened Components
- **Rad-Hard Processors** - [Wikipedia](https://en.wikipedia.org/wiki/Radiation_hardening)
- **BAE RAD750** - [Wikipedia](https://en.wikipedia.org/wiki/RAD750)
- **RAD5545** - Next-generation space processor
- **Microchip SAMRH71** - Radiation-hardened ARM Cortex-M7
- **Cobham Gaisler LEON** - [LEON Processor](https://www.gaisler.com/index.php/products/processors)

#### Avionics Systems
- **Flight Control Computers** - [Wikipedia](https://en.wikipedia.org/wiki/Flight_control_computer)
- **Mission Computers** - Integrated avionics processing
- **ARINC Standards** - [Wikipedia](https://en.wikipedia.org/wiki/ARINC)
  - **ARINC 429** - Aviation data bus
  - **ARINC 664 (AFDX)** - Avionics Full-Duplex Switched Ethernet
  - **ARINC 653** - Avionics Application Software Standard Interface

#### Military Standards
- **MIL-STD-1553** - [Wikipedia](https://en.wikipedia.org/wiki/MIL-STD-1553) - Military data bus
- **MIL-STD-461** - EMC requirements
- **MIL-STD-810** - [Wikipedia](https://en.wikipedia.org/wiki/MIL-STD-810) - Environmental testing
- **JEDS (Joint Electron Device Engineering Council)** - Military semiconductor standards

### Medical Devices

#### Medical Imaging Hardware
- **MRI Processors** - [Wikipedia](https://en.wikipedia.org/wiki/Magnetic_resonance_imaging)
- **CT Scanner Electronics** - [Wikipedia](https://en.wikipedia.org/wiki/CT_scan)
- **Ultrasound Processors** - [Wikipedia](https://en.wikipedia.org/wiki/Medical_ultrasound)
- **PET Scan Electronics** - [Wikipedia](https://en.wikipedia.org/wiki/Positron_emission_tomography)
- **X-Ray Detectors** - Digital radiography sensors

#### Patient Monitoring
- **ECG/EKG Processors** - [Wikipedia](https://en.wikipedia.org/wiki/Electrocardiography)
- **Pulse Oximeter Chips** - [Wikipedia](https://en.wikipedia.org/wiki/Pulse_oximetry)
- **Blood Pressure Monitors** - Electronic sphygmomanometers
- **Vital Signs Monitors** - Multi-parameter monitoring

#### Medical Safety Standards
- **IEC 60601** - [Wikipedia](https://en.wikipedia.org/wiki/IEC_60601) - Medical electrical equipment safety
- **ISO 13485** - [Wikipedia](https://en.wikipedia.org/wiki/ISO_13485) - Medical devices quality management
- **FDA 21 CFR Part 11** - Electronic records compliance

### Telecommunications

#### Baseband Processors
- **Qualcomm Snapdragon X Series** - 5G modems | [Qualcomm 5G](https://www.qualcomm.com/products/mobile/5g)
- **MediaTek 5G Modems** - [MediaTek 5G](https://www.mediatek.com/innovations/5g)
- **Intel XMM Modems** - Cellular modem chipsets
- **Exynos Modems (Samsung)** - Integrated 5G modems

#### RF Components
- **RF Power Amplifiers** - [Wikipedia](https://en.wikipedia.org/wiki/RF_power_amplifier)
- **RF Transceivers** - Transmit/receive radio modules
- **Antenna Tuners** - Impedance matching systems
- **Duplexers/Diplexers** - RF signal separation
- **SAW Filters** - [Wikipedia](https://en.wikipedia.org/wiki/Surface_acoustic_wave_filter)
- **BAW Filters** - [Wikipedia](https://en.wikipedia.org/wiki/Thin-film_bulk_acoustic_resonator)

#### Telecom Standards
- **3GPP Standards** - [Wikipedia](https://en.wikipedia.org/wiki/3GPP) | [3GPP](https://www.3gpp.org/)
- **O-RAN** - [Wikipedia](https://en.wikipedia.org/wiki/O-RAN_Alliance) | [O-RAN Alliance](https://www.o-ran.org/)
- **CPRI** - [Wikipedia](https://en.wikipedia.org/wiki/Common_Public_Radio_Interface)
- **eCPRI** - Enhanced CPRI for 5G

## Fabrication Processes

### Current Process Nodes
- **2nm** - [Wikipedia](https://en.wikipedia.org/wiki/2_nm_process) | [TSMC 2nm](https://www.tsmc.com/english/dedicatedFoundry/technology/logic/l_2nm)
- **3nm** - [Wikipedia](https://en.wikipedia.org/wiki/3_nm_process) | [TSMC 3nm](https://www.tsmc.com/english/dedicatedFoundry/technology/logic/l_3nm)
- **4nm** - [Wikipedia](https://en.wikipedia.org/wiki/4_nm_process) | [TSMC 4nm](https://www.tsmc.com/english/dedicatedFoundry/technology/logic/l_4nm)
- **5nm** - [Wikipedia](https://en.wikipedia.org/wiki/5_nm_process) | [TSMC 5nm](https://www.tsmc.com/english/dedicatedFoundry/technology/logic/l_5nm)
- **6nm** - [Wikipedia](https://en.wikipedia.org/wiki/6_nm_process)
- **7nm** - [Wikipedia](https://en.wikipedia.org/wiki/7_nm_process) | [TSMC 7nm](https://www.tsmc.com/english/dedicatedFoundry/technology/logic/l_7nm)

### Mature Process Nodes
- **10nm** - [Wikipedia](https://en.wikipedia.org/wiki/10_nm_process)
- **12nm** - [Wikipedia](https://en.wikipedia.org/wiki/12_nm_process)
- **14nm** - [Wikipedia](https://en.wikipedia.org/wiki/14_nm_process) | [Intel 14nm](https://www.intel.com/content/www/us/en/silicon-innovations/intel-14nm-technology.html)
- **16nm** - [Wikipedia](https://en.wikipedia.org/wiki/16_nm_process)
- **20nm** - [Wikipedia](https://en.wikipedia.org/wiki/20_nm_process)
- **22nm** - [Wikipedia](https://en.wikipedia.org/wiki/22_nm_process)
- **28nm** - [Wikipedia](https://en.wikipedia.org/wiki/28_nm_process)
- **32nm** - [Wikipedia](https://en.wikipedia.org/wiki/32_nm_process)
- **40nm** - [Wikipedia](https://en.wikipedia.org/wiki/40_nm_process)
- **45nm** - [Wikipedia](https://en.wikipedia.org/wiki/45_nm_process)
- **65nm** - [Wikipedia](https://en.wikipedia.org/wiki/65_nm_process)
- **90nm** - [Wikipedia](https://en.wikipedia.org/wiki/90_nm_process)
- **130nm** - [Wikipedia](https://en.wikipedia.org/wiki/130_nm_process)
- **180nm** - [Wikipedia](https://en.wikipedia.org/wiki/180_nm_process)
- **250nm** - [Wikipedia](https://en.wikipedia.org/wiki/250_nm_process)
- **350nm** - [Wikipedia](https://en.wikipedia.org/wiki/350_nm_process)
- **500nm** - [Wikipedia](https://en.wikipedia.org/wiki/500_nm_process)

### Future Process Nodes
- **1.4nm** - [Wikipedia](https://en.wikipedia.org/wiki/1.4_nm_process)
- **1nm** - [Wikipedia](https://en.wikipedia.org/wiki/1_nm_process)
- **Sub-1nm** - Research stage processes

### Manufacturing Technologies
- **FinFET** - [Wikipedia](https://en.wikipedia.org/wiki/FinFET)
- **GAA (Gate-All-Around)** - [Wikipedia](https://en.wikipedia.org/wiki/Gate-all-around_FET)
- **EUV Lithography** - [Wikipedia](https://en.wikipedia.org/wiki/Extreme_ultraviolet_lithography) | [ASML EUV](https://www.asml.com/en/products/euv-lithography-systems)
- **High-K Dielectric** - [Wikipedia](https://en.wikipedia.org/wiki/High-%CE%BA_dielectric)
- **Metal Gate** - [Wikipedia](https://en.wikipedia.org/wiki/Metal_gate)
- **Strained Silicon** - [Wikipedia](https://en.wikipedia.org/wiki/Strained_silicon)
- **Silicon on Insulator (SOI)** - [Wikipedia](https://en.wikipedia.org/wiki/Silicon_on_insulator)

### Foundries
- **TSMC (Taiwan Semiconductor)** - [Wikipedia](https://en.wikipedia.org/wiki/TSMC) | [TSMC](https://www.tsmc.com/)
- **Samsung Foundry** - [Wikipedia](https://en.wikipedia.org/wiki/Samsung_Foundry) | [Samsung Foundry](https://www.samsung.com/semiconductor/minisite/foundry/)
- **Intel Foundry Services** - [Wikipedia](https://en.wikipedia.org/wiki/Intel_Foundry_Services) | [Intel Foundry](https://www.intel.com/content/www/us/en/foundry/overview.html)
- **GlobalFoundries** - [Wikipedia](https://en.wikipedia.org/wiki/GlobalFoundries) | [GlobalFoundries](https://gf.com/)
- **UMC (United Microelectronics)** - [Wikipedia](https://en.wikipedia.org/wiki/United_Microelectronics_Corporation) | [UMC](https://www.umc.com/)
- **SMIC (Semiconductor Manufacturing International)** - [Wikipedia](https://en.wikipedia.org/wiki/Semiconductor_Manufacturing_International_Corporation) | [SMIC](https://www.smics.com/)
- **Tower Semiconductor** - [Wikipedia](https://en.wikipedia.org/wiki/Tower_Semiconductor) | [Tower](https://towersemi.com/)
- **Skywater** - Open-source foundry | [Skywater PDK](https://github.com/google/skywater-pdk)

### Packaging Technologies
- **BGA (Ball Grid Array)** - [Wikipedia](https://en.wikipedia.org/wiki/Ball_grid_array)
- **LGA (Land Grid Array)** - [Wikipedia](https://en.wikipedia.org/wiki/Land_grid_array)
- **PGA (Pin Grid Array)** - [Wikipedia](https://en.wikipedia.org/wiki/Pin_grid_array)
- **QFN (Quad Flat No-lead)** - [Wikipedia](https://en.wikipedia.org/wiki/Flat_no-leads_package)
- **QFP (Quad Flat Package)** - [Wikipedia](https://en.wikipedia.org/wiki/Quad_Flat_Package)
- **TSOP (Thin Small Outline Package)** - [Wikipedia](https://en.wikipedia.org/wiki/Small_Outline_Integrated_Circuit)
- **CSP (Chip-Scale Package)** - [Wikipedia](https://en.wikipedia.org/wiki/Chip-scale_package)
- **WLCSP (Wafer-Level Chip-Scale Package)** - [Wikipedia](https://en.wikipedia.org/wiki/Wafer-level_packaging)
- **3D Packaging** - [Wikipedia](https://en.wikipedia.org/wiki/Three-dimensional_integrated_circuit)
- **Through-Silicon Via (TSV)** - [Wikipedia](https://en.wikipedia.org/wiki/Through-silicon_via)
- **Chiplets** - [Wikipedia](https://en.wikipedia.org/wiki/Chiplet)
- **2.5D Integration** - [Wikipedia](https://en.wikipedia.org/wiki/2.5D_interposer)
- **Fan-Out Wafer-Level Packaging (FOWLP)** - [Wikipedia](https://en.wikipedia.org/wiki/Wafer-level_packaging#Fan-out_wafer-level_packaging)
- **Flip Chip** - [Wikipedia](https://en.wikipedia.org/wiki/Flip_chip)
- **Wire Bonding** - [Wikipedia](https://en.wikipedia.org/wiki/Wire_bonding)

## Memory Technologies

### Volatile Memory

#### DRAM Types
- **DDR5 SDRAM** - [Wikipedia](https://en.wikipedia.org/wiki/DDR5_SDRAM) | [JEDEC DDR5 Spec](https://www.jedec.org/standards-documents/docs/jesd79-5)
- **DDR4 SDRAM** - [Wikipedia](https://en.wikipedia.org/wiki/DDR4_SDRAM) | [JEDEC DDR4 Spec](https://www.jedec.org/standards-documents/docs/jesd79-4a)
- **DDR3 SDRAM** - [Wikipedia](https://en.wikipedia.org/wiki/DDR3_SDRAM)
- **DDR2 SDRAM** - [Wikipedia](https://en.wikipedia.org/wiki/DDR2_SDRAM)
- **DDR SDRAM** - [Wikipedia](https://en.wikipedia.org/wiki/DDR_SDRAM)
- **SDRAM** - [Wikipedia](https://en.wikipedia.org/wiki/SDRAM)
- **LPDDR5** - [Wikipedia](https://en.wikipedia.org/wiki/LPDDR#LPDDR5) | [JEDEC LPDDR5 Spec](https://www.jedec.org/standards-documents/docs/jesd209-5)
- **LPDDR5X** - [Wikipedia](https://en.wikipedia.org/wiki/LPDDR#LPDDR5X)
- **LPDDR4** - [Wikipedia](https://en.wikipedia.org/wiki/LPDDR#LPDDR4)
- **GDDR6** - [Wikipedia](https://en.wikipedia.org/wiki/GDDR6_SDRAM) | [JEDEC GDDR6 Spec](https://www.jedec.org/standards-documents/docs/jesd250c)
- **GDDR6X** - [Wikipedia](https://en.wikipedia.org/wiki/GDDR6X_SDRAM)
- **GDDR7** - Next-generation graphics memory
- **HBM3 (High Bandwidth Memory)** - [Wikipedia](https://en.wikipedia.org/wiki/High_Bandwidth_Memory) | [JEDEC HBM3 Spec](https://www.jedec.org/standards-documents/docs/jesd238)
- **HBM2E** - [Wikipedia](https://en.wikipedia.org/wiki/High_Bandwidth_Memory#HBM2E)
- **HBM2** - [Wikipedia](https://en.wikipedia.org/wiki/High_Bandwidth_Memory#HBM2)

#### SRAM Types
- **SRAM** - [Wikipedia](https://en.wikipedia.org/wiki/Static_random-access_memory)
- **Asynchronous SRAM** - Traditional async SRAM
- **Synchronous SRAM** - Clocked SRAM
- **Pipeline Burst SRAM** - High-speed cache SRAM
- **MRAM (as Cache)** - [Wikipedia](https://en.wikipedia.org/wiki/Magnetoresistive_random-access_memory)

### Non-Volatile Memory

#### Flash Memory
- **3D NAND Flash** - [Wikipedia](https://en.wikipedia.org/wiki/3D_NAND_flash_memory)
- **QLC NAND** - [Wikipedia](https://en.wikipedia.org/wiki/Flash_memory#Multi-level_cell)
- **TLC NAND** - [Wikipedia](https://en.wikipedia.org/wiki/Flash_memory#Multi-level_cell)
- **MLC NAND** - [Wikipedia](https://en.wikipedia.org/wiki/Flash_memory#Multi-level_cell)
- **SLC NAND** - [Wikipedia](https://en.wikipedia.org/wiki/Flash_memory#Single-level_cell)
- **PLC (Penta-Level Cell)** - Future 5-bit per cell NAND
- **V-NAND (Samsung)** - [Wikipedia](https://en.wikipedia.org/wiki/V-NAND) - 3D vertical NAND
- **BiCS (Toshiba/Kioxia)** - [Wikipedia](https://en.wikipedia.org/wiki/3D_NAND#BiCS) - 3D bit-cost scaling
- **NOR Flash** - [Wikipedia](https://en.wikipedia.org/wiki/Flash_memory#NOR_flash)
- **Serial NOR** - SPI/Quad-SPI NOR Flash

#### Emerging NVM
- **3D XPoint (Optane)** - [Wikipedia](https://en.wikipedia.org/wiki/3D_XPoint)
- **ReRAM** - [Wikipedia](https://en.wikipedia.org/wiki/Resistive_random-access_memory)
- **MRAM** - [Wikipedia](https://en.wikipedia.org/wiki/Magnetoresistive_random-access_memory)
- **STT-MRAM** - [Wikipedia](https://en.wikipedia.org/wiki/Magnetoresistive_random-access_memory#Spin-transfer_torque_MRAM)
- **PCM (Phase-Change Memory)** - [Wikipedia](https://en.wikipedia.org/wiki/Phase-change_memory)
- **FeRAM** - [Wikipedia](https://en.wikipedia.org/wiki/Ferroelectric_RAM)
- **CBRAM (Conductive-Bridging RAM)** - [Wikipedia](https://en.wikipedia.org/wiki/Conductive-bridging_RAM)
- **NRAM (Nano-RAM)** - [Wikipedia](https://en.wikipedia.org/wiki/Nano-RAM)
- **Racetrack Memory** - [Wikipedia](https://en.wikipedia.org/wiki/Racetrack_memory)

### Cache Architectures
- **L1 Cache** - [Wikipedia](https://en.wikipedia.org/wiki/CPU_cache#L1_cache) - Primary cache
- **L2 Cache** - [Wikipedia](https://en.wikipedia.org/wiki/CPU_cache#L2_cache) - Secondary cache
- **L3 Cache** - [Wikipedia](https://en.wikipedia.org/wiki/CPU_cache#L3_cache) - Last-level cache
- **L4 Cache** - [Wikipedia](https://en.wikipedia.org/wiki/CPU_cache#L4_cache) - eDRAM/3D stacked cache
- **Victim Cache** - [Wikipedia](https://en.wikipedia.org/wiki/CPU_cache#Victim_cache)
- **Trace Cache** - [Wikipedia](https://en.wikipedia.org/wiki/CPU_cache#Trace_cache)
- **3D V-Cache (AMD)** - [Wikipedia](https://en.wikipedia.org/wiki/3D_V-Cache) - Stacked L3 cache
- **eDRAM Cache (Intel Iris Pro)** - [Wikipedia](https://en.wikipedia.org/wiki/EDRAM) - Embedded DRAM cache
- **Inclusive Cache** - [Wikipedia](https://en.wikipedia.org/wiki/CPU_cache#Policies)
- **Exclusive Cache** - [Wikipedia](https://en.wikipedia.org/wiki/CPU_cache#Policies)
- **VIPT (Virtually Indexed, Physically Tagged)** - Cache addressing scheme
- **PIPT (Physically Indexed, Physically Tagged)** - Cache addressing scheme

## Storage Technologies

### Storage Media

#### Solid State
- **SSD (Solid State Drive)** - [Wikipedia](https://en.wikipedia.org/wiki/Solid-state_drive)
- **NVMe SSD** - [Wikipedia](https://en.wikipedia.org/wiki/NVM_Express)
- **SATA SSD** - [Wikipedia](https://en.wikipedia.org/wiki/Serial_ATA)
- **M.2 SSD** - [Wikipedia](https://en.wikipedia.org/wiki/M.2)
- **U.2 SSD** - [Wikipedia](https://en.wikipedia.org/wiki/U.2)
- **U.3 SSD** - [Wikipedia](https://en.wikipedia.org/wiki/U.2#U.3) - Tri-mode support
- **Enterprise SSD** - High-endurance drives
- **Consumer SSD** - Desktop/laptop drives
- **SD Cards** - [Wikipedia](https://en.wikipedia.org/wiki/SD_card)
  - **SDXC** - [Wikipedia](https://en.wikipedia.org/wiki/SD_card#SDXC)
  - **SDHC** - [Wikipedia](https://en.wikipedia.org/wiki/SD_card#SDHC)
  - **SDUC** - [Wikipedia](https://en.wikipedia.org/wiki/SD_card#SDUC) - Ultra Capacity
  - **microSD** - [Wikipedia](https://en.wikipedia.org/wiki/SD_card#microSD)
- **CompactFlash** - [Wikipedia](https://en.wikipedia.org/wiki/CompactFlash)
- **CFexpress** - [Wikipedia](https://en.wikipedia.org/wiki/CFexpress)

#### Magnetic
- **HDD (Hard Disk Drive)** - [Wikipedia](https://en.wikipedia.org/wiki/Hard_disk_drive)
- **SMR (Shingled Magnetic Recording)** - [Wikipedia](https://en.wikipedia.org/wiki/Shingled_magnetic_recording)
- **CMR (Conventional Magnetic Recording)** - [Wikipedia](https://en.wikipedia.org/wiki/Perpendicular_recording)
- **PMR (Perpendicular Magnetic Recording)** - [Wikipedia](https://en.wikipedia.org/wiki/Perpendicular_recording)
- **HAMR (Heat-Assisted Magnetic Recording)** - [Wikipedia](https://en.wikipedia.org/wiki/Heat-assisted_magnetic_recording)
- **MAMR (Microwave-Assisted Magnetic Recording)** - [Wikipedia](https://en.wikipedia.org/wiki/Microwave-assisted_magnetic_recording)
- **Helium-Filled HDDs** - [Wikipedia](https://en.wikipedia.org/wiki/Hard_disk_drive#Helium-filled_drives)
- **Enterprise HDD** - Data center drives
- **NAS HDD** - Network-attached storage optimized
- **Surveillance HDD** - 24/7 recording optimized

#### Optical
- **Blu-ray** - [Wikipedia](https://en.wikipedia.org/wiki/Blu-ray)
  - **BD-R** - [Wikipedia](https://en.wikipedia.org/wiki/Blu-ray#BD-R) - Recordable
  - **BD-RE** - [Wikipedia](https://en.wikipedia.org/wiki/Blu-ray#BD-RE) - Rewritable
  - **BDXL** - [Wikipedia](https://en.wikipedia.org/wiki/Blu-ray#BDXL) - Extended capacity
- **DVD** - [Wikipedia](https://en.wikipedia.org/wiki/DVD)
  - **DVD-R/+R** - [Wikipedia](https://en.wikipedia.org/wiki/DVD-R)
  - **DVD-RW/+RW** - [Wikipedia](https://en.wikipedia.org/wiki/DVD-RW)
- **CD** - [Wikipedia](https://en.wikipedia.org/wiki/Compact_disc)
  - **CD-R** - [Wikipedia](https://en.wikipedia.org/wiki/CD-R)
  - **CD-RW** - [Wikipedia](https://en.wikipedia.org/wiki/CD-RW)
- **M-DISC** - [Wikipedia](https://en.wikipedia.org/wiki/M-DISC) - Archival optical media
- **Holographic Storage** - [Wikipedia](https://en.wikipedia.org/wiki/Holographic_data_storage)
- **Archival Disc** - [Wikipedia](https://en.wikipedia.org/wiki/Archival_Disc)

#### Tape
- **LTO (Linear Tape-Open)** - [Wikipedia](https://en.wikipedia.org/wiki/Linear_Tape-Open)
  - **LTO-9** - [Wikipedia](https://en.wikipedia.org/wiki/Linear_Tape-Open#LTO-9) - 18TB native
  - **LTO-8** - [Wikipedia](https://en.wikipedia.org/wiki/Linear_Tape-Open#LTO-8) - 12TB native
  - **LTO-7** - [Wikipedia](https://en.wikipedia.org/wiki/Linear_Tape-Open#LTO-7) - 6TB native
- **IBM 3592** - [Wikipedia](https://en.wikipedia.org/wiki/IBM_3592) - Enterprise tape
- **Oracle T10000** - [Wikipedia](https://en.wikipedia.org/wiki/StorageTek_tape_formats#T10000) - Enterprise tape
- **DAT (Digital Audio Tape)** - [Wikipedia](https://en.wikipedia.org/wiki/Digital_Audio_Tape)
- **DDS (Digital Data Storage)** - [Wikipedia](https://en.wikipedia.org/wiki/Digital_Data_Storage)

### Storage Interfaces
- **NVMe** - [Wikipedia](https://en.wikipedia.org/wiki/NVM_Express) | [NVMe Spec](https://nvmexpress.org/specifications/)
- **SATA** - [Wikipedia](https://en.wikipedia.org/wiki/Serial_ATA) | [SATA-IO Specs](https://www.sata-io.org/developers/purchase-specification)
- **SAS (Serial Attached SCSI)** - [Wikipedia](https://en.wikipedia.org/wiki/Serial_Attached_SCSI) | [SAS Specs](https://www.t10.org/scsi-3.htm)
- **U.2** - [Wikipedia](https://en.wikipedia.org/wiki/U.2)
- **M.2** - [Wikipedia](https://en.wikipedia.org/wiki/M.2) | [M.2 Spec](https://www.jedec.org/standards-documents/docs/mo-297)
- **eUFS** - [Wikipedia](https://en.wikipedia.org/wiki/Universal_Flash_Storage) | [JEDEC UFS Spec](https://www.jedec.org/standards-documents/docs/jesd220e)

### Storage Protocols
- **SCSI** - [Wikipedia](https://en.wikipedia.org/wiki/SCSI) | [T10 SCSI Specs](https://www.t10.org/scsi-3.htm)
- **iSCSI** - [Wikipedia](https://en.wikipedia.org/wiki/ISCSI) | [RFC 7143](https://tools.ietf.org/html/rfc7143)
- **Fibre Channel** - [Wikipedia](https://en.wikipedia.org/wiki/Fibre_Channel) | [T11 FC Specs](https://www.t11.org/fc-bb-5/documents/)
- **NVMe-oF (NVMe over Fabrics)** - [Wikipedia](https://en.wikipedia.org/wiki/NVM_Express#NVMe-oF) | [NVMe-oF Spec](https://nvmexpress.org/specifications/)
- **ATA/ATAPI** - [Wikipedia](https://en.wikipedia.org/wiki/AT_Attachment)
- **AHCI (Advanced Host Controller Interface)** - [Wikipedia](https://en.wikipedia.org/wiki/AHCI)

### Storage Controllers
- **NVMe Controllers** - PCIe-based SSD controllers
- **SATA Controllers** - Serial ATA host controllers
- **SAS HBA (Host Bus Adapter)** - [Wikipedia](https://en.wikipedia.org/wiki/Host_adapter)
- **Fibre Channel HBA** - [Wikipedia](https://en.wikipedia.org/wiki/Host_adapter#Fibre_Channel_host_adapter)
- **USB Storage Controllers** - USB-to-storage bridge chips
- **SD Card Controllers** - SD host controllers
- **eMMC Controllers** - Embedded MultiMediaCard controllers
- **Flash Controllers** - SSD/Flash memory controllers
- **Phison Controllers** - Third-party SSD controllers
- **Silicon Motion Controllers** - SSD controller manufacturer
- **Marvell Controllers** - Storage and network controllers
- **Realtek Controllers** - Consumer storage controllers

## Interconnect Technologies

### System Buses
- **PCIe 5.0** - [Wikipedia](https://en.wikipedia.org/wiki/PCI_Express#PCI_Express_5.0) | [PCIe 5.0 Spec](https://pcisig.com/specifications)
- **PCIe 4.0** - [Wikipedia](https://en.wikipedia.org/wiki/PCI_Express#PCI_Express_4.0)
- **PCIe 3.0** - [Wikipedia](https://en.wikipedia.org/wiki/PCI_Express#PCI_Express_3.0)
- **PCIe 6.0** - [Wikipedia](https://en.wikipedia.org/wiki/PCI_Express#PCI_Express_6.0) | [PCIe 6.0 Spec](https://pcisig.com/specifications)
- **CXL (Compute Express Link)** - [Wikipedia](https://en.wikipedia.org/wiki/Compute_Express_Link) | [CXL Spec](https://www.computeexpresslink.org/specifications)
- **UPI (Ultra Path Interconnect)** - [Wikipedia](https://en.wikipedia.org/wiki/Intel_Ultra_Path_Interconnect)
- **Infinity Fabric** - [Wikipedia](https://en.wikipedia.org/wiki/AMD_Infinity_Fabric)
- **DMI (Direct Media Interface)** - [Wikipedia](https://en.wikipedia.org/wiki/Direct_Media_Interface)

### High-Speed Interconnects
- **InfiniBand** - [Wikipedia](https://en.wikipedia.org/wiki/InfiniBand) | [InfiniBand Specs](https://www.infinibandta.org/ibta-specification/)
- **Ethernet** - [Wikipedia](https://en.wikipedia.org/wiki/Ethernet) | [IEEE 802.3 Specs](https://standards.ieee.org/standard/802_3-2018.html)
- **RoCE (RDMA over Converged Ethernet)** - [Wikipedia](https://en.wikipedia.org/wiki/RDMA_over_Converged_Ethernet)
- **iWARP** - [Wikipedia](https://en.wikipedia.org/wiki/IWARP)
- **Omni-Path** - [Wikipedia](https://en.wikipedia.org/wiki/Omni-Path)
- **NVLink** - [Wikipedia](https://en.wikipedia.org/wiki/NVLink) | [NVIDIA NVLink](https://www.nvidia.com/en-us/data-center/nvlink/)

### CPU Interconnects
- **NUMA (Non-Uniform Memory Access)** - [Wikipedia](https://en.wikipedia.org/wiki/Non-uniform_memory_access)
- **ccNUMA** - [Wikipedia](https://en.wikipedia.org/wiki/Cache-coherent_NUMA)
- **HyperTransport** - [Wikipedia](https://en.wikipedia.org/wiki/HyperTransport)
- **QPI (QuickPath Interconnect)** - [Wikipedia](https://en.wikipedia.org/wiki/Intel_QuickPath_Interconnect)

### Memory Interfaces
- **DDR Interface** - Memory controller to DRAM interface
- **LPDDR Interface** - Low-power DDR interface
- **GDDR Interface** - Graphics DDR interface
- **HBM Interface** - High Bandwidth Memory interface with silicon interposer
- **Memory PHY** - [Wikipedia](https://en.wikipedia.org/wiki/Physical_layer) - Physical layer for memory

### On-Chip Interconnects
- **AXI (Advanced eXtensible Interface)** - [Wikipedia](https://en.wikipedia.org/wiki/Advanced_eXtensible_Interface) - ARM on-chip bus
- **AHB (Advanced High-performance Bus)** - [Wikipedia](https://en.wikipedia.org/wiki/Advanced_Microcontroller_Bus_Architecture#AHB)
- **APB (Advanced Peripheral Bus)** - [Wikipedia](https://en.wikipedia.org/wiki/Advanced_Microcontroller_Bus_Architecture#APB)
- **Wishbone Bus** - [Wikipedia](https://en.wikipedia.org/wiki/Wishbone_(computer_bus)) - Open-source SoC bus
- **TileLink** - RISC-V on-chip interconnect
- **CoreConnect** - IBM on-chip bus
- **Avalon Interface** - Intel FPGA on-chip bus
- **NoC (Network-on-Chip)** - [Wikipedia](https://en.wikipedia.org/wiki/Network_on_a_chip)
- **Ring Bus** - Circular on-chip interconnect (Intel processors)
- **Mesh/2D Mesh** - 2D grid interconnect topology
- **Crossbar** - Full crossbar interconnect switch

## Graphics Technologies

### GPU Architectures
- **NVIDIA Ada Lovelace** - [Wikipedia](https://en.wikipedia.org/wiki/Ada_Lovelace_(microarchitecture)) | [NVIDIA Specs](https://www.nvidia.com/en-us/geforce/graphics-cards/40-series/)
- **NVIDIA Ampere** - [Wikipedia](https://en.wikipedia.org/wiki/Ampere_(microarchitecture))
- **NVIDIA Turing** - [Wikipedia](https://en.wikipedia.org/wiki/Turing_(microarchitecture))
- **AMD RDNA 3** - [Wikipedia](https://en.wikipedia.org/wiki/RDNA_3) | [AMD RDNA3 Specs](https://www.amd.com/en/products/graphics)
- **AMD RDNA 2** - [Wikipedia](https://en.wikipedia.org/wiki/RDNA_2)
- **Intel Xe** - [Wikipedia](https://en.wikipedia.org/wiki/Intel_Xe) | [Intel Arc Specs](https://www.intel.com/content/www/us/en/products/docs/discrete-gpus/arc/a-series/overview.html)

### Graphics APIs
- **DirectX 12** - [Wikipedia](https://en.wikipedia.org/wiki/DirectX#DirectX_12) | [Microsoft DirectX](https://docs.microsoft.com/en-us/windows/win32/direct3d12/)
- **Vulkan** - [Wikipedia](https://en.wikipedia.org/wiki/Vulkan_(API)) | [Vulkan Spec](https://www.khronos.org/vulkan/)
- **OpenGL** - [Wikipedia](https://en.wikipedia.org/wiki/OpenGL) | [OpenGL Spec](https://www.opengl.org/registry/)
- **Metal** - [Wikipedia](https://en.wikipedia.org/wiki/Metal_(API)) | [Apple Metal](https://developer.apple.com/metal/)
- **OpenCL** - [Wikipedia](https://en.wikipedia.org/wiki/OpenCL) | [OpenCL Spec](https://www.khronos.org/opencl/)
- **CUDA** - [Wikipedia](https://en.wikipedia.org/wiki/CUDA) | [NVIDIA CUDA](https://developer.nvidia.com/cuda-zone)
- **ROCm** - [Wikipedia](https://en.wikipedia.org/wiki/GPUOpen#ROCm) | [AMD ROCm](https://rocmdocs.amd.com/)

### Ray Tracing Technologies
- **RT Cores (NVIDIA)** - [Wikipedia](https://en.wikipedia.org/wiki/Turing_(microarchitecture)#RT_cores)
- **Ray Accelerators (AMD)** - [Wikipedia](https://en.wikipedia.org/wiki/RDNA_2#Ray_tracing)
- **DirectX Raytracing (DXR)** - [Wikipedia](https://en.wikipedia.org/wiki/DirectX_Raytracing)
- **Vulkan Ray Tracing** - [Wikipedia](https://en.wikipedia.org/wiki/Vulkan_(API)#Ray_tracing)

### Video Encode/Decode
- **NVENC (NVIDIA Encoder)** - [Wikipedia](https://en.wikipedia.org/wiki/Nvidia_NVENC) - Hardware H.264/H.265/AV1 encoding
- **NVDEC (NVIDIA Decoder)** - Hardware video decoding
- **VCE (AMD Video Coding Engine)** - [Wikipedia](https://en.wikipedia.org/wiki/Video_Core_Next#Video_Coding_Engine) - AMD hardware encoder
- **VCN (AMD Video Core Next)** - [Wikipedia](https://en.wikipedia.org/wiki/Video_Core_Next) - Unified encode/decode
- **Intel Quick Sync** - [Wikipedia](https://en.wikipedia.org/wiki/Intel_Quick_Sync_Video) - Intel integrated video encode/decode
- **Apple ProRes Accelerators** - Hardware ProRes encode/decode in Apple Silicon
- **AV1 Hardware Decoders** - [Wikipedia](https://en.wikipedia.org/wiki/AV1#Hardware_support)
- **HEVC/H.265 Hardware** - [Wikipedia](https://en.wikipedia.org/wiki/High_Efficiency_Video_Coding)
- **VP9 Hardware Decoders** - [Wikipedia](https://en.wikipedia.org/wiki/VP9)
- **Tensor Cores** - [Wikipedia](https://en.wikipedia.org/wiki/Tensor_Processing_Unit#Tensor_Cores) - AI/DLSS acceleration
- **Matrix Cores (AMD)** - AI acceleration in RDNA 3
- **XeSS Accelerators (Intel)** - AI super sampling hardware

### Display Technologies
- **OLED (Organic LED)** - [Wikipedia](https://en.wikipedia.org/wiki/OLED)
- **AMOLED (Active Matrix OLED)** - [Wikipedia](https://en.wikipedia.org/wiki/AMOLED)
- **Mini-LED** - [Wikipedia](https://en.wikipedia.org/wiki/Mini-LED)
- **Micro-LED** - [Wikipedia](https://en.wikipedia.org/wiki/MicroLED)
- **Quantum Dot (QLED)** - [Wikipedia](https://en.wikipedia.org/wiki/Quantum_dot_display)
- **IPS (In-Plane Switching)** - [Wikipedia](https://en.wikipedia.org/wiki/IPS_panel)
- **VA (Vertical Alignment)** - [Wikipedia](https://en.wikipedia.org/wiki/Liquid-crystal_display#Vertical_alignment_(VA))
- **TN (Twisted Nematic)** - [Wikipedia](https://en.wikipedia.org/wiki/Thin-film-transistor_liquid-crystal_display#Twisted_nematic_(TN))
- **E Ink / E-Paper** - [Wikipedia](https://en.wikipedia.org/wiki/Electronic_paper)
- **HDR (High Dynamic Range)** - [Wikipedia](https://en.wikipedia.org/wiki/High-dynamic-range_video)
  - **HDR10** - [Wikipedia](https://en.wikipedia.org/wiki/HDR10)
  - **HDR10+** - [Wikipedia](https://en.wikipedia.org/wiki/HDR10%2B)
  - **Dolby Vision** - [Wikipedia](https://en.wikipedia.org/wiki/Dolby_Vision)
- **VRR (Variable Refresh Rate)** - [Wikipedia](https://en.wikipedia.org/wiki/Variable_refresh_rate)
  - **G-Sync** - [Wikipedia](https://en.wikipedia.org/wiki/Nvidia_G-Sync)
  - **FreeSync** - [Wikipedia](https://en.wikipedia.org/wiki/FreeSync)
  - **Adaptive-Sync** - [Wikipedia](https://en.wikipedia.org/wiki/FreeSync#Adaptive-Sync)

## Network Technologies

### Ethernet Standards
- **400 Gigabit Ethernet** - [Wikipedia](https://en.wikipedia.org/wiki/400_Gigabit_Ethernet) | [IEEE 802.3bs](https://standards.ieee.org/standard/802_3bs-2017.html)
- **200 Gigabit Ethernet** - [Wikipedia](https://en.wikipedia.org/wiki/200_Gigabit_Ethernet)
- **100 Gigabit Ethernet** - [Wikipedia](https://en.wikipedia.org/wiki/100_Gigabit_Ethernet) | [IEEE 802.3ba](https://standards.ieee.org/standard/802_3ba-2010.html)
- **40 Gigabit Ethernet** - [Wikipedia](https://en.wikipedia.org/wiki/40_Gigabit_Ethernet)
- **25 Gigabit Ethernet** - [Wikipedia](https://en.wikipedia.org/wiki/25_Gigabit_Ethernet)
- **10 Gigabit Ethernet** - [Wikipedia](https://en.wikipedia.org/wiki/10_Gigabit_Ethernet)
- **2.5GBASE-T** - [Wikipedia](https://en.wikipedia.org/wiki/2.5GBASE-T_and_5GBASE-T)
- **5GBASE-T** - [Wikipedia](https://en.wikipedia.org/wiki/2.5GBASE-T_and_5GBASE-T)

### Wireless Technologies
- **Wi-Fi 7 (802.11be)** - [Wikipedia](https://en.wikipedia.org/wiki/IEEE_802.11be) | [IEEE 802.11be](https://standards.ieee.org/project/802_11be.html)
- **Wi-Fi 6E (802.11ax)** - [Wikipedia](https://en.wikipedia.org/wiki/Wi-Fi_6#Wi-Fi_6E)
- **Wi-Fi 6 (802.11ax)** - [Wikipedia](https://en.wikipedia.org/wiki/Wi-Fi_6) | [IEEE 802.11ax](https://standards.ieee.org/standard/802_11ax-2021.html)
- **5G NR** - [Wikipedia](https://en.wikipedia.org/wiki/5G_NR) | [3GPP 5G Specs](https://www.3gpp.org/release-18)
- **Bluetooth 5.4** - [Wikipedia](https://en.wikipedia.org/wiki/Bluetooth#Bluetooth_5.0) | [Bluetooth SIG](https://www.bluetooth.com/specifications/)
- **Thread** - [Wikipedia](https://en.wikipedia.org/wiki/Thread_(network_protocol)) | [Thread Group](https://www.threadgroup.org/support#specifications)
- **Zigbee** - [Wikipedia](https://en.wikipedia.org/wiki/Zigbee) | [Zigbee Alliance](https://zigbeealliance.org/zigbee_for_developers/zigbee-3-0/)

### Network Protocols
- **TCP/IP** - [Wikipedia](https://en.wikipedia.org/wiki/Internet_protocol_suite) | [RFC 793](https://tools.ietf.org/html/rfc793)
- **QUIC** - [Wikipedia](https://en.wikipedia.org/wiki/QUIC) | [RFC 9000](https://tools.ietf.org/html/rfc9000)
- **HTTP/3** - [Wikipedia](https://en.wikipedia.org/wiki/HTTP/3) | [RFC 9114](https://tools.ietf.org/html/rfc9114)
- **RDMA** - [Wikipedia](https://en.wikipedia.org/wiki/Remote_direct_memory_access)
- **SR-IOV** - [Wikipedia](https://en.wikipedia.org/wiki/Single-root_input/output_virtualization)

### Network Processors
- **Broadcom StrataXGS** - Ethernet switch chips | [Broadcom](https://www.broadcom.com/products/ethernet-connectivity/switching)
- **Marvell Prestera** - Network switch processors | [Marvell](https://www.marvell.com/products/switching.html)
- **Intel Tofino** - Programmable network switches | [Intel Tofino](https://www.intel.com/content/www/us/en/products/network-io/programmable-ethernet-switch.html)
- **NVIDIA BlueField DPU** - Data Processing Units | [NVIDIA BlueField](https://www.nvidia.com/en-us/networking/products/data-processing-unit/)
- **Pensando DSC** - Distributed Services Cards (now AMD) | [AMD Pensando](https://www.amd.com/en/products/adaptive-socs-and-fpgas/pensando.html)
- **Fungible DPU** - Data Processing Units (now Microsoft)
- **Mellanox ConnectX** - SmartNICs and network adapters | [NVIDIA Mellanox](https://www.nvidia.com/en-us/networking/ethernet-adapters/)
- **Chelsio T6** - Network adapters and accelerators | [Chelsio](https://www.chelsio.com/)
- **Netronome Agilio** - SmartNIC processors (now AMD Xilinx)
- **Cavium OCTEON** - Multi-core MIPS network processors (now Marvell)

## I/O Technologies

### Universal Serial Bus (USB)
- **USB4** - [Wikipedia](https://en.wikipedia.org/wiki/USB4) | [USB-IF USB4 Spec](https://www.usb.org/documents)
- **USB 3.2** - [Wikipedia](https://en.wikipedia.org/wiki/USB_3.0#USB_3.2)
- **USB-C** - [Wikipedia](https://en.wikipedia.org/wiki/USB-C) | [USB-IF USB-C Spec](https://www.usb.org/documents)
- **Thunderbolt 4** - [Wikipedia](https://en.wikipedia.org/wiki/Thunderbolt_(interface)#Thunderbolt_4) | [Intel Thunderbolt](https://www.intel.com/content/www/us/en/architecture-and-technology/thunderbolt/thunderbolt-technology-general.html)
- **Thunderbolt 3** - [Wikipedia](https://en.wikipedia.org/wiki/Thunderbolt_(interface)#Thunderbolt_3)

### Display Interfaces
- **DisplayPort 2.1** - [Wikipedia](https://en.wikipedia.org/wiki/DisplayPort#2.1) | [VESA DisplayPort](https://www.displayport.org/displayport-introduction/)
- **HDMI 2.1** - [Wikipedia](https://en.wikipedia.org/wiki/HDMI#Version_2.1) | [HDMI Spec](https://www.hdmi.org/spec/hdmi2_1)
- **DSC (Display Stream Compression)** - [Wikipedia](https://en.wikipedia.org/wiki/Display_Stream_Compression) | [VESA DSC](https://www.vesa.org/featured-articles/vesa-display-stream-compression-dsc-reduces-display-link-bandwidth-requirements-while-supporting-visually-lossless-image-quality/)

### Audio Interfaces
- **HD Audio** - [Wikipedia](https://en.wikipedia.org/wiki/Intel_High_Definition_Audio) | [Intel HD Audio Spec](https://www.intel.com/content/www/us/en/standards/high-definition-audio-specification.html)
- **AC'97** - [Wikipedia](https://en.wikipedia.org/wiki/AC%2797)
- **I2S (Inter-IC Sound)** - [Wikipedia](https://en.wikipedia.org/wiki/I%C2%B2S) - Digital audio interface
- **S/PDIF** - [Wikipedia](https://en.wikipedia.org/wiki/S/PDIF) - Sony/Philips Digital Interface
- **TOSLINK** - [Wikipedia](https://en.wikipedia.org/wiki/TOSLINK) - Optical audio
- **ADAT** - [Wikipedia](https://en.wikipedia.org/wiki/ADAT) - Alesis Digital Audio Tape interface

### Legacy I/O
- **PS/2** - [Wikipedia](https://en.wikipedia.org/wiki/PS/2_port) - Keyboard and mouse interface
- **Serial Port (RS-232)** - [Wikipedia](https://en.wikipedia.org/wiki/RS-232)
- **Parallel Port (LPT)** - [Wikipedia](https://en.wikipedia.org/wiki/Parallel_port)
- **VGA** - [Wikipedia](https://en.wikipedia.org/wiki/Video_Graphics_Array)
- **DVI** - [Wikipedia](https://en.wikipedia.org/wiki/Digital_Visual_Interface)
- **FireWire (IEEE 1394)** - [Wikipedia](https://en.wikipedia.org/wiki/IEEE_1394)
- **eSATA** - [Wikipedia](https://en.wikipedia.org/wiki/Serial_ATA#eSATA)
- **IDE/PATA** - [Wikipedia](https://en.wikipedia.org/wiki/Parallel_ATA)
- **Floppy Drive Interface** - [Wikipedia](https://en.wikipedia.org/wiki/Floppy_disk)
- **Game Port** - [Wikipedia](https://en.wikipedia.org/wiki/Game_port)
- **SCSI Parallel** - [Wikipedia](https://en.wikipedia.org/wiki/SCSI)
- **Centronics** - [Wikipedia](https://en.wikipedia.org/wiki/Centronics)

## Input Devices

### Keyboards
- **Mechanical Keyboards** - [Wikipedia](https://en.wikipedia.org/wiki/Keyboard_technology#Mechanical-switch_keyboard)
  - **Cherry MX Switches** - [Wikipedia](https://en.wikipedia.org/wiki/Cherry_(keyboards)#MX_switches)
  - **Alps Switches** - [Wikipedia](https://en.wikipedia.org/wiki/Alps_Electric#ALPS_SKCM/SKCL_series)
  - **Topre Switches** - Electrostatic capacitive switches
  - **Kailh Switches** - Cherry MX-compatible switches
- **Membrane Keyboards** - [Wikipedia](https://en.wikipedia.org/wiki/Membrane_keyboard)
- **Scissor-Switch Keyboards** - [Wikipedia](https://en.wikipedia.org/wiki/Keyboard_technology#Scissor-switch_keyboard)
- **Capacitive Keyboards** - [Wikipedia](https://en.wikipedia.org/wiki/Keyboard_technology#Capacitive_keyboard)
- **Hall Effect Keyboards** - Magnetic sensing keyboards
- **Keyboard Controllers** - USB/PS2 keyboard controller chips
- **Key Matrix Scanning** - [Wikipedia](https://en.wikipedia.org/wiki/Keyboard_matrix_circuit)

### Pointing Devices
- **Optical Mouse** - [Wikipedia](https://en.wikipedia.org/wiki/Optical_mouse)
- **Laser Mouse** - [Wikipedia](https://en.wikipedia.org/wiki/Optical_mouse#Laser_mice)
- **Trackball** - [Wikipedia](https://en.wikipedia.org/wiki/Trackball)
- **Touchpad** - [Wikipedia](https://en.wikipedia.org/wiki/Touchpad)
  - **Synaptics Touchpads** - [Synaptics](https://www.synaptics.com/)
  - **ELAN Touchpads** - Touchpad controllers
  - **Precision Touchpad** - [Microsoft Precision Touchpad](https://docs.microsoft.com/en-us/windows-hardware/design/component-guidelines/touchpad)
- **TrackPoint** - [Wikipedia](https://en.wikipedia.org/wiki/Pointing_stick)
- **Graphics Tablet** - [Wikipedia](https://en.wikipedia.org/wiki/Graphics_tablet)
  - **Wacom Technology** - [Wikipedia](https://en.wikipedia.org/wiki/Wacom) | [Wacom](https://www.wacom.com/)
  - **EMR (Electromagnetic Resonance)** - Wacom's pen technology
- **Stylus/Pen Input** - [Wikipedia](https://en.wikipedia.org/wiki/Stylus_(computing))
  - **Apple Pencil** - [Wikipedia](https://en.wikipedia.org/wiki/Apple_Pencil)
  - **Surface Pen** - [Microsoft Surface Pen](https://www.microsoft.com/en-us/d/surface-pen/8zl5c82qmg6b)
  - **Wacom Pen** - Active stylus technology

### Touchscreens
- **Capacitive Touchscreens** - [Wikipedia](https://en.wikipedia.org/wiki/Touchscreen#Capacitive)
  - **Projected Capacitive (P-Cap)** - [Wikipedia](https://en.wikipedia.org/wiki/Touchscreen#Projected_capacitive)
  - **Surface Capacitive** - [Wikipedia](https://en.wikipedia.org/wiki/Touchscreen#Surface_capacitance)
- **Resistive Touchscreens** - [Wikipedia](https://en.wikipedia.org/wiki/Touchscreen#Resistive)
- **Infrared Touchscreens** - [Wikipedia](https://en.wikipedia.org/wiki/Touchscreen#Infrared)
- **Surface Acoustic Wave** - [Wikipedia](https://en.wikipedia.org/wiki/Touchscreen#Surface_acoustic_wave)
- **Optical Touchscreens** - [Wikipedia](https://en.wikipedia.org/wiki/Touchscreen#Optical)
- **Touch Controllers** - Touchscreen controller ICs
  - **Goodix Touch Controllers** - Mobile touch controllers
  - **Focaltech Controllers** - Touchscreen ICs
  - **Ilitek Controllers** - Touch panel controllers

### Game Controllers
- **Gamepad Controllers** - [Wikipedia](https://en.wikipedia.org/wiki/Gamepad)
- **Joystick** - [Wikipedia](https://en.wikipedia.org/wiki/Joystick)
- **Racing Wheel** - [Wikipedia](https://en.wikipedia.org/wiki/Racing_wheel)
- **Flight Stick** - [Wikipedia](https://en.wikipedia.org/wiki/Joystick#Flight_simulation)
- **Motion Controllers** - [Wikipedia](https://en.wikipedia.org/wiki/Motion_controller)
- **VR Controllers** - Virtual reality input devices
- **Haptic Feedback** - [Wikipedia](https://en.wikipedia.org/wiki/Haptic_technology)
  - **Rumble Motors** - Vibration feedback
  - **HD Rumble** - Advanced haptic feedback (Nintendo Switch)
  - **DualSense Haptics** - PlayStation 5 haptic feedback

### Biometric Readers
- **Fingerprint Sensors** - [Wikipedia](https://en.wikipedia.org/wiki/Fingerprint_recognition)
  - **Capacitive Fingerprint** - Capacitive sensing
  - **Optical Fingerprint** - Optical imaging
  - **Ultrasonic Fingerprint** - Ultrasonic sensors (Qualcomm)
  - **Synaptics Fingerprint** - [Synaptics](https://www.synaptics.com/products/biometrics/fingerprint)
  - **Goodix Fingerprint** - In-display fingerprint sensors
- **Facial Recognition** - [Wikipedia](https://en.wikipedia.org/wiki/Facial_recognition_system)
  - **Face ID (Apple)** - [Wikipedia](https://en.wikipedia.org/wiki/Face_ID) - TrueDepth camera
  - **Windows Hello** - [Wikipedia](https://en.wikipedia.org/wiki/Windows_10#System_security) - IR cameras
  - **3D Structured Light** - Depth sensing for face recognition
- **Iris Scanners** - [Wikipedia](https://en.wikipedia.org/wiki/Iris_recognition)
- **Voice Recognition** - [Wikipedia](https://en.wikipedia.org/wiki/Speaker_recognition)
- **Retina Scanners** - [Wikipedia](https://en.wikipedia.org/wiki/Retinal_scan)

## Output Devices

### Displays

#### Display Panel Types
- **LCD (Liquid Crystal Display)** - [Wikipedia](https://en.wikipedia.org/wiki/Liquid-crystal_display)
- **LED-Backlit LCD** - [Wikipedia](https://en.wikipedia.org/wiki/LED-backlit_LCD)
- **OLED** - [Wikipedia](https://en.wikipedia.org/wiki/OLED)
- **AMOLED** - [Wikipedia](https://en.wikipedia.org/wiki/AMOLED)
- **Mini-LED** - [Wikipedia](https://en.wikipedia.org/wiki/Mini-LED)
- **Micro-LED** - [Wikipedia](https://en.wikipedia.org/wiki/MicroLED)
- **Quantum Dot** - [Wikipedia](https://en.wikipedia.org/wiki/Quantum_dot_display)
- **E Ink** - [Wikipedia](https://en.wikipedia.org/wiki/E_Ink)
- **CRT** - [Wikipedia](https://en.wikipedia.org/wiki/Cathode-ray_tube)
- **Plasma Display** - [Wikipedia](https://en.wikipedia.org/wiki/Plasma_display)

#### Display Resolutions
- **8K (7680×4320)** - [Wikipedia](https://en.wikipedia.org/wiki/8K_resolution)
- **5K (5120×2880)** - [Wikipedia](https://en.wikipedia.org/wiki/5K_resolution)
- **4K/UHD (3840×2160)** - [Wikipedia](https://en.wikipedia.org/wiki/4K_resolution)
- **QHD/1440p (2560×1440)** - [Wikipedia](https://en.wikipedia.org/wiki/Graphics_display_resolution#QHD_(Quad_HD))
- **Full HD/1080p (1920×1080)** - [Wikipedia](https://en.wikipedia.org/wiki/1080p)
- **HD/720p (1280×720)** - [Wikipedia](https://en.wikipedia.org/wiki/720p)
- **WQHD (2560×1440)** - Wide Quad HD
- **UWQHD (3440×1440)** - Ultra-wide QHD
- **Retina Display** - [Wikipedia](https://en.wikipedia.org/wiki/Retina_display) - High-DPI displays

### Printers
- **Inkjet Printers** - [Wikipedia](https://en.wikipedia.org/wiki/Inkjet_printing)
  - **Piezoelectric Inkjet** - [Wikipedia](https://en.wikipedia.org/wiki/Inkjet_printing#Piezoelectric)
  - **Thermal Inkjet** - [Wikipedia](https://en.wikipedia.org/wiki/Inkjet_printing#Thermal)
- **Laser Printers** - [Wikipedia](https://en.wikipedia.org/wiki/Laser_printing)
- **Thermal Printers** - [Wikipedia](https://en.wikipedia.org/wiki/Thermal_printing)
  - **Direct Thermal** - [Wikipedia](https://en.wikipedia.org/wiki/Thermal_printing#Direct_thermal)
  - **Thermal Transfer** - [Wikipedia](https://en.wikipedia.org/wiki/Thermal-transfer_printing)
- **3D Printers** - [Wikipedia](https://en.wikipedia.org/wiki/3D_printing)
  - **FDM (Fused Deposition Modeling)** - [Wikipedia](https://en.wikipedia.org/wiki/Fused_filament_fabrication)
  - **SLA (Stereolithography)** - [Wikipedia](https://en.wikipedia.org/wiki/Stereolithography)
  - **SLS (Selective Laser Sintering)** - [Wikipedia](https://en.wikipedia.org/wiki/Selective_laser_sintering)
  - **DLP (Digital Light Processing)** - [Wikipedia](https://en.wikipedia.org/wiki/Digital_light_processing)
- **Dot Matrix Printers** - [Wikipedia](https://en.wikipedia.org/wiki/Dot_matrix_printing)
- **Dye-Sublimation Printers** - [Wikipedia](https://en.wikipedia.org/wiki/Dye-sublimation_printer)

### Projectors
- **DLP (Digital Light Processing)** - [Wikipedia](https://en.wikipedia.org/wiki/Digital_Light_Processing)
- **LCD Projectors** - [Wikipedia](https://en.wikipedia.org/wiki/LCD_projector)
- **LCoS (Liquid Crystal on Silicon)** - [Wikipedia](https://en.wikipedia.org/wiki/Liquid_crystal_on_silicon)
- **Laser Projectors** - [Wikipedia](https://en.wikipedia.org/wiki/Laser_projector)
- **LED Projectors** - LED-based projection
- **Pico Projectors** - [Wikipedia](https://en.wikipedia.org/wiki/Handheld_projector)

### Audio Output
- **DAC (Digital-to-Analog Converter)** - [Wikipedia](https://en.wikipedia.org/wiki/Digital-to-analog_converter)
- **Audio Amplifiers** - [Wikipedia](https://en.wikipedia.org/wiki/Audio_amplifier)
  - **Class A/B Amplifiers** - [Wikipedia](https://en.wikipedia.org/wiki/Amplifier#Class_A)
  - **Class D Amplifiers** - [Wikipedia](https://en.wikipedia.org/wiki/Class-D_amplifier)
- **Audio Codecs** - [Wikipedia](https://en.wikipedia.org/wiki/Audio_codec)
  - **Realtek ALC Series** - PC audio codecs
  - **Cirrus Logic** - Audio codec manufacturer
  - **ESS Sabre DACs** - High-end DAC chips
- **Speakers** - [Wikipedia](https://en.wikipedia.org/wiki/Loudspeaker)
- **Headphone Amplifiers** - [Wikipedia](https://en.wikipedia.org/wiki/Headphone_amplifier)

## Sensors & Data Acquisition

### Environmental Sensors
- **Temperature Sensors** - [Wikipedia](https://en.wikipedia.org/wiki/Temperature_measurement)
  - **Thermocouples** - [Wikipedia](https://en.wikipedia.org/wiki/Thermocouple)
  - **RTD (Resistance Temperature Detector)** - [Wikipedia](https://en.wikipedia.org/wiki/Resistance_thermometer)
  - **Thermistors** - [Wikipedia](https://en.wikipedia.org/wiki/Thermistor)
  - **Infrared Temperature Sensors** - Non-contact temperature
- **Humidity Sensors** - [Wikipedia](https://en.wikipedia.org/wiki/Humidity_sensor)
- **Pressure Sensors** - [Wikipedia](https://en.wikipedia.org/wiki/Pressure_sensor)
  - **Barometric Pressure** - Atmospheric pressure sensors
  - **MEMS Pressure Sensors** - [Wikipedia](https://en.wikipedia.org/wiki/Pressure_sensor#MEMS)
- **Gas Sensors** - [Wikipedia](https://en.wikipedia.org/wiki/Gas_detector)
  - **CO2 Sensors** - Carbon dioxide detection
  - **VOC Sensors** - Volatile organic compounds
  - **Smoke Detectors** - [Wikipedia](https://en.wikipedia.org/wiki/Smoke_detector)
- **Ambient Light Sensors** - [Wikipedia](https://en.wikipedia.org/wiki/Ambient_light_sensor)
- **UV Sensors** - Ultraviolet light detection

### Motion Sensors
- **Accelerometers** - [Wikipedia](https://en.wikipedia.org/wiki/Accelerometer)
  - **MEMS Accelerometers** - [Wikipedia](https://en.wikipedia.org/wiki/Accelerometer#MEMS_accelerometers)
  - **Piezoelectric Accelerometers** - [Wikipedia](https://en.wikipedia.org/wiki/Accelerometer#Piezoelectric_accelerometers)
- **Gyroscopes** - [Wikipedia](https://en.wikipedia.org/wiki/Gyroscope)
  - **MEMS Gyroscopes** - [Wikipedia](https://en.wikipedia.org/wiki/Vibrating_structure_gyroscope)
  - **Ring Laser Gyroscopes** - [Wikipedia](https://en.wikipedia.org/wiki/Ring_laser_gyroscope)
  - **Fiber Optic Gyroscopes** - [Wikipedia](https://en.wikipedia.org/wiki/Fibre-optic_gyroscope)
- **Magnetometers** - [Wikipedia](https://en.wikipedia.org/wiki/Magnetometer)
- **IMU (Inertial Measurement Unit)** - [Wikipedia](https://en.wikipedia.org/wiki/Inertial_measurement_unit)
- **PIR (Passive Infrared)** - [Wikipedia](https://en.wikipedia.org/wiki/Passive_infrared_sensor) - Motion detection
- **Ultrasonic Motion Sensors** - [Wikipedia](https://en.wikipedia.org/wiki/Motion_detector#Ultrasonic)
- **Microwave Motion Sensors** - [Wikipedia](https://en.wikipedia.org/wiki/Motion_detector#Microwave)

### Optical Sensors
- **Image Sensors** - [Wikipedia](https://en.wikipedia.org/wiki/Image_sensor)
  - **CCD (Charge-Coupled Device)** - [Wikipedia](https://en.wikipedia.org/wiki/Charge-coupled_device)
  - **CMOS Image Sensors** - [Wikipedia](https://en.wikipedia.org/wiki/Active-pixel_sensor)
  - **BSI (Back-Side Illuminated)** - [Wikipedia](https://en.wikipedia.org/wiki/Back-illuminated_sensor)
  - **Stacked CMOS** - [Wikipedia](https://en.wikipedia.org/wiki/Active-pixel_sensor#Stacked_sensors)
- **Camera Modules** - Integrated camera assemblies
  - **Sony IMX Sensors** - Popular CMOS image sensors
  - **Samsung ISOCELL** - Samsung image sensors
  - **OmniVision Sensors** - Image sensor manufacturer
- **Photodiodes** - [Wikipedia](https://en.wikipedia.org/wiki/Photodiode)
- **Phototransistors** - [Wikipedia](https://en.wikipedia.org/wiki/Phototransistor)
- **LiDAR Sensors** - [Wikipedia](https://en.wikipedia.org/wiki/Lidar)
- **Time-of-Flight (ToF) Sensors** - [Wikipedia](https://en.wikipedia.org/wiki/Time-of-flight_camera)
- **Proximity Sensors** - [Wikipedia](https://en.wikipedia.org/wiki/Proximity_sensor)
  - **Infrared Proximity** - IR-based proximity detection
  - **Capacitive Proximity** - Capacitive sensing
  - **Ultrasonic Proximity** - Ultrasonic distance measurement

### Position & Navigation
- **GPS Receivers** - [Wikipedia](https://en.wikipedia.org/wiki/GPS_navigation_device)
  - **GNSS (Global Navigation Satellite System)** - [Wikipedia](https://en.wikipedia.org/wiki/Satellite_navigation)
  - **GPS** - [Wikipedia](https://en.wikipedia.org/wiki/Global_Positioning_System) - US system
  - **GLONASS** - [Wikipedia](https://en.wikipedia.org/wiki/GLONASS) - Russian system
  - **Galileo** - [Wikipedia](https://en.wikipedia.org/wiki/Galileo_(satellite_navigation)) - European system
  - **BeiDou** - [Wikipedia](https://en.wikipedia.org/wiki/BeiDou) - Chinese system
- **Compass Sensors** - Digital compass/magnetometer
- **Encoders** - [Wikipedia](https://en.wikipedia.org/wiki/Rotary_encoder)
  - **Rotary Encoders** - Angular position
  - **Linear Encoders** - Linear position
- **Hall Effect Sensors** - [Wikipedia](https://en.wikipedia.org/wiki/Hall_effect_sensor)
- **Resolver** - [Wikipedia](https://en.wikipedia.org/wiki/Resolver_(electrical)) - Rotary position sensor

### ADC/DAC
- **ADC (Analog-to-Digital Converter)** - [Wikipedia](https://en.wikipedia.org/wiki/Analog-to-digital_converter)
  - **SAR ADC** - [Wikipedia](https://en.wikipedia.org/wiki/Successive-approximation_ADC) - Successive approximation
  - **Delta-Sigma ADC** - [Wikipedia](https://en.wikipedia.org/wiki/Delta-sigma_modulation)
  - **Pipeline ADC** - [Wikipedia](https://en.wikipedia.org/wiki/Analog-to-digital_converter#Pipeline_ADC)
  - **Flash ADC** - [Wikipedia](https://en.wikipedia.org/wiki/Flash_ADC)
- **DAC (Digital-to-Analog Converter)** - [Wikipedia](https://en.wikipedia.org/wiki/Digital-to-analog_converter)
  - **R-2R Ladder DAC** - [Wikipedia](https://en.wikipedia.org/wiki/Resistor_ladder#R%E2%80%932R_resistor_ladder_network_(digital_to_analog_conversion))
  - **PWM DAC** - [Wikipedia](https://en.wikipedia.org/wiki/Pulse-width_modulation#Digital-to-analog_conversion) - Pulse-width modulation
  - **Delta-Sigma DAC** - [Wikipedia](https://en.wikipedia.org/wiki/Delta-sigma_modulation)

## Chipset Technologies

### Intel Chipsets
- **Intel Z790** - [Wikipedia](https://en.wikipedia.org/wiki/List_of_Intel_chipsets#LGA_1700_chipsets) | [Intel Z790 Specs](https://ark.intel.com/content/www/us/en/ark/products/chipsets/series/230/intel-z790-chipset.html)
- **Intel B760** - [Wikipedia](https://en.wikipedia.org/wiki/List_of_Intel_chipsets#LGA_1700_chipsets)
- **Intel H770** - [Wikipedia](https://en.wikipedia.org/wiki/List_of_Intel_chipsets#LGA_1700_chipsets)
- **Intel X299** - [Wikipedia](https://en.wikipedia.org/wiki/List_of_Intel_chipsets#LGA_2066_chipsets)

### AMD Chipsets
- **AMD X670E** - [Wikipedia](https://en.wikipedia.org/wiki/List_of_AMD_chipsets#AM5_chipsets) | [AMD 600 Series Specs](https://www.amd.com/en/products/chipsets-am5)
- **AMD B650** - [Wikipedia](https://en.wikipedia.org/wiki/List_of_AMD_chipsets#AM5_chipsets)
- **AMD TRX50** - [Wikipedia](https://en.wikipedia.org/wiki/List_of_AMD_chipsets#sTRX4_chipsets)

### Mobile Chipsets
- **Qualcomm** - Mobile chipset manufacturer | [Qualcomm](https://www.qualcomm.com/)
  - **Power Management ICs (PMICs)** - Power management for mobile SoCs
  - **Audio Codecs** - Mobile audio chipsets
  - **RF Front-End Modules** - Cellular RF components
- **MediaTek** - Mobile chipset components | [MediaTek](https://www.mediatek.com/)
- **Samsung** - Exynos companion chipsets
- **Broadcom** - Wi-Fi/Bluetooth combo chips | [Broadcom](https://www.broadcom.com/)
- **Texas Instruments** - Power management and charging ICs
- **Dialog Semiconductor** - PMICs and power ICs (now Renesas)
- **Maxim Integrated** - Power management ICs (now Analog Devices)

## Power Management

### Power Standards
- **ACPI** - [Wikipedia](https://en.wikipedia.org/wiki/Advanced_Configuration_and_Power_Interface) | [ACPI Spec](https://uefi.org/specifications)
- **ATX Power Supply** - [Wikipedia](https://en.wikipedia.org/wiki/ATX#Power_supply) | [ATX Spec](https://www.intel.com/content/www/us/en/support/articles/000005796/boards-and-kits.html)
- **80 Plus** - [Wikipedia](https://en.wikipedia.org/wiki/80_Plus) | [80 Plus Certification](https://www.80plus.org/)
- **PCI Power Management** - [Wikipedia](https://en.wikipedia.org/wiki/PCI_Express#Power_management)

### CPU Power Technologies
- **Intel SpeedStep** - [Wikipedia](https://en.wikipedia.org/wiki/SpeedStep)
- **AMD Cool'n'Quiet** - [Wikipedia](https://en.wikipedia.org/wiki/Cool%27n%27Quiet)
- **Intel Turbo Boost** - [Wikipedia](https://en.wikipedia.org/wiki/Intel_Turbo_Boost) | [Intel Turbo Boost Spec](https://www.intel.com/content/www/us/en/architecture-and-technology/turbo-boost/turbo-boost-technology.html)
- **AMD Precision Boost** - [Wikipedia](https://en.wikipedia.org/wiki/List_of_AMD_CPU_microarchitectures#Zen)
- **DVFS (Dynamic Voltage and Frequency Scaling)** - [Wikipedia](https://en.wikipedia.org/wiki/Dynamic_frequency_scaling)
- **P-States** - [Wikipedia](https://en.wikipedia.org/wiki/P-state) - Performance states
- **C-States** - [Wikipedia](https://en.wikipedia.org/wiki/Advanced_Configuration_and_Power_Interface#Processor_states) - CPU sleep states
- **S-States** - [Wikipedia](https://en.wikipedia.org/wiki/Advanced_Configuration_and_Power_Interface#Global_states) - System sleep states

### Battery Technologies
- **Lithium-Ion (Li-ion)** - [Wikipedia](https://en.wikipedia.org/wiki/Lithium-ion_battery)
- **Lithium Polymer (LiPo)** - [Wikipedia](https://en.wikipedia.org/wiki/Lithium_polymer_battery)
- **Lithium Iron Phosphate (LiFePO4)** - [Wikipedia](https://en.wikipedia.org/wiki/Lithium_iron_phosphate_battery)
- **NiMH (Nickel-Metal Hydride)** - [Wikipedia](https://en.wikipedia.org/wiki/Nickel%E2%80%93metal_hydride_battery)
- **NiCd (Nickel-Cadmium)** - [Wikipedia](https://en.wikipedia.org/wiki/Nickel%E2%80%93cadmium_battery)
- **Solid-State Batteries** - [Wikipedia](https://en.wikipedia.org/wiki/Solid-state_battery)
- **Battery Management Systems (BMS)** - [Wikipedia](https://en.wikipedia.org/wiki/Battery_management_system)
- **Fuel Gauges** - Battery monitoring ICs
- **Charging ICs** - Battery charging controllers

### Power Delivery
- **USB Power Delivery (USB-PD)** - [Wikipedia](https://en.wikipedia.org/wiki/USB_Power_Delivery) | [USB-IF PD Spec](https://www.usb.org/usb-charger-pd)
- **Quick Charge (Qualcomm)** - [Wikipedia](https://en.wikipedia.org/wiki/Quick_Charge)
- **Power over Ethernet (PoE)** - [Wikipedia](https://en.wikipedia.org/wiki/Power_over_Ethernet)
  - **PoE (802.3af)** - [Wikipedia](https://en.wikipedia.org/wiki/Power_over_Ethernet#Standard_802.3af_(PoE)) - 15.4W
  - **PoE+ (802.3at)** - [Wikipedia](https://en.wikipedia.org/wiki/Power_over_Ethernet#Standard_802.3at_(PoE+)) - 30W
  - **PoE++ (802.3bt)** - [Wikipedia](https://en.wikipedia.org/wiki/Power_over_Ethernet#Standard_802.3bt_(PoE++)) - Up to 90W
- **Wireless Charging** - [Wikipedia](https://en.wikipedia.org/wiki/Inductive_charging)
  - **Qi Wireless Charging** - [Wikipedia](https://en.wikipedia.org/wiki/Qi_(standard))
  - **MagSafe** - [Apple MagSafe](https://www.apple.com/shop/accessories/all/magsafe)
  - **AirFuel** - [Wikipedia](https://en.wikipedia.org/wiki/AirFuel_Alliance)
- **ATX12V** - [Wikipedia](https://en.wikipedia.org/wiki/ATX#ATX12V) - 12V power standard
- **EPS12V** - [Wikipedia](https://en.wikipedia.org/wiki/EPS12V) - Server power standard
- **PCIe Power Connectors** - [Wikipedia](https://en.wikipedia.org/wiki/PCI_Express#Power)
  - **6-pin PCIe** - 75W
  - **8-pin PCIe** - 150W
  - **12VHPWR (PCIe 5.0)** - [Wikipedia](https://en.wikipedia.org/wiki/PCI_Express#Upcoming_revisions) - Up to 600W

### Voltage Regulators
- **VRM (Voltage Regulator Module)** - [Wikipedia](https://en.wikipedia.org/wiki/Voltage_regulator_module)
- **Buck Converter** - [Wikipedia](https://en.wikipedia.org/wiki/Buck_converter) - Step-down DC-DC
- **Boost Converter** - [Wikipedia](https://en.wikipedia.org/wiki/Boost_converter) - Step-up DC-DC
- **Buck-Boost Converter** - [Wikipedia](https://en.wikipedia.org/wiki/Buck%E2%80%93boost_converter)
- **LDO (Low-Dropout Regulator)** - [Wikipedia](https://en.wikipedia.org/wiki/Low-dropout_regulator)
- **Switching Regulators** - [Wikipedia](https://en.wikipedia.org/wiki/Switched-mode_power_supply)
- **Linear Regulators** - [Wikipedia](https://en.wikipedia.org/wiki/Linear_regulator)
- **PMIC (Power Management IC)** - Integrated power management
- **PWM Controllers** - [Wikipedia](https://en.wikipedia.org/wiki/Pulse-width_modulation) - Pulse-width modulation
- **DrMOS** - [Wikipedia](https://en.wikipedia.org/wiki/DrMOS) - Integrated power stage
- **Multi-Phase VRM** - Multiple phases for CPU power delivery

## Clock & Timing

### Crystal Oscillators
- **Quartz Crystal** - [Wikipedia](https://en.wikipedia.org/wiki/Crystal_oscillator)
- **TCXO (Temperature Compensated Crystal Oscillator)** - [Wikipedia](https://en.wikipedia.org/wiki/Crystal_oscillator#Temperature-compensated_crystal_oscillator)
- **OCXO (Oven-Controlled Crystal Oscillator)** - [Wikipedia](https://en.wikipedia.org/wiki/Crystal_oven)
- **VCXO (Voltage-Controlled Crystal Oscillator)** - [Wikipedia](https://en.wikipedia.org/wiki/Voltage-controlled_oscillator)
- **MEMS Oscillators** - [Wikipedia](https://en.wikipedia.org/wiki/MEMS_oscillator)
- **32.768 kHz Crystal** - Real-time clock crystal

### Clock Generators
- **PLL (Phase-Locked Loop)** - [Wikipedia](https://en.wikipedia.org/wiki/Phase-locked_loop)
- **Clock Multiplier** - [Wikipedia](https://en.wikipedia.org/wiki/Clock_multiplier)
- **Clock Divider** - [Wikipedia](https://en.wikipedia.org/wiki/Frequency_divider)
- **Spread Spectrum Clocking** - [Wikipedia](https://en.wikipedia.org/wiki/Spread-spectrum_clock_signal) - EMI reduction
- **Jitter Cleaner** - Low-jitter clock generation
- **Clock Distribution** - Clock fanout buffers
- **Clock Synthesizer** - Programmable clock generators

### Timing Standards
- **RTC (Real-Time Clock)** - [Wikipedia](https://en.wikipedia.org/wiki/Real-time_clock)
- **NTP (Network Time Protocol)** - [Wikipedia](https://en.wikipedia.org/wiki/Network_Time_Protocol) | [RFC 5905](https://tools.ietf.org/html/rfc5905)
- **PTP (Precision Time Protocol)** - [Wikipedia](https://en.wikipedia.org/wiki/Precision_Time_Protocol) - IEEE 1588
- **GPS Time** - [Wikipedia](https://en.wikipedia.org/wiki/Time_and_frequency_transfer#GPS_disciplined_oscillator)
- **Atomic Clocks** - [Wikipedia](https://en.wikipedia.org/wiki/Atomic_clock)
- **GNSS Time** - Satellite-based timing
- **White Rabbit** - [Wikipedia](https://en.wikipedia.org/wiki/White_Rabbit_Project) - Precision timing

## Firmware Technologies

### Boot Technologies
- **UEFI** - [Wikipedia](https://en.wikipedia.org/wiki/Unified_Extensible_Firmware_Interface) | [UEFI Spec](https://uefi.org/specifications)
- **BIOS** - [Wikipedia](https://en.wikipedia.org/wiki/BIOS)
- **Secure Boot** - [Wikipedia](https://en.wikipedia.org/wiki/Unified_Extensible_Firmware_Interface#Secure_Boot)
- **TPM 2.0** - [Wikipedia](https://en.wikipedia.org/wiki/Trusted_Platform_Module) | [TPM 2.0 Spec](https://trustedcomputinggroup.org/resource/tpm-library-specification/)

### Management Technologies
- **Intel AMT** - [Wikipedia](https://en.wikipedia.org/wiki/Intel_Active_Management_Technology) | [Intel AMT](https://www.intel.com/content/www/us/en/architecture-and-technology/intel-active-management-technology.html)
- **IPMI** - [Wikipedia](https://en.wikipedia.org/wiki/Intelligent_Platform_Management_Interface) | [IPMI Spec](https://www.intel.com/content/www/us/en/servers/ipmi/ipmi-specifications.html)
- **Redfish** - [Wikipedia](https://en.wikipedia.org/wiki/Redfish_(specification)) | [DMTF Redfish](https://www.dmtf.org/standards/redfish)

### Embedded Firmware
- **U-Boot** - [Wikipedia](https://en.wikipedia.org/wiki/Das_U-Boot) | [U-Boot](https://www.denx.de/wiki/U-Boot) - Universal bootloader
- **Barebox** - [Wikipedia](https://en.wikipedia.org/wiki/Barebox) - Bootloader
- **Coreboot** - [Wikipedia](https://en.wikipedia.org/wiki/Coreboot) | [Coreboot](https://www.coreboot.org/) - Open-source firmware
- **Libreboot** - [Wikipedia](https://en.wikipedia.org/wiki/Libreboot) | [Libreboot](https://libreboot.org/) - Free software BIOS
- **OpenFirmware (Open Firmware)** - [Wikipedia](https://en.wikipedia.org/wiki/Open_Firmware)
- **Device Tree** - [Wikipedia](https://en.wikipedia.org/wiki/Device_tree) - Hardware description
- **ATF (ARM Trusted Firmware)** - [ARM Trusted Firmware](https://www.trustedfirmware.org/)
- **SCP (System Control Processor)** - ARM system control firmware
- **EC (Embedded Controller)** - [Wikipedia](https://en.wikipedia.org/wiki/Embedded_controller) - Keyboard, battery management
- **SMBUS/I2C Firmware** - Sensor and power management firmware

## Cooling Technologies

### Thermal Management
- **Heat Pipes** - [Wikipedia](https://en.wikipedia.org/wiki/Heat_pipe)
- **Vapor Chambers** - [Wikipedia](https://en.wikipedia.org/wiki/Vapor_chamber)
- **Thermal Interface Materials (TIM)** - [Wikipedia](https://en.wikipedia.org/wiki/Thermal_interface_material)
  - **Thermal Paste/Compound** - [Wikipedia](https://en.wikipedia.org/wiki/Thermal_grease)
  - **Thermal Pads** - Gap-filling thermal interface
  - **Phase Change Materials** - [Wikipedia](https://en.wikipedia.org/wiki/Phase-change_material)
  - **Liquid Metal** - Gallium-based thermal compound
  - **Graphite Sheets** - Thermal interface pads
- **Heat Spreaders** - [Wikipedia](https://en.wikipedia.org/wiki/Heat_spreader)
- **IHS (Integrated Heat Spreader)** - CPU heat spreader

### Active Cooling
- **Air Cooling** - [Wikipedia](https://en.wikipedia.org/wiki/Computer_cooling#Air_cooling)
  - **CPU Air Coolers** - Tower and downdraft coolers
  - **Case Fans** - [Wikipedia](https://en.wikipedia.org/wiki/Computer_fan)
  - **PWM Fans** - [Wikipedia](https://en.wikipedia.org/wiki/Pulse-width_modulation#Servos) - Pulse-width modulation controlled
  - **DC Fans** - Voltage-controlled fans
  - **Blower Fans** - Centrifugal fans
- **Liquid Cooling** - [Wikipedia](https://en.wikipedia.org/wiki/Computer_cooling#Liquid_cooling)
  - **AIO (All-In-One) Coolers** - Closed-loop liquid coolers
  - **Custom Water Cooling** - Open-loop systems
  - **CPU Water Blocks** - Liquid cooling blocks
  - **Radiators** - Heat exchangers
  - **Pumps** - Coolant circulation
  - **Reservoirs** - Coolant storage
  - **Coolant** - Water/glycol mixtures
- **Thermoelectric Cooling (Peltier)** - [Wikipedia](https://en.wikipedia.org/wiki/Thermoelectric_cooling)
- **Phase Change Cooling** - [Wikipedia](https://en.wikipedia.org/wiki/Computer_cooling#Phase-change_cooling)
- **Immersion Cooling** - [Wikipedia](https://en.wikipedia.org/wiki/Immersion_cooling)
  - **Single-Phase Immersion** - Liquid cooling bath
  - **Two-Phase Immersion** - Boiling liquid cooling
- **Direct-to-Chip Cooling** - Server liquid cooling

### Passive Cooling
- **Heatsinks** - [Wikipedia](https://en.wikipedia.org/wiki/Heat_sink)
  - **Aluminum Heatsinks** - Common heatsink material
  - **Copper Heatsinks** - High thermal conductivity
  - **Finned Heatsinks** - Increased surface area
- **Fanless Cooling** - [Wikipedia](https://en.wikipedia.org/wiki/Fanless_computer)
- **Chassis Cooling** - Case-based passive cooling
- **Thermal Throttling** - [Wikipedia](https://en.wikipedia.org/wiki/Dynamic_frequency_scaling) - Automatic speed reduction

## Connectors & Cables

### Power Connectors
- **ATX 24-pin** - [Wikipedia](https://en.wikipedia.org/wiki/ATX#Connectors) - Main motherboard power
- **ATX 4/8-pin EPS** - CPU power connector
- **PCIe 6/8-pin** - Graphics card power
- **12VHPWR** - PCIe 5.0 power connector (16-pin)
- **SATA Power** - [Wikipedia](https://en.wikipedia.org/wiki/Serial_ATA#Power_connectors) - 15-pin power
- **Molex** - [Wikipedia](https://en.wikipedia.org/wiki/Molex_connector) - 4-pin peripheral power
- **Berg Connector** - Floppy drive power (4-pin mini)
- **Barrel Jack** - [Wikipedia](https://en.wikipedia.org/wiki/Coaxial_power_connector) - DC power connector
- **USB-C Power** - USB Type-C power delivery
- **XT60/XT90** - High-current connectors (hobby/drone)
- **Anderson Powerpole** - Modular power connectors

### Data Connectors
- **USB Connectors** - [Wikipedia](https://en.wikipedia.org/wiki/USB#Connector_types)
  - **USB Type-A** - Standard USB connector
  - **USB Type-B** - Printer/device connector
  - **USB Type-C** - Reversible connector
  - **Mini-USB** - Older small connector
  - **Micro-USB** - Mobile device connector
- **SATA Data** - [Wikipedia](https://en.wikipedia.org/wiki/Serial_ATA#SATA_data_cable) - 7-pin data
- **M.2 Connector** - [Wikipedia](https://en.wikipedia.org/wiki/M.2) - SSD/WiFi connector
- **U.2/U.3 Connector** - Enterprise SSD connector
- **RJ45** - [Wikipedia](https://en.wikipedia.org/wiki/Modular_connector#8P8C) - Ethernet connector
- **SFP/SFP+/QSFP** - [Wikipedia](https://en.wikipedia.org/wiki/Small_form-factor_pluggable_transceiver) - Fiber/copper modules
- **coaxial (F-type, BNC)** - [Wikipedia](https://en.wikipedia.org/wiki/Coaxial_cable) - RF connectors
- **Fiber Connectors** - [Wikipedia](https://en.wikipedia.org/wiki/Optical_fiber_connector)
  - **LC** - [Wikipedia](https://en.wikipedia.org/wiki/Optical_fiber_connector#LC) - Lucent Connector
  - **SC** - [Wikipedia](https://en.wikipedia.org/wiki/Optical_fiber_connector#SC) - Subscriber Connector
  - **ST** - [Wikipedia](https://en.wikipedia.org/wiki/Optical_fiber_connector#ST) - Straight Tip
  - **MPO/MTP** - [Wikipedia](https://en.wikipedia.org/wiki/Optical_fiber_connector#MPO/MTP) - Multi-fiber

### Board-to-Board Connectors
- **FPC/FFC (Flexible Flat Cable)** - [Wikipedia](https://en.wikipedia.org/wiki/Flexible_flat_cable)
- **IDC (Insulation-Displacement Connector)** - [Wikipedia](https://en.wikipedia.org/wiki/Insulation-displacement_connector)
- **Pin Headers** - [Wikipedia](https://en.wikipedia.org/wiki/Pin_header)
- **Pogo Pins** - [Wikipedia](https://en.wikipedia.org/wiki/Pogo_pin) - Spring-loaded contacts
- **Edge Connectors** - [Wikipedia](https://en.wikipedia.org/wiki/Edge_connector)
- **Card Edge Connectors** - PCIe, PCI, etc.
- **ZIF (Zero Insertion Force)** - [Wikipedia](https://en.wikipedia.org/wiki/Zero_insertion_force) - CPU sockets

### Cable Types
- **Copper Cables** - [Wikipedia](https://en.wikipedia.org/wiki/Copper_conductor)
  - **Cat5e/Cat6/Cat6a/Cat7/Cat8** - [Wikipedia](https://en.wikipedia.org/wiki/Category_6_cable) - Ethernet cables
  - **Twinax** - [Wikipedia](https://en.wikipedia.org/wiki/Twinaxial_cabling) - Twin coaxial
  - **Coaxial** - [Wikipedia](https://en.wikipedia.org/wiki/Coaxial_cable)
- **Fiber Optic Cables** - [Wikipedia](https://en.wikipedia.org/wiki/Optical_fiber)
  - **Single-Mode Fiber (SMF)** - [Wikipedia](https://en.wikipedia.org/wiki/Single-mode_optical_fiber)
  - **Multi-Mode Fiber (MMF)** - [Wikipedia](https://en.wikipedia.org/wiki/Multi-mode_optical_fiber)
  - **OM1/OM2/OM3/OM4/OM5** - Multi-mode fiber grades
  - **OS1/OS2** - Single-mode fiber grades
- **Active Optical Cables (AOC)** - [Wikipedia](https://en.wikipedia.org/wiki/Active_optical_cable)
- **Direct Attach Cables (DAC)** - [Wikipedia](https://en.wikipedia.org/wiki/Twinaxial_cabling#Direct-attach_copper_(DAC)) - Passive copper

## Mechanical Components

### Computer Cases
- **Desktop Cases** - [Wikipedia](https://en.wikipedia.org/wiki/Computer_case)
  - **Full Tower** - Large ATX case
  - **Mid Tower** - Standard ATX case
  - **Mini Tower** - Micro-ATX case
  - **Small Form Factor (SFF)** - [Wikipedia](https://en.wikipedia.org/wiki/Small_form_factor) - Compact cases
- **Rack Mount Cases** - [Wikipedia](https://en.wikipedia.org/wiki/19-inch_rack)
- **Open-Air Cases** - Test bench style
- **HTPC Cases** - [Wikipedia](https://en.wikipedia.org/wiki/Home_theater_PC) - Media center cases

### Server Racks
- **19-inch Rack** - [Wikipedia](https://en.wikipedia.org/wiki/19-inch_rack) - Standard server rack
- **23-inch Rack** - Telecom rack standard
- **Open Rack** - [Wikipedia](https://en.wikipedia.org/wiki/Open_Compute_Project#Open_Rack) - Open Compute Project
- **Rack Units (U)** - [Wikipedia](https://en.wikipedia.org/wiki/Rack_unit) - 1U = 1.75 inches
- **Rack PDUs** - [Wikipedia](https://en.wikipedia.org/wiki/Power_distribution_unit) - Power distribution
- **Cable Management** - Cable organization systems
- **Rack Rails** - Mounting hardware

### Enclosures
- **Drive Enclosures** - External HDD/SSD enclosures
- **NAS Enclosures** - Network-attached storage cases
- **DAS Enclosures** - Direct-attached storage
- **Hot-Swap Bays** - Tool-less drive bays
- **Backplanes** - Multi-drive connectivity boards
- **Industrial Enclosures** - Ruggedized cases
- **IP-Rated Enclosures** - [Wikipedia](https://en.wikipedia.org/wiki/IP_Code) - Weatherproof cases
- **NEMA Enclosures** - [Wikipedia](https://en.wikipedia.org/wiki/NEMA_enclosure_types)

### Mounting Hardware
- **Standoffs** - PCB mounting spacers
- **Screws** - M3, M2.5, 6-32, M.2 screws
- **Washers** - Insulating and grounding washers
- **Thermal Pads** - Mounting thermal interfaces
- **Adhesives** - Thermal adhesive, double-sided tape
- **Clips & Brackets** - Component mounting hardware

## Hardware Design & Manufacturing

### PCB Design
- **PCB (Printed Circuit Board)** - [Wikipedia](https://en.wikipedia.org/wiki/Printed_circuit_board)
- **Single-Layer PCB** - One copper layer
- **Multi-Layer PCB** - [Wikipedia](https://en.wikipedia.org/wiki/Printed_circuit_board#Multi-layer_boards) - 2, 4, 6, 8+ layers
- **HDI (High-Density Interconnect)** - [Wikipedia](https://en.wikipedia.org/wiki/High-density_interconnect) - Microvias
- **Rigid PCB** - Standard PCB
- **Flex PCB** - [Wikipedia](https://en.wikipedia.org/wiki/Flexible_electronics) - Flexible circuits
- **Rigid-Flex PCB** - [Wikipedia](https://en.wikipedia.org/wiki/Flexible_electronics#Rigid-Flex) - Combined rigid and flex
- **Metal Core PCB (MCPCB)** - [Wikipedia](https://en.wikipedia.org/wiki/Metal_core_printed_circuit_board) - Thermal management
- **RF PCB** - Radio frequency boards
- **High-Speed PCB Design** - Signal integrity considerations
- **Impedance Control** - [Wikipedia](https://en.wikipedia.org/wiki/Controlled_impedance) - Controlled impedance traces

### IC Packaging
- **Wafer Fabrication** - [Wikipedia](https://en.wikipedia.org/wiki/Wafer_(electronics))
- **Die Bonding** - [Wikipedia](https://en.wikipedia.org/wiki/Die_bonding)
- **Wire Bonding** - [Wikipedia](https://en.wikipedia.org/wiki/Wire_bonding)
- **Flip Chip** - [Wikipedia](https://en.wikipedia.org/wiki/Flip_chip)
- **Underfill** - Epoxy protection for flip chips
- **Molding/Encapsulation** - IC package protection
- **Package-on-Package (PoP)** - [Wikipedia](https://en.wikipedia.org/wiki/Package_on_package) - Stacked packages

### Assembly Technologies
- **SMT (Surface-Mount Technology)** - [Wikipedia](https://en.wikipedia.org/wiki/Surface-mount_technology)
- **Through-Hole Technology** - [Wikipedia](https://en.wikipedia.org/wiki/Through-hole_technology)
- **Pick and Place** - [Wikipedia](https://en.wikipedia.org/wiki/SMT_placement_equipment) - Component placement
- **Reflow Soldering** - [Wikipedia](https://en.wikipedia.org/wiki/Reflow_soldering)
- **Wave Soldering** - [Wikipedia](https://en.wikipedia.org/wiki/Wave_soldering)
- **Selective Soldering** - [Wikipedia](https://en.wikipedia.org/wiki/Selective_soldering)
- **BGA Reballing** - [Wikipedia](https://en.wikipedia.org/wiki/Ball_grid_array#Reballing)
- **Conformal Coating** - [Wikipedia](https://en.wikipedia.org/wiki/Conformal_coating) - PCB protection
- **AOI (Automated Optical Inspection)** - [Wikipedia](https://en.wikipedia.org/wiki/Automated_optical_inspection)
- **X-Ray Inspection** - BGA and hidden solder joint inspection

### Design Tools
- **EDA (Electronic Design Automation)** - [Wikipedia](https://en.wikipedia.org/wiki/Electronic_design_automation)
- **Altium Designer** - [Wikipedia](https://en.wikipedia.org/wiki/Altium_Designer) | [Altium](https://www.altium.com/)
- **KiCad** - [Wikipedia](https://en.wikipedia.org/wiki/KiCad) | [KiCad](https://www.kicad.org/) - Open-source EDA
- **Eagle (Autodesk)** - [Wikipedia](https://en.wikipedia.org/wiki/EAGLE_(program))
- **OrCAD** - [Wikipedia](https://en.wikipedia.org/wiki/OrCAD)
- **Cadence Allegro** - [Wikipedia](https://en.wikipedia.org/wiki/Cadence_Design_Systems) - Professional PCB design
- **Mentor Graphics** - [Wikipedia](https://en.wikipedia.org/wiki/Siemens_EDA) - Now Siemens EDA
- **SPICE Simulation** - [Wikipedia](https://en.wikipedia.org/wiki/SPICE) - Circuit simulation
- **Verilog/VHDL** - [Wikipedia](https://en.wikipedia.org/wiki/Verilog) - Hardware description languages
- **SystemVerilog** - [Wikipedia](https://en.wikipedia.org/wiki/SystemVerilog)

## Emerging Technologies

### Quantum Computing
- **Superconducting Qubits** - [Wikipedia](https://en.wikipedia.org/wiki/Superconducting_quantum_computing)
- **Trapped Ion Qubits** - [Wikipedia](https://en.wikipedia.org/wiki/Trapped_ion_quantum_computer)
- **Photonic Qubits** - [Wikipedia](https://en.wikipedia.org/wiki/Linear_optical_quantum_computing)

### Neuromorphic Computing
- **Intel Loihi** - [Wikipedia](https://en.wikipedia.org/wiki/Neuromorphic_engineering#Intel_Loihi) | [Intel Loihi](https://www.intel.com/content/www/us/en/research/neuromorphic-computing.html)
- **IBM TrueNorth** - [Wikipedia](https://en.wikipedia.org/wiki/TrueNorth)
- **Memristors** - [Wikipedia](https://en.wikipedia.org/wiki/Memristor)

### Optical Computing
- **Silicon Photonics** - [Wikipedia](https://en.wikipedia.org/wiki/Silicon_photonics)
- **Optical Interconnects** - [Wikipedia](https://en.wikipedia.org/wiki/Optical_communication)

### DNA Storage
- **DNA Data Storage** - [Wikipedia](https://en.wikipedia.org/wiki/DNA_digital_data_storage)

### Carbon Nanotube Electronics
- **CNT (Carbon Nanotubes)** - [Wikipedia](https://en.wikipedia.org/wiki/Carbon_nanotube)
- **CNT Transistors** - [Wikipedia](https://en.wikipedia.org/wiki/Carbon_nanotube_field-effect_transistor)
- **CNT Interconnects** - [Wikipedia](https://en.wikipedia.org/wiki/Carbon_nanotube#Electronics)

### Spintronics
- **Spin Electronics** - [Wikipedia](https://en.wikipedia.org/wiki/Spintronics)
- **GMR (Giant Magnetoresistance)** - [Wikipedia](https://en.wikipedia.org/wiki/Giant_magnetoresistance)
- **TMR (Tunneling Magnetoresistance)** - [Wikipedia](https://en.wikipedia.org/wiki/Tunnel_magnetoresistance)
- **Spin Transfer Torque** - [Wikipedia](https://en.wikipedia.org/wiki/Spin-transfer_torque)
- **Spin Hall Effect** - [Wikipedia](https://en.wikipedia.org/wiki/Spin_Hall_effect)

## Specialized Computing

### AI/ML Accelerators
- **Google TPU (Tensor Processing Unit)** - [Wikipedia](https://en.wikipedia.org/wiki/Tensor_Processing_Unit) | [Google Cloud TPU](https://cloud.google.com/tpu)
  - **TPU v4** - Current generation TPU pods
  - **TPU v5e** - Efficient training and inference
  - **TPU v5p** - High-performance AI training
- **AWS Trainium** - [Wikipedia](https://en.wikipedia.org/wiki/AWS_Trainium) | [AWS Trainium](https://aws.amazon.com/machine-learning/trainium/) - Custom AI training chip
  - **Trn1 Instances** - Trainium-powered EC2 instances
- **AWS Inferentia** - [Wikipedia](https://en.wikipedia.org/wiki/AWS_Inferentia) | [AWS Inferentia](https://aws.amazon.com/machine-learning/inferentia/) - AI inference chip
  - **Inferentia1** - First generation inference
  - **Inferentia2** - Second generation inference
- **Tesla Dojo** - [Wikipedia](https://en.wikipedia.org/wiki/Tesla_Dojo) - Tesla's AI training supercomputer
  - **Dojo D1 Chip** - Custom training ASIC (7nm, 362 teraFLOPS)
  - **Training Tiles** - 25 D1 chips per tile
  - **ExaPod** - Complete Dojo system
- **Groq LPU (Language Processing Unit)** - [Groq](https://groq.com/) - Deterministic AI inference processor
  - **Groq GroqChip** - Temporal computing architecture
- **Intel Habana** - [Wikipedia](https://en.wikipedia.org/wiki/Habana_Labs) | [Intel Habana](https://habana.ai/)
  - **Gaudi2** - AI training accelerator
  - **Gaudi3** - Next-gen training (2024)
  - **Goya** - AI inference processor
- **Graphcore IPU (Intelligence Processing Unit)** - [Wikipedia](https://en.wikipedia.org/wiki/Graphcore) | [Graphcore](https://www.graphcore.ai/)
  - **Bow IPU** - Latest generation (Bow-2000)
  - **IPU-M2000** - Machine intelligence accelerator
  - **IPU-POD** - Scalable AI infrastructure
- **Cerebras WSE (Wafer-Scale Engine)** - [Wikipedia](https://en.wikipedia.org/wiki/Cerebras) | [Cerebras](https://www.cerebras.net/)
  - **WSE-3** - Third generation (4 trillion transistors)
  - **CS-3** - Complete AI system with WSE-3
  - **CS-2** - Previous generation system
- **SambaNova DataScale** - [SambaNova](https://sambanova.ai/) - Reconfigurable dataflow architecture
  - **SN40L** - DataScale SN40L chip
  - **DataScale SN30** - AI platform
- **Tenstorrent** - [Tenstorrent](https://www.tenstorrent.com/) - RISC-V based AI processors
  - **Grayskull** - AI training and inference
  - **Wormhole** - Next-gen AI chip
  - **Blackhole** - Future architecture
- **Mythic AI** - [Mythic](https://www.mythic-ai.com/) - Analog matrix processor
  - **Mythic M1076** - Analog AI compute chip
- **Esperanto ET-SoC** - [Esperanto](https://www.esperanto.ai/) - RISC-V AI chip (1000+ cores)
  - **ET-SoC-1** - ML inference processor
- **Biren BR100** - [Biren Technology](https://www.birentech.com/) - Chinese AI GPU (77 billion transistors)
- **Huawei Ascend** - [Wikipedia](https://en.wikipedia.org/wiki/Ascend_(processor)) - Huawei AI processors
  - **Ascend 910B** - AI training chip (7nm)
  - **Ascend 310** - AI inference chip
  - **Ascend 910** - Previous gen training
- **Alibaba Hanguang 800** - NPU for AI inference (12nm)
- **Baidu Kunlun** - [Wikipedia](https://en.wikipedia.org/wiki/Baidu#Artificial_intelligence) - Baidu AI chip
  - **Kunlun 2** - Second generation AI chip
- **Apple Neural Engine** - [Wikipedia](https://en.wikipedia.org/wiki/Apple_Neural_Engine) - Integrated in Apple Silicon
  - **16-core Neural Engine** - M3/A17 Pro generation
- **Qualcomm AI Engine** - NPU in Snapdragon SoCs
  - **Hexagon NPU** - Qualcomm's AI accelerator
- **MediaTek APU (AI Processing Unit)** - AI accelerators in Dimensity SoCs
  - **APU 690** - Latest generation AI unit
- **Samsung NPU** - Neural Processing Unit in Exynos SoCs
- **Moffett AI** - [Moffett AI](https://www.moffett.ai/) - AI inference chips (Antoum series)
- **EdgeCortix** - [EdgeCortix](https://www.edgecortix.com/) - MERA AI accelerators
- **Cambricon** - [Wikipedia](https://en.wikipedia.org/wiki/Cambricon) - Chinese AI chip company
  - **MLU370** - AI inference accelerator
  - **MLU290** - AI training chip

### FPGA Technologies
- **Xilinx (AMD) FPGAs** - [Wikipedia](https://en.wikipedia.org/wiki/Xilinx) | [AMD Xilinx](https://www.xilinx.com/products/silicon-devices/fpga.html)
- **Intel (Altera) FPGAs** - [Wikipedia](https://en.wikipedia.org/wiki/Altera) | [Intel FPGAs](https://www.intel.com/content/www/us/en/products/details/fpga.html)
- **Lattice FPGAs** - [Wikipedia](https://en.wikipedia.org/wiki/Lattice_Semiconductor) | [Lattice FPGAs](https://www.latticesemi.com/Products/FPGAandCPLD)

### DSP Technologies
- **Texas Instruments DSPs** - [Wikipedia](https://en.wikipedia.org/wiki/Digital_signal_processor) | [TI DSPs](https://www.ti.com/processors/digital-signal-processors/overview.html)
- **Analog Devices DSPs** - [Wikipedia](https://en.wikipedia.org/wiki/Analog_Devices) | [ADI DSPs](https://www.analog.com/en/products/processors-dsp.html)
- **SHARC DSPs** - [Wikipedia](https://en.wikipedia.org/wiki/Super_Harvard_Architecture_Single-Chip_Computer) - Analog Devices
- **Blackfin DSPs** - [Wikipedia](https://en.wikipedia.org/wiki/Blackfin) - Analog Devices
- **TMS320 Family** - [Wikipedia](https://en.wikipedia.org/wiki/Texas_Instruments_TMS320) - Texas Instruments

### Cryptographic Accelerators
- **Hardware Security Modules (HSM)** - [Wikipedia](https://en.wikipedia.org/wiki/Hardware_security_module)
- **Crypto Coprocessors** - Dedicated cryptography processors
- **AES Accelerators** - [Wikipedia](https://en.wikipedia.org/wiki/AES_instruction_set) - Hardware AES encryption
- **SHA Accelerators** - Hardware SHA hashing
- **RSA Accelerators** - Public-key cryptography acceleration
- **ECC Accelerators** - Elliptic curve cryptography
- **TRNG (True Random Number Generator)** - [Wikipedia](https://en.wikipedia.org/wiki/Hardware_random_number_generator)
- **TPM (Trusted Platform Module)** - [Wikipedia](https://en.wikipedia.org/wiki/Trusted_Platform_Module)
- **Secure Enclave** - Apple's security coprocessor
- **Titan M (Google)** - Security chip for Pixel devices
- **Cavium/Marvell Nitrox** - Crypto acceleration cards

### Network Accelerators
- **SmartNICs** - [Wikipedia](https://en.wikipedia.org/wiki/Network_interface_controller#Smart_NIC) - Programmable network cards
- **DPU (Data Processing Unit)** - [Wikipedia](https://en.wikipedia.org/wiki/Data_processing_unit)
  - **NVIDIA BlueField DPU** - [NVIDIA BlueField](https://www.nvidia.com/en-us/networking/products/data-processing-unit/)
  - **AMD Pensando DSC** - Distributed Services Cards
  - **Intel IPU (Infrastructure Processing Unit)** - [Intel IPU](https://www.intel.com/content/www/us/en/products/details/network-io/ipu.html)
- **TCP Offload Engine (TOE)** - [Wikipedia](https://en.wikipedia.org/wiki/TCP_offload_engine)
- **RDMA NICs** - Remote Direct Memory Access cards
- **Packet Processing Engines** - Hardware packet processing
- **Traffic Managers** - QoS and traffic shaping hardware
- **Load Balancers** - Hardware load balancing

### Storage Accelerators
- **NVMe-oF Accelerators** - NVMe over Fabrics offload
- **Compression Accelerators** - Hardware data compression
  - **Intel QAT (QuickAssist Technology)** - [Wikipedia](https://en.wikipedia.org/wiki/Intel_QuickAssist_Technology)
  - **Cavium/Marvell ZIP** - Compression acceleration
- **Deduplication Accelerators** - Hardware deduplication
- **Erasure Coding Accelerators** - Reed-Solomon and other EC
- **RAID Accelerators** - Hardware RAID processing
- **Encryption Accelerators** - Storage encryption offload

## Standards Organizations

### Industry Consortia
- **PCI-SIG (PCI Special Interest Group)** - [Wikipedia](https://en.wikipedia.org/wiki/PCI-SIG) | [PCI-SIG](https://pcisig.com/) - PCIe standards
- **USB Implementers Forum (USB-IF)** - [Wikipedia](https://en.wikipedia.org/wiki/USB_Implementers_Forum) | [USB-IF](https://www.usb.org/)
- **JEDEC (Joint Electron Device Engineering Council)** - [Wikipedia](https://en.wikipedia.org/wiki/JEDEC) | [JEDEC](https://www.jedec.org/) - Memory and semiconductor standards
- **VESA (Video Electronics Standards Association)** - [Wikipedia](https://en.wikipedia.org/wiki/VESA) | [VESA](https://www.vesa.org/) - DisplayPort, VESA mounts
- **SATA-IO** - [Wikipedia](https://en.wikipedia.org/wiki/Serial_ATA#SATA-IO) | [SATA-IO](https://www.sata-io.org/) - SATA standards
- **NVM Express (NVMe)** - [Wikipedia](https://en.wikipedia.org/wiki/NVM_Express) | [NVMe](https://nvmexpress.org/) - NVMe standards
- **Khronos Group** - [Wikipedia](https://en.wikipedia.org/wiki/Khronos_Group) | [Khronos](https://www.khronos.org/) - OpenGL, Vulkan, OpenCL
- **Bluetooth SIG** - [Wikipedia](https://en.wikipedia.org/wiki/Bluetooth_Special_Interest_Group) | [Bluetooth SIG](https://www.bluetooth.com/)
- **Wi-Fi Alliance** - [Wikipedia](https://en.wikipedia.org/wiki/Wi-Fi_Alliance) | [Wi-Fi Alliance](https://www.wi-fi.org/)
- **HDMI Forum** - [Wikipedia](https://en.wikipedia.org/wiki/HDMI#HDMI_Licensing,_LLC) | [HDMI](https://www.hdmi.org/)
- **CXL Consortium** - [Wikipedia](https://en.wikipedia.org/wiki/Compute_Express_Link) | [CXL](https://www.computeexpresslink.org/) - Compute Express Link
- **UCIe (Universal Chiplet Interconnect Express)** - [UCIe](https://www.uciexpress.org/) - Chiplet standards
- **Open Compute Project (OCP)** - [Wikipedia](https://en.wikipedia.org/wiki/Open_Compute_Project) | [OCP](https://www.opencompute.org/)
- **RISC-V International** - [Wikipedia](https://en.wikipedia.org/wiki/RISC-V_International) | [RISC-V](https://riscv.org/)
- **InfiniBand Trade Association** - [Wikipedia](https://en.wikipedia.org/wiki/InfiniBand_Trade_Association) | [IBTA](https://www.infinibandta.org/)
- **Gen-Z Consortium** - [Wikipedia](https://en.wikipedia.org/wiki/Gen-Z_(interconnect))
- **CCIX (Cache Coherent Interconnect for Accelerators)** - Defunct accelerator interconnect

### Standards Bodies
- **IEEE (Institute of Electrical and Electronics Engineers)** - [Wikipedia](https://en.wikipedia.org/wiki/Institute_of_Electrical_and_Electronics_Engineers) | [IEEE](https://www.ieee.org/)
  - **IEEE 802.3** - Ethernet standards
  - **IEEE 802.11** - Wi-Fi standards
  - **IEEE 1394** - FireWire
  - **IEEE 1588** - Precision Time Protocol (PTP)
- **ISO (International Organization for Standardization)** - [Wikipedia](https://en.wikipedia.org/wiki/International_Organization_for_Standardization) | [ISO](https://www.iso.org/)
- **IEC (International Electrotechnical Commission)** - [Wikipedia](https://en.wikipedia.org/wiki/International_Electrotechnical_Commission) | [IEC](https://www.iec.ch/)
- **ANSI (American National Standards Institute)** - [Wikipedia](https://en.wikipedia.org/wiki/American_National_Standards_Institute) | [ANSI](https://www.ansi.org/)
- **IETF (Internet Engineering Task Force)** - [Wikipedia](https://en.wikipedia.org/wiki/Internet_Engineering_Task_Force) | [IETF](https://www.ietf.org/) - Internet protocols (RFCs)
- **3GPP (3rd Generation Partnership Project)** - [Wikipedia](https://en.wikipedia.org/wiki/3GPP) | [3GPP](https://www.3gpp.org/) - Mobile telecom standards
- **DMTF (Distributed Management Task Force)** - [Wikipedia](https://en.wikipedia.org/wiki/Distributed_Management_Task_Force) | [DMTF](https://www.dmtf.org/) - Redfish, CIM
- **UEFI Forum** - [Wikipedia](https://en.wikipedia.org/wiki/UEFI_Forum) | [UEFI](https://uefi.org/) - UEFI and ACPI specs
- **Trusted Computing Group (TCG)** - [Wikipedia](https://en.wikipedia.org/wiki/Trusted_Computing_Group) | [TCG](https://trustedcomputinggroup.org/) - TPM specs
- **ETSI (European Telecommunications Standards Institute)** - [Wikipedia](https://en.wikipedia.org/wiki/ETSI) | [ETSI](https://www.etsi.org/)
- **ITU (International Telecommunication Union)** - [Wikipedia](https://en.wikipedia.org/wiki/International_Telecommunication_Union) | [ITU](https://www.itu.int/)

## Testing and Validation

### Test Standards
- **JEDEC Standards** - [Wikipedia](https://en.wikipedia.org/wiki/JEDEC) | [JEDEC Standards](https://www.jedec.org/standards-documents)
- **PCI-SIG Compliance** - [Wikipedia](https://en.wikipedia.org/wiki/PCI-SIG) | [PCI-SIG](https://pcisig.com/)
- **USB-IF Compliance** - [Wikipedia](https://en.wikipedia.org/wiki/USB_Implementers_Forum) | [USB-IF](https://www.usb.org/)
- **HDMI Compliance** - [HDMI Compliance](https://www.hdmi.org/adopter/compliance-testing-information.html)
- **DisplayPort Compliance** - [VESA DisplayPort](https://www.displayport.org/product-certification/)
- **Bluetooth Qualification** - [Bluetooth SIG](https://www.bluetooth.com/develop-with-bluetooth/qualification-listing/)
- **Wi-Fi Certification** - [Wi-Fi Alliance](https://www.wi-fi.org/certification)

### Reliability Testing
- **HALT (Highly Accelerated Life Testing)** - [Wikipedia](https://en.wikipedia.org/wiki/Highly_accelerated_life_testing)
- **HASS (Highly Accelerated Stress Screening)** - [Wikipedia](https://en.wikipedia.org/wiki/Highly_accelerated_stress_screening)
- **Burn-In Testing** - [Wikipedia](https://en.wikipedia.org/wiki/Burn-in) - Infant mortality screening
- **Temperature Cycling** - [Wikipedia](https://en.wikipedia.org/wiki/Temperature_cycling) - Thermal stress testing
- **Vibration Testing** - [Wikipedia](https://en.wikipedia.org/wiki/Vibration_testing) - Mechanical stress
- **Shock Testing** - [Wikipedia](https://en.wikipedia.org/wiki/Shock_testing) - Impact resistance
- **Humidity Testing** - Environmental chamber testing
- **Salt Spray Testing** - [Wikipedia](https://en.wikipedia.org/wiki/Salt_spray_test) - Corrosion resistance
- **EMI/EMC Testing** - [Wikipedia](https://en.wikipedia.org/wiki/Electromagnetic_compatibility) - Electromagnetic compatibility
  - **FCC Part 15** - [Wikipedia](https://en.wikipedia.org/wiki/FCC_Part_15) - US EMC regulations
  - **CE Marking** - [Wikipedia](https://en.wikipedia.org/wiki/CE_marking) - European EMC compliance
  - **CISPR** - [Wikipedia](https://en.wikipedia.org/wiki/CISPR) - International EMI standards
- **ESD Testing** - [Wikipedia](https://en.wikipedia.org/wiki/Electrostatic_discharge) - Electrostatic discharge
- **MTBF (Mean Time Between Failures)** - [Wikipedia](https://en.wikipedia.org/wiki/Mean_time_between_failures)
- **MTTF (Mean Time To Failure)** - [Wikipedia](https://en.wikipedia.org/wiki/Mean_time_to_failure)
- **Accelerated Aging** - Lifecycle simulation testing

### Certification Programs
- **80 PLUS** - [Wikipedia](https://en.wikipedia.org/wiki/80_Plus) | [80 Plus](https://www.80plus.org/) - Power supply efficiency
- **Energy Star** - [Wikipedia](https://en.wikipedia.org/wiki/Energy_Star) | [Energy Star](https://www.energystar.gov/) - Energy efficiency
- **EPEAT** - [Wikipedia](https://en.wikipedia.org/wiki/EPEAT) | [EPEAT](https://www.epeat.net/) - Environmental standards
- **RoHS (Restriction of Hazardous Substances)** - [Wikipedia](https://en.wikipedia.org/wiki/Restriction_of_Hazardous_Substances_Directive)
- **WEEE (Waste Electrical and Electronic Equipment)** - [Wikipedia](https://en.wikipedia.org/wiki/Waste_Electrical_and_Electronic_Equipment_Directive)
- **UL Certification** - [Wikipedia](https://en.wikipedia.org/wiki/UL_(safety_organization)) | [UL](https://www.ul.com/) - Safety certification
- **CSA Certification** - [Wikipedia](https://en.wikipedia.org/wiki/CSA_Group) - Canadian Standards Association
- **TÜV Certification** - [Wikipedia](https://en.wikipedia.org/wiki/T%C3%9CV) - German technical certification
- **FCC Certification** - [Wikipedia](https://en.wikipedia.org/wiki/Federal_Communications_Commission) - US radio/EMI certification
- **IC (Innovation, Science and Economic Development Canada)** - Canadian radio certification
- **CE Marking** - [Wikipedia](https://en.wikipedia.org/wiki/CE_marking) - European Conformity
- **CCC (China Compulsory Certificate)** - [Wikipedia](https://en.wikipedia.org/wiki/China_Compulsory_Certificate) - Chinese certification
- **KC Mark** - Korean Certification
- **VCCI** - [Wikipedia](https://en.wikipedia.org/wiki/Voluntary_Control_Council_for_Interference_by_Information_Technology_Equipment) - Japanese EMC
- **ISO 9001** - [Wikipedia](https://en.wikipedia.org/wiki/ISO_9001) - Quality management
- **ISO 14001** - [Wikipedia](https://en.wikipedia.org/wiki/ISO_14001) - Environmental management
- **MIL-STD Compliance** - [Wikipedia](https://en.wikipedia.org/wiki/United_States_Military_Standard) - Military standards
- **ITAR Compliance** - [Wikipedia](https://en.wikipedia.org/wiki/International_Traffic_in_Arms_Regulations) - Defense export control
- **Common Criteria** - [Wikipedia](https://en.wikipedia.org/wiki/Common_Criteria) - IT security certification
- **FIPS 140-2/140-3** - [Wikipedia](https://en.wikipedia.org/wiki/FIPS_140-2) - Cryptographic module validation

