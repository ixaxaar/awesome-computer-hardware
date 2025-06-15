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
        - [Instruction Set Extensions](#instruction-set-extensions)
            - [x86 SIMD Extensions](#x86-simd-extensions)
            - [x86 Specialized Extensions](#x86-specialized-extensions)
            - [ARM SIMD Extensions](#arm-simd-extensions)
            - [PowerPC SIMD Extensions](#powerpc-simd-extensions)
            - [RISC-V Vector Extensions](#risc-v-vector-extensions)
            - [Other Architecture Extensions](#other-architecture-extensions)
        - [Virtualization Technologies](#virtualization-technologies)
        - [Security Technologies](#security-technologies)
    - [Fabrication Processes](#fabrication-processes)
        - [Current Process Nodes](#current-process-nodes)
        - [Mature Process Nodes](#mature-process-nodes)
        - [Future Process Nodes](#future-process-nodes)
        - [Manufacturing Technologies](#manufacturing-technologies)
    - [Memory Technologies](#memory-technologies)
        - [Volatile Memory](#volatile-memory)
        - [Non-Volatile Memory](#non-volatile-memory)
    - [Storage Technologies](#storage-technologies)
        - [Storage Interfaces](#storage-interfaces)
        - [Storage Protocols](#storage-protocols)
    - [Interconnect Technologies](#interconnect-technologies)
        - [System Buses](#system-buses)
        - [High-Speed Interconnects](#high-speed-interconnects)
        - [CPU Interconnects](#cpu-interconnects)
    - [Graphics Technologies](#graphics-technologies)
        - [GPU Architectures](#gpu-architectures)
        - [Graphics APIs](#graphics-apis)
        - [Ray Tracing Technologies](#ray-tracing-technologies)
    - [Network Technologies](#network-technologies)
        - [Ethernet Standards](#ethernet-standards)
        - [Wireless Technologies](#wireless-technologies)
        - [Network Protocols](#network-protocols)
    - [I/O Technologies](#io-technologies)
        - [Universal Serial Bus (USB)](#universal-serial-bus-usb)
        - [Display Interfaces](#display-interfaces)
        - [Audio Interfaces](#audio-interfaces)
    - [Chipset Technologies](#chipset-technologies)
        - [Intel Chipsets](#intel-chipsets)
        - [AMD Chipsets](#amd-chipsets)
    - [Power Management](#power-management)
        - [Power Standards](#power-standards)
        - [CPU Power Technologies](#cpu-power-technologies)
    - [Firmware Technologies](#firmware-technologies)
        - [Boot Technologies](#boot-technologies)
        - [Management Technologies](#management-technologies)
    - [Cooling Technologies](#cooling-technologies)
        - [Thermal Management](#thermal-management)
    - [Emerging Technologies](#emerging-technologies)
        - [Quantum Computing](#quantum-computing)
        - [Neuromorphic Computing](#neuromorphic-computing)
        - [Optical Computing](#optical-computing)
        - [DNA Storage](#dna-storage)
    - [Specialized Computing](#specialized-computing)
        - [AI/ML Accelerators](#aiml-accelerators)
        - [FPGA Technologies](#fpga-technologies)
        - [DSP Technologies](#dsp-technologies)
    - [Testing and Validation](#testing-and-validation)
        - [Test Standards](#test-standards)


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

## Memory Technologies

### Volatile Memory
- **DDR5 SDRAM** - [Wikipedia](https://en.wikipedia.org/wiki/DDR5_SDRAM) | [JEDEC DDR5 Spec](https://www.jedec.org/standards-documents/docs/jesd79-5)
- **DDR4 SDRAM** - [Wikipedia](https://en.wikipedia.org/wiki/DDR4_SDRAM) | [JEDEC DDR4 Spec](https://www.jedec.org/standards-documents/docs/jesd79-4a)
- **DDR3 SDRAM** - [Wikipedia](https://en.wikipedia.org/wiki/DDR3_SDRAM)
- **LPDDR5** - [Wikipedia](https://en.wikipedia.org/wiki/LPDDR#LPDDR5) | [JEDEC LPDDR5 Spec](https://www.jedec.org/standards-documents/docs/jesd209-5)
- **GDDR6** - [Wikipedia](https://en.wikipedia.org/wiki/GDDR6_SDRAM) | [JEDEC GDDR6 Spec](https://www.jedec.org/standards-documents/docs/jesd250c)
- **GDDR6X** - [Wikipedia](https://en.wikipedia.org/wiki/GDDR6X_SDRAM)
- **HBM3 (High Bandwidth Memory)** - [Wikipedia](https://en.wikipedia.org/wiki/High_Bandwidth_Memory) | [JEDEC HBM3 Spec](https://www.jedec.org/standards-documents/docs/jesd238)
- **SRAM** - [Wikipedia](https://en.wikipedia.org/wiki/Static_random-access_memory)

### Non-Volatile Memory
- **3D NAND Flash** - [Wikipedia](https://en.wikipedia.org/wiki/3D_NAND_flash_memory)
- **QLC NAND** - [Wikipedia](https://en.wikipedia.org/wiki/Flash_memory#Multi-level_cell)
- **TLC NAND** - [Wikipedia](https://en.wikipedia.org/wiki/Flash_memory#Multi-level_cell)
- **SLC NAND** - [Wikipedia](https://en.wikipedia.org/wiki/Flash_memory#Single-level_cell)
- **3D XPoint (Optane)** - [Wikipedia](https://en.wikipedia.org/wiki/3D_XPoint)
- **ReRAM** - [Wikipedia](https://en.wikipedia.org/wiki/Resistive_random-access_memory)
- **MRAM** - [Wikipedia](https://en.wikipedia.org/wiki/Magnetoresistive_random-access_memory)
- **PCM (Phase-Change Memory)** - [Wikipedia](https://en.wikipedia.org/wiki/Phase-change_memory)
- **FeRAM** - [Wikipedia](https://en.wikipedia.org/wiki/Ferroelectric_RAM)

## Storage Technologies

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

## Cooling Technologies

### Thermal Management
- **Heat Pipes** - [Wikipedia](https://en.wikipedia.org/wiki/Heat_pipe)
- **Vapor Chambers** - [Wikipedia](https://en.wikipedia.org/wiki/Vapor_chamber)
- **Thermal Interface Materials (TIM)** - [Wikipedia](https://en.wikipedia.org/wiki/Thermal_interface_material)
- **Liquid Cooling** - [Wikipedia](https://en.wikipedia.org/wiki/Computer_cooling#Liquid_cooling)
- **Phase Change Cooling** - [Wikipedia](https://en.wikipedia.org/wiki/Immersion_cooling)

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

## Specialized Computing

### AI/ML Accelerators
- **Google TPU** - [Wikipedia](https://en.wikipedia.org/wiki/Tensor_Processing_Unit) | [Google Cloud TPU](https://cloud.google.com/tpu)
- **Intel Habana** - [Wikipedia](https://en.wikipedia.org/wiki/Habana_Labs) | [Intel Habana](https://habana.ai/)
- **Graphcore IPU** - [Wikipedia](https://en.wikipedia.org/wiki/Graphcore) | [Graphcore IPU](https://www.graphcore.ai/products/ipu)
- **Cerebras WSE** - [Wikipedia](https://en.wikipedia.org/wiki/Cerebras) | [Cerebras Systems](https://www.cerebras.net/)

### FPGA Technologies
- **Xilinx (AMD) FPGAs** - [Wikipedia](https://en.wikipedia.org/wiki/Xilinx) | [AMD Xilinx](https://www.xilinx.com/products/silicon-devices/fpga.html)
- **Intel (Altera) FPGAs** - [Wikipedia](https://en.wikipedia.org/wiki/Altera) | [Intel FPGAs](https://www.intel.com/content/www/us/en/products/details/fpga.html)
- **Lattice FPGAs** - [Wikipedia](https://en.wikipedia.org/wiki/Lattice_Semiconductor) | [Lattice FPGAs](https://www.latticesemi.com/Products/FPGAandCPLD)

### DSP Technologies
- **Texas Instruments DSPs** - [Wikipedia](https://en.wikipedia.org/wiki/Digital_signal_processor) | [TI DSPs](https://www.ti.com/processors/digital-signal-processors/overview.html)
- **Analog Devices DSPs** - [Wikipedia](https://en.wikipedia.org/wiki/Analog_Devices) | [ADI DSPs](https://www.analog.com/en/products/processors-dsp.html)

## Testing and Validation

### Test Standards
- **JEDEC Standards** - [Wikipedia](https://en.wikipedia.org/wiki/JEDEC) | [JEDEC Standards](https://www.jedec.org/standards-documents)
- **PCI-SIG Compliance** - [Wikipedia](https://en.wikipedia.org/wiki/PCI-SIG) | [PCI-SIG](https://pcisig.com/)
- **USB-IF Compliance** - [Wikipedia](https://en.wikipedia.org/wiki/USB_Implementers_Forum) | [USB-IF](https://www.usb.org/)

