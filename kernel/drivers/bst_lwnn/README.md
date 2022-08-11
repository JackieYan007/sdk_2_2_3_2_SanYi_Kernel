# Linux LWNN Driver Module Design Details
Author: Shichen Lin  
Date: 7/21/2021

## Terminology
| Terminology | Definition |
|-------------|------------|
| CMA         | Contiguous Memory Allocator |
| DMA         | Direct Memory Access |
| DSP         | Digital Signal Processor |
| DTS         | Device tree configurations |
| IPC         | Inter-Processor Communication |
| ISP         | Image Signal Processor |
| VSP         | Video Signal Processor |

## Content
- [1. Introduction](#1-introduction)
- [2. Design Structure](#2-design-structure)
   - [2.1 Sysfile](#21-sysfile)
   - [2.2 Misc Device](#22-misc-device)
   - [2.3 Memory Manager](#22-memory_manager)
   - [2.4 Firmware Manager](#23-firmware-manager)
   - [2.5 Message Manager](#24-message-manager)
- [3. Known Issues](#3-known-issues)
- [4. Future Work Areas](#4-future-work-areas)

## 1. Introduction
The entire LWNN driver consists of three parts, the library module, the driver module and the firmware module. So far, CV DSP2 and CV DSP3 are planned to be controlled by the LWNN driver. This document includes most design details of the driver module in Linux, including design reasoning, concerns, known issues if existent and areas for future work.

## 2. Design Structure
The Linux LWNN driver module mainly consists of five components, memory manager, firmware manager, message manager, misc device and sysfile. It also takes DTS configurations. The contained DTS properties are like the following:
- reg - The first resource is the control register block of the CV module. The latter resources are the firmware memory regions of CV DSPs.
- memory-region - The reserved memory of the LWNN driver.
- ipc-register-addr - The IPC buffer array address used to initialize the IPC module of DSP firmware.
- bus-offset - The offset which is computed by a host-side physical address minus the corresponding DSP-side address.
- mbox-names - The property used by the IPC module.
- mboxes - The property used by the IPC module.
- dsp-num - The number of available DSP(s).
- dsp - The DSP instance property
   - index - The index of the DSP in the CV subsystem.
   - assigned-mem-size - The assigned memory size to DSP firmware from the reserved memory.
   - firmware - The firmware file name.
   - rt-init-addr - The physical address of initialization data shared with the DSP.
   - ipc-src-core - The ARM core ID that the DSP should send IPC messages to.

The boot sequence of the current kernel driver is like the following:
1. Create sysfile(s) for driver runtime configuration.
2. Initialize the memory manager as both firmware manager and message manager depend on it.
3. Initialize the firmware manager, including allocating assigned memory to each DSP.
4. Initialize the message manager, including creating related worker threads.
5. Create a misc device. This is the final step since users can start to use the kernel driver after this step. Therefore, it should be done after everthing is ready.

The current driver module design covers some multi-DSP extendability, but it is far from mature completion. Even the DTS configurations have not supported this yet. The firmware manager and the message manager treats each DSP independently. The failure of a DSP in these two submodules do not take down the entire driver module except that the failed DSP is the last available DSP.

### 2.1 Sysfile
Sysfile(s) are created for driver runtime configurations. So far, this only include the setup of driver print level. The print level sysfile is `/sys/kernel/bst_lwnn/bst_lwnn_print_level`. Its valid written values should be:
- 0 - BST_LWNN_NO_PRINT
- 1 - BST_LWNN_LOG_PRINT
- 2 - BST_LWNN_DEBUG_PRINT

### 2.2 Misc Device
As the kernel driver is responsible for DMA memory allocation as well mapping while sysfiles do not support `mmap` callback, misc device is chosen to be the userland interface in the original design. The created misc device is registered as a driver file, `"/dev/bst_cv<id>"` where `id` is the property in the DTS configurations, in the Linux filesystem. The supposed file operations on the driver file include `open`, `close` and `ioctl`.

The supported `ioctl` commands include the following:
- XRP_IOCTL_ALLOC - Allocate a DMA buffer
- XRP_IOCTL_FREE - Free a DMA buffer
- XRP_IOCTL_QUEUE - Send a command to DSP
- XRP_IOCTL_WAIT - Wait for the response from DSP of a previously sent command
- XRP_IOCTL_SYNC - Flush the caches of a DMA buffer back to DDR

### 2.3. Memory Manager
The memory manager is mainly responsible for reserved memory allocation. Each allocated memory buffer by the memory manager is referred as a memory block. The actual memory allocation is done through CMA which corresponds to `reusable` property in the driver reserved memory. Except for internal DMA buffer usage, the DMA buffers allocated for userspace are tracked by the memory manager. A memory context is used to track all allocated DMA buffers via ioctl of the same file descriptor(`struct file`) with a hash table whose keys are physical addresses of tracked DMA buffers and values are memory block pointers. Once the file descriptor is closed, all memory blocks attached to it will be released for garbage collection. Memory contexts are kept as a linked list by the memory manager.

The allocated user DMA buffers are mapped as cacheable memory for faster copy speed. Therefore, apart from allocation and deallocation, synchornization mechanism is also implemented to guarantee memory coherence.

### 2.4. Firmware Manager
The firmware manager takes over hardware bootup. This includes configuring control registers and booting the firmware. It should be noticed that the actual bootup is not done during the initialization stage of the firmware manager because the firmware is saved as a file and the filesystem is not online at this stage. Therefore, this steps is delayed to the first `open` of the misc device file.

During initialization, the firmware manager basically reads the configurations from DTS and does relative preparation, including mapping registers and firmware memory into kernel space as well as allocating the DMA memory assigned to the DSP. This is also the reason why the firmware manager is initialized after the memory manager.

As the SYS CRM module is already set up at bootup stage, only control registers in the CV module need to be configured. For explicit control register information, please refer to `"BST A1000 Full Chip Level Address and Register Description.doc"`.

To boot up the DSP firmware, the firmware manager needs to first load the firmware into DDR and then release the DSP reset. After the firmware starts running, the firmware manager will do a handshake process to fully initialize the DSP firmware, including passing DSP configurations. The hanshake is done through shared memory and its memory address is hard coded and also unchangeable in the DSP firmware. For handshake details, please check the data structure, `struct bst_lwnn_rt_init`, in source codes. Once the handshake finishes, the booted DSP can the be viewed as available or "online" in the source codes.

For DSP firmware, its assigned memory will be used as messaging response buffers and debugging trace print buffer. For now, there exists an unreliable mechanism to print some debugging information from DSP at ARM side. However, because there is no synchornization protection, consecutive prints at DSP side can make the print buffer be overwritten.

### 2.5. Message Manager
The message manager handles messaging with DSP(s). It should be noticed that the driver module is only responsible for sending and receiving messages. The message content of normal execution commands is handled by the library module. This means a command sent to the DSP is actually initiated from the libary module and its result is finally returned back to the library module as well while the driver module solely works as an intermediate layer here.

Inside the driver module, messaging is completely done via the IPC module. However, the command data is saved in extra DMA buffers, so an IPC message itself contains not the command data directly but the pointer to the command data buffer instead. Multiple levels of indirection can be used here in passing multiple data chunks to DSP. For simplicity, sent messages and response content from DSP are respectively referred as requests and responses in below.

In the implementation of the message manager, each DSP has its own messaging worker thread to send requests and wait for responses in a sequential order. Only after a DSP returns the reponse to its last request, a new request can be dequeued from the message queue of that DSP and then sent to the DSP. However, if a DSP does not response within the preset timeout(BST_LWNN_RSP_TIMEOUT_MS which is 1s), then it will be recognized as failure and should be brought down in the driver module.

It is assumed that there is no dependency among requests. Therefore, for a request without a specified target DSP, it will be scheduled based on the fewest-request-first policy. This means it will be assgined to the DSP with the fewest requests.

## 3. Known Issues
- Since current DMA memory mapping to userspace is done through the misc device file, this leaves the `mmap` functionality to userspace as well and this could be a serious security concern.

## 4. Future Work Areas
- The DMA buffer passing to userspace can be achieved with `dma-buf` framework to resolve the `mmap` security concern mentioned in [Section 3](#3-known-issues).
- More scheduling policies can be supported by the driver module when scheduling requests. Also, sysfile configuration can be added to choose the desired scheduling policy when the target DSP of a request is not specified.
- To better support current ISP and VSP drivers, prototypes of APIs for `dma-buf` importing have been added but not tested yet.
- The driver module was initially built as a kernel module to facilitate testing, so it has exit functions in implementations. However, when it becomes a built-in driver, which in normal cases would only be offline at the time of shutting down the system, these exit functions are no longer used. Therefore, they can be removed if needed. Also, because of this, synchronization protection might not really covered in these exit functions as it was managed not to use the driver when removing the kernel module.
