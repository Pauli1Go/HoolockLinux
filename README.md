# HoolockLinux for iPad 7

This is an experimental HoolockLinux kernel fork focused on Linux support for
the seventh-generation iPad using the Apple A10 / T8010 SoC. The currently
tested target is the cellular `iPad7,12` (`J172`).

> **Touch documentation:** Read the
> [full iPad 7 touch bring-up guide](https://github.com/Pauli1Go/HoolockLinux-bringup-docs/blob/main/docs/touch.md).

## Current state

The system boots Debian from the internal NVMe storage and provides a graphical
desktop with touchscreen, battery status, Wi-Fi, and Bluetooth support.

The current kernel includes:

- T8010 H9P PCIe host bridge and Apple DART support;
- Apple ANS FlatDMA, NVMMU, and runtime NVMe HMB setup;
- read/write access and root filesystem boot from internal NVMe;
- T8010 SmartIO, SIO DMA, SPI, and touch-clock support;
- automatic J172 Apple Z2 touchscreen support;
- J172 battery, charging, and power-status support;
- BCM4355C1 Wi-Fi through the standard `brcmfmac` stack;
- Apple SysCfg/NVMEM support for Wi-Fi calibration;
- T8010 UART3 support; and
- J172 BCM4355C1 Bluetooth through the standard `hci_bcm` and BlueZ stacks.

The tested boot flow uses PongoOS and the patched
[m1n1](https://github.com/Pauli1Go/m1n1) loader. It supplies the
NVMe Host Memory Buffer, publishes the private SysCfg region for Linux NVMEM,
and provides the device-specific Bluetooth address through Device Tree.

## Touch

The J172 touchscreen initializes automatically and is registered as the
standard Linux input device `iPad7,12 Touchscreen`. Single-touch, swipes, and
simultaneous multi-touch work through evdev and libinput without a userspace
touch daemon, manual parameters, or driver rebind.

## Wi-Fi

The BCM4355C1 Wi-Fi controller is exposed through normal Linux networking
interfaces. Firmware, NVRAM, CLM, TxCap, MAC address, and device calibration
are loaded automatically. NetworkManager, `nmcli`, `wpa_supplicant`, and
`hostapd` work without driver-specific system tweaks.

Station mode, WPA2, DHCP, Internet access, disconnect/reconnect, and access
point mode have been validated.

## Bluetooth

Bluetooth uses the BCM4355C1 controller in the Wi-Fi combination chip through
T8010 UART3. The kernel controls the J172 power and wake GPIOs, initializes the
Broadcom transport, switches the controller and host UART to their runtime
rates, and automatically loads `brcm/BCM4355C1.hcd` with the standard Linux
firmware API.

The controller appears as a normal BlueZ HCI device. Discovery, pairing,
reconnect, and A2DP audio have been validated using standard Linux userspace.

**Bluetooth RF calibration is still open and is not implemented.** The private
SysCfg records `BTRx`, `BTTx`, and `BCAL` are not currently consumed by the
Linux driver or loader. Range, transmit power, receive sensitivity, and RF
performance must therefore not yet be considered fully calibrated.

## Firmware

Apple firmware and private device calibration are not included in this
repository. The complete reproducible extraction, verification, installation,
SysCfg, and initramfs instructions are maintained in
[HoolockLinux-linux-firmware](https://github.com/Pauli1Go/HoolockLinux-linux-firmware).

The standard firmware paths are used. No firmware-path override, userspace
firmware daemon, module parameter, or driver rebind is required.

## NVMe

The tested iPad boots Debian from `/dev/nvme0n1p2`. The ext4 root filesystem
mounts read/write, and controlled writes, parallel writes, discard, and trim
have been validated without NVMe, DART, or ext4 errors.

Runtime NVMe controller reset remains intentionally disabled because resetting
the live H9P/ANS controller can require a hardware restart. Normal boot and
runtime I/O are unaffected.

Do not overwrite the APFS partition. The tested layout retains APFS on
partition 1 and uses partition 2 for Linux.

## Boot requirements

A complete tested boot requires:

1. a checkm8/palera1n-compatible boot transport;
2. PongoOS;
3. the patched [m1n1](https://github.com/Pauli1Go/m1n1) loader;
4. the HoolockLinux kernel and J172 Device Tree;
5. an initramfs containing the required touch, Wi-Fi, and Bluetooth firmware;
6. the private SysCfg trailer from the same iPad; and
7. a prepared Linux root filesystem on internal NVMe storage.

This remains an experimental bring-up project. Back up important data and
understand the partitioning and boot process before modifying a device.
