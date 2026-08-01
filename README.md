# HoolockLinux for iPad 7 and iPhone 7

## Support matrix

| Feature | iPad 7 | iPhone 7 |
|---|---|---|
| Tested device | Cellular iPad 7 (`iPad7,12` / J172) | iPhone 7 Plus (`iPhone9,4` / D111) |
| Apple SoC | A10 / T8010 | A10 / T8010 |
| Linux boot through PongoOS and m1n1 | Working | Working |
| Internal NVMe storage | Read/write; root filesystem boot | Read/write; root filesystem boot |
| Debian graphical desktop | Working | Working |
| Touchscreen | Working, including multi-touch | Working, including multi-touch |
| Battery and charging status | Working | Not yet validated |
| Wi-Fi | Working, including station and AP modes | Not yet implemented |
| Bluetooth | Working; RF calibration remains open | Not yet implemented |

“Working” means that the feature has been validated on the device named in
the table. Other variants may require additional Device Tree, calibration,
power-sequencing, or driver work. In particular, the standard-size iPhone 7
has not yet received the same hardware validation as the tested iPhone 7 Plus.

This is an experimental HoolockLinux kernel fork focused on Linux support for
Apple A10 / T8010 devices. The currently tested targets are the cellular
seventh-generation iPad (`iPad7,12` / J172) and the iPhone 7 Plus
(`iPhone9,4` / D111).

> **Touch documentation:** Read the
> [full touch bring-up guide](https://github.com/Pauli1Go/HoolockLinux-bringup-docs/blob/main/docs/touch.md).

## iPad 7

### Current state

The cellular iPad 7 boots Debian from internal NVMe storage and provides a
graphical desktop with touchscreen, battery status, Wi-Fi, and Bluetooth
support.

The iPad 7 kernel support includes:

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

### Touch

The J172 touchscreen initializes automatically and is registered as the
standard Linux input device `iPad7,12 Touchscreen`. Single-touch, swipes, and
simultaneous multi-touch work through evdev and libinput without a userspace
touch daemon, manual parameters, or driver rebind.

### Wi-Fi

The BCM4355C1 Wi-Fi controller is exposed through normal Linux networking
interfaces. Firmware, NVRAM, CLM, TxCap, MAC address, and device calibration
are loaded automatically. NetworkManager, `nmcli`, `wpa_supplicant`, and
`hostapd` work without driver-specific system tweaks.

Station mode, WPA2, DHCP, Internet access, disconnect/reconnect, and access
point mode have been validated.

### Bluetooth

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

## iPhone 7

### Current state

The tested iPhone 7 Plus boots Debian from internal NVMe storage and reaches a
graphical desktop. Internal storage and the D111 touchscreen are operational.
Battery, Wi-Fi, Bluetooth, audio, cameras, sensors, and cellular hardware are
not part of the currently validated iPhone support.

The iPhone 7 Plus support includes:

- the common T8010 PCIe, DART, ANS, NVMMU, HMB, and NVMe storage stack;
- read/write access and root filesystem boot from internal NVMe;
- the T8010 SPI2 controller and board-specific SIO DMA routing;
- 8-bit Chestnut PMIC register access;
- regulator-managed Adelyn core and Chestnut high-voltage touch supplies;
- D111 Apple Z2 firmware and runtime protocol support; and
- transfer of four device-specific touch calibrations by m1n1.

### Touch

The D111 touchscreen initializes automatically and is registered as
`iPhone9,4 Touchscreen`. Single-touch, swipes, and simultaneous multi-touch
work through the normal Linux evdev and libinput paths.

m1n1 copies the device-specific multitouch, orb-gap, orb-force, and dynamic
shape-acceleration calibration payloads from the Apple Device Tree. Missing,
invalid, oversized, or unresolved placeholder calibration disables only the
touchscreen node; Linux continues booting without touch.

The touchscreen power sequence uses normal regulator consumers for the
Adelyn core and Chestnut high-voltage rails. The kernel does not program PMIC
registers directly from the touchscreen driver or Device Tree.

### Remaining hardware

The iPad 7 Wi-Fi, Bluetooth, and battery integrations must not be assumed to
apply directly to the iPhone. The iPhone board topology, GPIOs, firmware,
calibration, power sequencing, and Device Tree descriptions still need to be
implemented and validated separately for each subsystem.

## Firmware and calibration

Apple firmware and private per-device calibration are not included in this
repository. The complete reproducible extraction, verification, installation,
SysCfg, and initramfs instructions are maintained in
[HoolockLinux-linux-firmware](https://github.com/Pauli1Go/HoolockLinux-linux-firmware).

The standard Linux firmware paths are used. No firmware-path override,
userspace firmware daemon, module parameter, or driver rebind is required for
the validated configurations. Firmware and calibration must always come from
the same physical device being booted.

## Internal NVMe storage

Both tested devices boot Debian from `/dev/nvme0n1p2`. The ext4 root
filesystem mounts read/write, and controlled writes have been validated
without NVMe, DART, or ext4 errors. The iPad 7 has additionally completed
parallel-write, discard, and trim testing.

Runtime NVMe controller reset remains intentionally disabled because resetting
the live H9P/ANS controller can require a hardware restart. Normal boot and
runtime I/O are unaffected.

Do not overwrite the APFS partition. The tested layout retains APFS on
partition 1 and uses partition 2 for Linux. Verify the exact device, capacity,
GPT, APFS geometry, backup path, and recovery boot before changing a device's
partition layout.

## Boot flow

Both devices use the same high-level boot flow:

1. enter DFU mode using a checkm8-compatible device;
2. boot PongoOS with palera1n;
3. load the patched [m1n1](https://github.com/Pauli1Go/m1n1) loader;
4. let m1n1 publish runtime hardware data and calibrations into Device Tree;
5. boot the HoolockLinux kernel, the matching board DTB, and initramfs; and
6. mount the prepared Linux root filesystem from internal NVMe.

### iPad 7 requirements

- the J172 Device Tree;
- touch, Wi-Fi, and Bluetooth firmware in the initramfs;
- the private SysCfg data from the same iPad; and
- a prepared Linux root filesystem on the iPad's Linux partition.

### iPhone 7 Plus requirements

- the D111 Device Tree;
- D111 touch firmware in the initramfs;
- all four touch calibrations from the same iPhone, supplied through m1n1; and
- a prepared Linux root filesystem on the iPhone's Linux partition.

This remains an experimental bring-up project. Back up important data and
understand the partitioning, recovery, and boot process before modifying a
device.
