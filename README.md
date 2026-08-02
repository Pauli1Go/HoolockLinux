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
| Battery and charging status | Working | Working |
| Wi-Fi | Working, including station and AP modes | Working in station mode; AP mode not working |
| Bluetooth | Working; RF calibration remains open | Not yet implemented |
| Integrated Speakers | Not yet implemented | Not yet implemented |
| Auto Brightness | Not yet implemented | Not yet implemented |
| Auto Rotation | Not yet implemented | Not yet implemented |
| Fake Home Button | - | Not yet implemented |
| GPU | Not yet implemented | Not yet implemented |

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
graphical desktop. Internal storage, the D111 touchscreen, battery telemetry,
charging, and Wi-Fi station mode are operational. Bluetooth, audio, cameras,
sensors, and cellular hardware are not part of the currently validated iPhone
support.

The iPhone 7 Plus support includes:

- the common T8010 PCIe, DART, ANS, NVMMU, HMB, and NVMe storage stack;
- read/write access and root filesystem boot from internal NVMe;
- the T8010 SPI2 controller and board-specific SIO DMA routing;
- 8-bit Chestnut PMIC register access;
- regulator-managed Adelyn core and Chestnut high-voltage touch supplies;
- D111 Apple Z2 firmware and runtime protocol support;
- transfer of four device-specific touch calibrations by m1n1;
- cached BQ27545 battery telemetry over the muxed UART/HDQ path;
- D2333 PMIC GPIO and board-specific BCM4355C1 power sequencing;
- BCM4355C1 Wi-Fi through the standard `brcmfmac` PCIe stack;
- runtime Apple SysCfg/NVMEM delivery of the device MAC address and Wi-Fi
  calibration; and
- automatic SN2400 charging with input-current ramping and VBUS foldback.

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

### Battery and charging

The D111 BQ27545 fuel gauge is exposed through the standard Linux
`power_supply` interface as `battery`. Present state, charge status, voltage,
current, capacity, temperature, charge counters, full-charge capacity, and
cycle count are available without issuing a new HDQ transaction for every
userspace read. A periodic cache worker performs the serialized HDQ telemetry
updates through the SN2400 mux.

The SN2400 charger is exposed as the standard `usb` power supply and binds
automatically during boot. It is built into the D111 kernel configuration, so
no module copy, `insmod`, userspace daemon, or manual driver rebind is needed.

Input current starts conservatively at 300 mA and increases in 25 mA steps.
The driver monitors VBUS and folds the learned limit back when the source or
cable droops. It treats VBUS below 4.0 V as unsafe, retains the safe session
limit across short cable interruptions, and starts a new current-learning
session after a sustained detach.

Hardware validation on the tested iPhone 7 Plus showed the battery charging
at approximately 0.39 A with VBUS around 4.88--4.98 V. Automatic binding,
cached battery-status updates, and controlled charger unbind/rebind were also
validated. The actual charge rate remains dependent on the connected cable and
power source.

### Wi-Fi

The D111 BCM4355C1 controller is exposed as a normal Linux Wi-Fi interface
through PCIe port 2 and the standard `brcmfmac` stack. The D2333 PMIC GPIO
controller, WLAN regulator, PCI power control, and T8010 DART2 provide the
board-specific power and DMA paths without userspace sequencing or a manual
driver rebind.

Firmware, CLM, TxCap, MAC address, and the private per-device WCAL calibration
are loaded automatically through the standard firmware and Apple SysCfg/NVMEM
paths. NetworkManager, `nmcli`, and `wpa_supplicant` work without
driver-specific userspace changes.

Station mode, passive scanning, WPA2, DHCP, local network access, and sustained
data transfer have been validated on the tested iPhone 7 Plus. Access point
mode is not working: the AP can become visible, but clients cannot complete a
connection. AP mode must therefore not be considered supported.

### Remaining hardware

The iPad 7 Bluetooth integration must not be assumed to apply directly to the
iPhone. Bluetooth, audio, cameras, sensors, cellular hardware, and other
remaining subsystems still require their own iPhone-specific board topology,
GPIO, firmware, calibration, power-sequencing, Device Tree, and hardware
validation work.

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
- D111 touch and Wi-Fi firmware in the initramfs;
- all four touch calibrations from the same iPhone, supplied through m1n1;
- the private SysCfg data from the same iPhone for Wi-Fi identity and
  calibration; and
- a prepared Linux root filesystem on the iPhone's Linux partition.

This remains an experimental bring-up project. Back up important data and
understand the partitioning, recovery, and boot process before modifying a
device.
