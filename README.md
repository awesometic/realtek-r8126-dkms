# Realtek r8126 DKMS

![GitHub release (latest SemVer)](https://img.shields.io/github/v/release/awesometic/realtek-r8126-dkms?sort=semver&style=for-the-badge)

This provides Realtek r8126 driver in DKMS way so that you can keep the latest driver even after the kernel upgrade.

## Before use

This DKMS package is for Realtek RTL8126 (r8126 in module name) Ethernet, which is designed for the PCI interface.

If you are searching for the Realtek 2.5 Gbits **PCI or USB Ethernet**, which may use RTL8125 and RTL8152 respectively, please refer to another DKMS project for that Realtek driver.

- 2.5 GbE **PCI interface**: [Realtek r8125 DKMS](https://github.com/awesometic/realtek-r8125-dkms)
- 10/100/1000 MbE or 2.5/5 GbE **USB interface**: [Realtek r8152 DKMS](https://github.com/awesometic/realtek-r8152-dkms)

## Installation

There are 3 ways to install this DKMS module. Choose one as your tastes.

Those are not interfering with each other. So you can do all 3 methods but absolutely you don't need to.

Installation using the Debian package is recommended for the sake of getting the newer driver.

### Debian package

#### Released package file

Download the latest Debian package from the Release tab on the Github repository.

Then enter the following command.

```bash
sudo dpkg -i realtek-r8126-dkms*.deb
```

> If multiple files selected by the wild card, you should type the specific version of the file.
>
> ```bash
> sudo dpkg -i realtek-r8126-dkms_10.015.00-1_amd64.deb
> ```

If dependency error occurs, try to fix that with `apt` command.

```bash
sudo apt install --fix-broken
```

#### Launchpad PPA (Recommended)

Add the Launchpad PPA.

```bash
sudo add-apt-repository ppa:awesometic/ppa
```

Then install the package using `apt` tool.

```bash
sudo apt install realtek-r8126-dkms
```

### autorun.sh

Using the `autorun.sh` script that Realtek provides on their original driver package. This is **not installed as a DKMS**, only efforts to the current kernel.

Download or clone this repository and move to the extracted directory, then run the script.

```bash
sudo ./autorun.sh
```

### dkms-install.sh

This script is from aircrack-ng team. You can install the DKMS module by a simple command.

Download or clone this repository and move to the extracted directory, then run the script.

```bash
sudo ./dkms-install.sh
```

## Verify the module is loaded successfully

After installing the DKMS package, you may not be able to use the new `r8126` module on the fly. This because the existing `r8169` module will be loaded priority to `r8126` so that it prevents working of the `r8126` module.

Check if the `r8169` module loaded currently.

```bash
lsmod | grep -i r8169
```

If there is a result, maybe the `r8126` module wasn't loaded properly. You can check out modules currently in use via `lspci -k` or `dmesg` too.

To use the `r8126` module explicitly you can add the `r8169` module to not be loaded by adding it to a blacklist file.

Enter the following command to configure the blacklist.

```bash
sudo tee -a /etc/modprobe.d/blacklist-r8169.conf > /dev/null <<EOT
# To use r8126 driver explicitly
blacklist r8169
EOT
```

To apply the new blacklist to your kernel, update your initramfs via

```bash
sudo update-initramfs -u
```

Finally, reboot to take effect.

> - If you need to load both r8169 and r8126, maybe removing r8126 firmware could make it work. Please enter `sudo rm -f /lib/firmware/rtl_nic/rtl8126*` to remove all the r8126 firmwares on the system. But it is just a workaround, you should have to do this every time installing the new kernel version or new Linux firmware.
> - In the case of the Debian package, I will update the scripts to make it do this during the installation.

## Debian package build

You can build yourself this after installing some dependencies including `dkms`.

```bash
sudo apt install devscripts debmake debhelper build-essential dkms dh-dkms
```

```bash
dpkg-buildpackage -b -rfakeroot -us -uc
```

## LICENSE

GPL-2 on Realtek driver and the debian packaing.

## References

- [Realtek r8126 driver release page](https://www.realtek.com/Download/List?cate_id=584)
