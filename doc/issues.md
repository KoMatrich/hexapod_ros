# Ubuntu server on Raspberry PI with Controler

Useful links:
* <https://forums.raspberrypi.com/viewtopic.php?t=304000>
* <https://forums.raspberrypi.com/viewtopic.php?t=207025>
* <https://forums.raspberrypi.com/viewtopic.php?t=242281>

## [Not detecting LE bluetooth devices](https://askubuntu.com/a/1336059/1249998)

### Solution:

```bash
sudo btmgmt le on
```

## [Keeps disconnecting/wont connect](https://askubuntu.com/questions/1250989/unable-to-connect-to-bluetooth-devices-org-bluez-error-connectionattemptfailed)

### Solution:

```rust
sudo bluetoothctl
[bluetooth]# power on
[bluetooth]# agent on
[bluetooth]# default-agent
[bluetooth]# scan on
[NEW] Device XX:XX:XX:XX:XX:XX
[bluetooth]# scan off
[bluetooth]# trust XX:XX:XX:XX:XX:XX
[bluetooth]# pair XX:XX:XX:XX:XX:XX
Attempting to pair with XX:XX:XX:XX:XX:XX
[CHG] Device XX:XX:XX:XX:XX:XX Connected: yes

[...]

[CHG] Device XX:XX:XX:XX:XX:XX Connected: yes
Connection successful
```

# Intel Realsense

## USB bus overflow

When plugged into USB 3.0 port on Raspberry PI. Official wrapper thinks that is plugged into USB 3.2. and after random duration wrapper crashes or won\`t start at all.

When using USB 2.0 port wrapper fallbacks to emergency mode and doesn\`t send all data that camera can supply (IMU).

### Solution:

Not found
