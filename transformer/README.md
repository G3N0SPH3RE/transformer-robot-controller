# transformer-robot-controller

## System setup

Add user to group for /dev/ttyUSB0
```console
sudo usermod -a -G dialout <username>
```

Create a udev rule for USB RS485 symlink in /dev
```console
sudo touch /etc/udev/rules.d/usb-serial.rules
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="usb-RS485"' | sudo tee /etc/udev/rules.d/usb-serial.rules
```


