Use a mouse as encoder. Requires read permission to `/dev/input/mice`.

# Publish Topic
* `mouse_encoder/tick`: `geometry_msg/Point` message with number of ticks in the x and y direction.

# Parameters
* `~dev_path`: Default to `/dev/input/mice`. Point to the device path of the mouse.

# Getting access to `/dev/input/mice`.
* Create a group named `input` 
```
$ sudo groupadd input
```
* Add yourself to the `input` group
```
$ sudo adduser your_user_name input
```
* Log out and log back in for the change to take effect
* Put all devices under `/dev/input/` into the `input` group to grant the group read/write permission. Can be done by adding a file name `99-pure-data.rules` under `/etc/udev/rules.d` with the following line:
```
SUBSYSTEM=="input", GROUP="input", MODE="660"
```
* Reboot for the rule to take effect.