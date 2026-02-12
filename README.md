# all-in-one-sensor

本科毕业设计

## 0. environment

- Ubuntu 22.04
- ROS2 humble

## 1. install

```bash
git clone https://github.com/LemperorD/all-in-one-sensor.git
```

```bash
git submodule update --init --recursive
```

[MVS下载中心](https://www.hikrobotics.com/cn/machinevision/service/download/?module=0)

```bash
sudo rm -rf /opt/MVS/lib/64/libusb-1.0.so.0
```

## 2. build

```bash
~/all-in-one-sensor/scripts/colconBuild.sh
```
