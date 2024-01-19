# Navio2

## PWM に root 権限なしでアクセスする

1. ルールの追加

`/etc/udev/rules.d/10-local.rules`に以下を追記 \
cf. [Need to configure non-root PWM access](https://community.emlid.com/t/need-to-configure-non-root-pwm-access/16501/10)

```txt
SUBSYSTEM=="pwm*", PROGRAM="/bin/sh -c '\
        chown -R root:gpio /sys/class/pwm && chmod -R 770 /sys/class/pwm;\
        chown -R root:gpio /sys/devices/platform/soc/*.spi/spi_master/spi1/spi1.0/pwm/pwmchip0 && chmod -R 770 /sys/devices/platform/soc/*.spi/spi_master/spi1/spi1.0/pwm/pwmchip0\
'"
```

2. udev ルールの適用

```bash
$ sudo udevadm control --reload-rules
$ sudo udevadm trigger
```
