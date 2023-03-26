#!/bin/sh

export LD_LIBRARY_PATH=.:$LD_LIBRARY_PATH

# reset sensor
echo 119 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio119/direction
echo 0 > /sys/class/gpio/gpio119/value
sleep 0.2
echo 1 > /sys/class/gpio/gpio119/value
echo 119 > /sys/class/gpio/unexport

# enable sensor mclk
echo 1 > /sys/class/vps/mipi_host0/param/snrclk_en
echo 24000000 > /sys/class/vps/mipi_host0/param/snrclk_freq
echo 1 > /sys/class/vps/mipi_host1/param/snrclk_en
echo 24000000 > /sys/class/vps/mipi_host1/param/snrclk_freq

# echo 1 > /sys/class/vps/mipi_host0/param/stop_check_instart
# echo 1 > /sys/class/vps/mipi_host1/param/stop_check_instart

# i2cdetect -r -y 0
# i2cdetect -r -y 1
# i2cdetect -r -y 2

# Start ISP Control Tool
# Access via browser: http://192.168.1.10:8214/
act-server --http-port=8214 --http-root=/etc/control-tool/content --fw-acc=stream --fw-channel=chardev --fw-dev=/dev/ac_isp --hw-acc=stream --hw-buf-size=1024 --hw-channel=devmem --hw-dev=/dev/mem --hw-mem-offset=0xb3000000 --hw-mem-size=0x400000 &

sync
sleep 0.5

tuning_tool -C "./gc4663_player.json" -v "./sdb_gc4663_raw_10bit_2560x1440_online_Pipeline.json" -H "/etc/tuning_tool/dump_yuv.json"  -r 100000 -c 0 -p 1 -t 1 -l 4 -m 0 -f 0 -g 1 -D 0
