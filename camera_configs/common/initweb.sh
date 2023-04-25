#!/bin/bash

set -e

act-server --http-port=8214 --http-root=/etc/control-tool/content \
	--fw-acc=stream --fw-channel=chardev --fw-dev=/dev/ac_isp \
	--hw-acc=stream --hw-buf-size=1024 --hw-channel=devmem \
	--hw-dev=/dev/mem --hw-mem-offset=0xb3000000 --hw-mem-size=0x400000 &

