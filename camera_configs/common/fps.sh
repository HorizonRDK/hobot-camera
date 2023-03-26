
atime=5
t1=`cat /sys/devices/platform/soc/b3000000.isp/isp_status | awk -v FS=':' '{if($1=="fs_irq_cnt")print $2}'`
sleep $atime
t2=`cat /sys/devices/platform/soc/b3000000.isp/isp_status | awk -v FS=':' '{if($1=="fs_irq_cnt")print $2}'`
echo ${t1}
echo ${t2}

fps=`expr \( $t2 - $t1 \) / $atime`
echo fps: $fps
