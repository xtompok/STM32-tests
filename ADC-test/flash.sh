echo "flash write_image erase $1 0x08000000" > ocd-hex.cfg
openocd -f /usr/share/openocd/scripts/interface/stlink-v2.cfg -f /usr/share/openocd/scripts/target/stm32f0x.cfg -f ocd-pre.cfg -f ocd-hex.cfg -f ocd-post.cfg

