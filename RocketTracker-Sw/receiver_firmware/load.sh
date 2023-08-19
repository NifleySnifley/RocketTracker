# picotool reboot -u -F
python3 -c "__import__('serial').Serial('/dev/ttyACM1', 1200).close()"
sleep 2
picotool load $1
picotool reboot
