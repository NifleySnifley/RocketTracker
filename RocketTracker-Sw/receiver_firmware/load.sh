# picotool reboot -u -F
python3 -c "__import__('serial').Serial('/dev/ttyACM0', 110).close()"
sleep 5
picotool load $1
picotool reboot
