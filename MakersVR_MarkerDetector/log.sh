#!/bin/bash

echo "Service status:"
systemctl status marker-detector.service --no-pager --lines=0
echo "Log Output:"
sudo journalctl -u marker-detector.service -f -n 100 --identifier=MakersVR --output=cat
