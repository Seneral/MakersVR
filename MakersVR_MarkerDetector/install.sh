#!/bin/bash

# Install service on next restart
cp marker-detector.service /etc/systemd/system/
sudo systemctl enable marker-detector.service

# Install logging in all shells
LOG_CMD="sudo journalctl -u marker-detector.service -f -n 1000 --identifier=MakersVR --output=cat"
sudo grep -qxF "${LOG_CMD}" /etc/profile || echo $LOG_CMD >> /etc/profile
