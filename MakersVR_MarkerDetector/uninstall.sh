#!/bin/bash

sudo systemctl disable marker-detector.service

# Install logging in all shells
LOG_CMD="sudo journalctl -u marker-detector.service -f -n 1000 --identifier=MakersVR --output=cat"
sudo sed -i "/${LOG_CMD}/d" /etc/profile
