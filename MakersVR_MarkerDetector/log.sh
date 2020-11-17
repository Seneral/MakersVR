#!/bin/bash

sudo journalctl -u marker-detector.service -f -n 100 --identifier=MakersVR --output=cat
