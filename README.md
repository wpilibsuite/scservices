# SC Services

This repository holds open source services for SystemCore. These are ingested into the SystemCore OS build.

Currently, allwpilib is a submodule for all builds. However eventually we plan on having the ingested build use the systemcore allwpilib, once we change the name.

## Services

### PowerDistribution

This service reads the Power Distribution can data, and sends that data to the DS service. That service then sends the data to the DS allowing integrated logging.

### Radio

This service reads the status from the Vivid Hosting radio, and sends that to the system service.
