.. _ADTF-NVM:

ADTF3175 NVM Contents
=====================

.. toctree::
   :maxdepth: 2

+-----------------+---------------------------+-----------------------------------------------+
| Block Type ID   | Component                 | Description                                   |
+=================+===========================+===============================================+
| 0x3D            | NVM Header                | Must always start at address 0x00             |
+-----------------+---------------------------+-----------------------------------------------+
| 0x01            | CCB                       | Image calibration content                     |
+-----------------+---------------------------+-----------------------------------------------+
| 0x56            | Imager Factory Firmware   | Reserved space for imager factory firmware    |
+-----------------+---------------------------+-----------------------------------------------+
| 0x57            | Imager Current Firmware   | Reserved space for Imager Current Firmware    |
+-----------------+---------------------------+-----------------------------------------------+
| 0x58            | Imager Upgrade Firmware   | Reserved space for Imager Upgrade Firmware    |
+-----------------+---------------------------+-----------------------------------------------+
| 0x53            | ADSD3500 Factory Firmware | Reserved space for ADSD3500 Factory Firmware  |
+-----------------+---------------------------+-----------------------------------------------+
| 0x54            | ADSD3500 Current Firmware | Reserved space for ADSD3500 Current Firmware  |
+-----------------+---------------------------+-----------------------------------------------+
| 0x55            | ADSD3500 Upgrade Firmware | Reserved space for ADSD3500 Upgrade Firmware  |
+-----------------+---------------------------+-----------------------------------------------+
| 0x52            | ADSD3500 Init Firmware    | Reserved space for ADSD3500 Init Firmware     |
+-----------------+---------------------------+-----------------------------------------------+
