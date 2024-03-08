ADI provide 5 executables (built for ARM64) to interract with imager Flash memory through ADSD3500
These 5 tools can be used in both host-boot usecase and self-boot.
For host-boot please check host_boot.sh

	1. NVM_WRITE - is used to flash a new image into the imager SPI flash memory.
!!ATTENTION!! This tool will completly overwrite the content of the memory. !!ATTENTION!!
	It takes only one parameter, a binary file to be written into the flash memory
		ex: ./NVM_FLASH_UPDATE flash.bin


	2. CCB_READ - is used to read back calibration CCB from the imager flash memory.
!!ATTENTION!! For modules with non ADSD3500 Flash memory layout, the CRC mismatch is expected. !!ATTENTION!!
	It takes only one parameter, a file name where the calibration read from the imager module should be saved
		ex: ./CCB_READ calibration.ccb
   
        3. NVM_READ - is used to read back complete flash image from the imager flash memory.
!!ATTENTION!! For modules with non ADSD3500 Flash memory layout, the CRC mismatch is expected. !!ATTENTION!!
        It takes only one parameter, a file name where the complete flash.bin from the imager module should be saved
                ex: ./NVM_READ backup_flash.bin

	4. CFG_READ - is used to read back imager firmware(CFG) from the imager flash memory.
!!ATTENTION!! For modules with non ADSD3500 Flash memory layout, the CRC mismatch is expected. !!ATTENTION!!
	It takes only one parameter, a file name where the calibration read from the imager module should be saved
		ex: ./CFG_READ imager_firmware.cfg

	5. INIT_FW_WRITE - is used to write init firmware to the imager flash memory.
!!ATTENTION!! For modules with non ADSD3500 Flash memory layout, the CRC mismatch is expected. !!ATTENTION!!
        It takes only one parameter, path to the file which need to be written to the imager flash memory.
                ex: ./INIT_FW_WRITE init_fw.stream



If module is not self-booting because Flash memory content is corrupted or does not contain ADSD3500 firmware, NVM_Utils tool can be used to fix it.
