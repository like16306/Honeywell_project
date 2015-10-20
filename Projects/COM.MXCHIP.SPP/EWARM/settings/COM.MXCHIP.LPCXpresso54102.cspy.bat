@REM This batch file has been generated by the IAR Embedded Workbench
@REM C-SPY Debugger, as an aid to preparing a command line for running
@REM the cspybat command line utility using the appropriate settings.
@REM
@REM Note that this file is generated every time a new debug session
@REM is initialized, so you may want to move or rename the file before
@REM making changes.
@REM
@REM You can launch cspybat by typing the name of this batch file followed
@REM by the name of the debug file (usually an ELF/DWARF or UBROF file).
@REM
@REM Read about available command line parameters in the C-SPY Debugging
@REM Guide. Hints about additional command line parameters that may be
@REM useful in specific cases:
@REM   --download_only   Downloads a code image without starting a debug
@REM                     session afterwards.
@REM   --silent          Omits the sign-on message.
@REM   --timeout         Limits the maximum allowed execution time.
@REM 


"D:\IAR_7.3\common\bin\cspybat" "D:\IAR_7.3\arm\bin\armproc.dll" "D:\IAR_7.3\arm\bin\armJET.dll"  %1 --plugin "D:\IAR_7.3\arm\bin\armbat.dll" --device_macro "D:\IAR_7.3\arm\config\debugger\NXP\LPC407x_LPC408x.dmac" --macro "D:\IAR_7.3\arm\config\flashloader\NXP\FlashNXPLPC540xx.mac" --flash_loader "D:\IAR_7.3\arm\config\flashloader\NXP\FlashNXPLPC512KNiobe.board" --backend -B "--endian=little" "--cpu=Cortex-M4F" "--fpu=VFPv4" "-p" "D:\IAR_7.3\arm\config\debugger\NXP\LPC540xx_M4.ddf" "--drv_verify_download" "--semihosting" "--device=LPC4088" "--multicore_nr_of_cores=1" "--jet_probe=cmsisdap" "--jet_script_file=D:\IAR_7.3\arm\config\debugger\NXP\LPC17xx.ProbeScript" "--jet_standard_reset=4,0,0" "--reset_style="0,-,0,Disabled__no_reset_"" "--reset_style="1,-,0,Software"" "--reset_style="2,-,0,Hardware"" "--reset_style="3,-,0,Core"" "--reset_style="4,-,1,System"" "--reset_style="7,ResetAndStopAtUser,0,Reset_and_halt_after_bootloader"" "--reset_style="8,ResetAndStopAtBoot,0,Reset_and_halt_before_bootloader"" "--jet_interface=SWD" "--jet_jtag_speed=12000" "--drv_catch_exceptions=0x7f0" 

