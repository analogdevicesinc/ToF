@echo off
if exist aditofpython_env (
	echo "Activating ADI ToF python env"
	call .\aditofpython_env\Scripts\activate.bat
) else (
	echo "aditofpython environment is not yet created.."
	call .\aditofpython_env.bat
)
@echo on