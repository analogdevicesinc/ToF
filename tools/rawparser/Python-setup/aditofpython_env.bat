@echo off
echo "Creating aditofpython_env."
python -m venv aditofpython_env
call .\aditofpython_env\Scripts\activate.bat
timeout /T 5
echo "aditofpython_env created and activated"
echo "Installing all the required modules.. This may take a while.."
python -m pip install --upgrade pip
python -m pip install -r .\requirements.txt
echo "required modules for aditofpython_env installed sucessfully... "
@echo on