@echo off
REM Set your SSH server details
set SSH_HOST=10.43.0.1
set SSH_PORT=22
set SSH_USER=analog
set SSH_PASSWORD=analog
REM Set the command you want to execute remotely

REM Execute the SSH command using plink
REM plink -ssh -P %SSH_PORT% -l %SSH_USER% -pw %SSH_PASSWORD% %SSH_HOST% 
plink analog@10.43.0.1 -pw analog -m setup_server.sh

PAUSE