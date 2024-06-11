# conftest.py
import pytest
import time

#####################USER-Configured#########################

@pytest.fixture(autouse=True)
def sdk_version():
    return "5.0.0"

@pytest.fixture(autouse=True)
def ip_set():
    return "10.43.0.1"
    
@pytest.fixture(autouse=True)
def config_file():
    return "config/config_adsd3500_adsd3100.json"   
  
@pytest.fixture(autouse=True)
def sdcard_version():
    return "microsd-5.0.0-4ab17db3.img" 
    #return "microsd-4.3.0-08d887e8.img"

#####################Test-Configured#########################   
@pytest.fixture(params=[0,1,2,3,4,5,6])
def available_modes_ini(request):
    return request.param

@pytest.fixture(autouse=True)
def delay_between_tests():
    time.sleep(3) # sleep for 3 second
    
