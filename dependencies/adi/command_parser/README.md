# Command Parser

## Overview
This library provides a robust and easily customizable method of configuring, reading and checking arguments used in the command line for any application. 

## How to create a configuration
In order to make a configuration simply create a **map<string, struct>** and populate it with the desired argumets for your specific application. The key of the map is the short version of an argument (denoted by "-").

The struct has the following possible options: 
* **string long_option** (long version of an argument, denoted by "--").
* **bool is_mandatory** (determines if the argument needs to be used in every run).
* **string position** (determines if the argument needs to be in a specific location).
* **string value** (is used to hold a default value or to hold a value used in command line).
This struct can be easily expanded in order to fit the need of an application.

## Methods of the library
**parseArguments**: 
* Read the arguments provided alongside with the value in a **vector<pair<string, string>>**.

**checkArgumentsExist**:
* Search if any arguemnts used are present in the configuration map.
* If arguments doesn't exist, will return an error.

**helpMenu**: 
* Check if help has been called.
* Returns error is help is missused.

**checkValue**: 
* Send the values provided from the configuration to the map.
* Checks if default or value were provided.

**checkMandatoryArguments**: 
* Check if all mandatory arguments have been provided.

**checkMandatoryPosition**:
* Check if all mandatory arguments are placed in the correct location.