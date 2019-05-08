## Intalling cucumber and cucumber-cpp

### Dependencies

For building and executing behavior-driven test specifications, cucumber and cucumber-cpp need to be installed first. Please mind that Cucumber-Cpp is not compatible with Cucumber-Ruby 3.x due to a bug in its wire protocol implementation.

### Installation Process

1.  Clone this repository and navigate to the cloned directory
2.  Follow the instructions for your environment


 - **For Linux and Windows Ubuntu BASH** Please note that for any particular command, including execution of ```.sh``` scripts, it may be necessary to add ```sudo``` prior to the command.  It is also a good practice to run ```sudo apt-get update``` prior to installation of new libraries.

  * **Linux:**
    * ```sudo apt-get install curl```
    * Call `install-cucumber-ubuntu.sh`

  * **Windows:** For Windows environments follow Linux instructions in the Ubuntu Bash environment. Please note that install instructions should be executed from the repository directory.  Changing to a Windows directory (ie ```cd /mnt/c .....```) can result in installation issues, particularly for Windows directories that contain spaces.
  
### Troubleshooting

* If challenges to installation are encountered (install script fails).  Please consult the forums.  Please feel free to submit additional tips or forum threads to this repo, for potential inclusion in this document.
