#!/bin/bash
# Run this script once after cloneing the repository.
# This script performs 3 functions. You will need to do them manually if it fails
#    a. Downloads PDDL-Parser (https://github.com/pucrs-automated-planning/pddl-parser) using the command  "git submodule update --init"
#    b. Adds a blank file called ```__init__.py``` to the pddl_parser directory (See https://docs.python.org/3/tutorial/modules.html#packages 
#       for explanatory details. TLDR; this file addition let's Python know that we want to consider it's encapsulating directory a package)
#    c. Changes baxter source code as instructed at https://github.com/Evana13G/RAPDR_babble/wiki/Changing-the-Baxter-Source-Code


# ensure that pddl_parser has been downloaded
git submodule update --init > /dev/null

# add an init file to the pddl module so python recognises it as a module
PDDL_DIR=util/src/util/pddl_parser
PYTHON_FILE=__init__.py
if [ -d "$PDDL_DIR"  ]; then
    touch $PDDL_DIR/$PYTHON_FILE
    if [ $? -ne 0 ] 
    then
        echo "Could not create the file $PDDL_DIR/$PYTHON_FILE"
        echo "Please ensure the direcory has the propper permissions."
        exit 1
    fi

else
    echo "Could not find the directory $PDDL_DIR."
    echo "Please ensure you have properly cloned the repository."
    exit 1
fi


# replace the source code in baxter to add a _rate=100 parameter 
# to the move_to_joint_position function
EDIT_FILE=../baxter_interface/src/baxter_interface/limb.py
INSERT_PARAM="_rate=100, "
if [ -f "$EDIT_FILE"  ]; then
    SEARCH_TERM=$(echo "$INSERT_PARAM" | cut -d'=' -f 1)
    grep -q $SEARCH_TERM $EDIT_FILE
    if [ $? -eq 1 ] #check if the rate parameter has already been added
    then
    sed -i "s/rate=100,/rate=$SEARCH_TERM,/g" $EDIT_FILE
    sed -i "s/def move_to_joint_positions(self, positions,/& $INSERT_PARAM/" $EDIT_FILE
        if [ $? -ne 0 ] 
        then
            echo "Could not edit the file $EDIT_FILE"
            echo "Please ensure sed is installed."
            echo "Please ensure that file permission are correct."
            exit 1
        fi
    else
        echo "You already ran this script. Please try manual edits if not working!"
    fi

else
    echo "Could not find the file $EDIT_FILE."
    echo "Please ensure that you are running this script from the RAPDR directory"
    echo "Please ensure that you have properly downloaded the Baxter files"
    exit 1
fi


