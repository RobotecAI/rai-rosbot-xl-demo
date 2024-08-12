#!/bin/bash

git submodule init
git submodule update

cd ./Project
git lfs install
git lfs pull

git apply ../patches/updates.patch
if [ $? -ne 0 ]
then
	echo "Failed to apply patch on a submodule."
	exit 1
fi

echo "Submodules updated."
exit 0
