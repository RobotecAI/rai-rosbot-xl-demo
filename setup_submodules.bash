#!/bin/bash

# Pull RobotVacuumSample repository and apply patches
git submodule init
git submodule update

cd ./Project
git lfs install
git lfs pull

git apply --reject --whitespace=fix ../patches/updates.patch
if [ $? -ne 0 ]
then
	echo "Failed to apply patch on a submodule."
	exit 1
fi

mkdir -p ./AssetBundling/SeedLists
cp ../patches/husarion.seed ./AssetBundling/SeedLists

mkdir -p Gem/Source/LEDStrip
cp ../patches/LEDStrip.* Gem/Source/LEDStrip

cp ../patches/sim_config.setreg Registry

# Remove unused (ambiguous) files from the submodule
rm -rf ./Assets/Importer/
rm -rf ./Assets/RoboVac/
rm -rf ./Docker
rm ./README.md
rm ./build.sh

echo "Submodules updated."
exit 0
