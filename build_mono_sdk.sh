#!/usr/bin/env bash 

set -e 

O3DE_SDK=$1
if [ -z "$O3DE_SDK" ]
then
    echo "O3DE_SDK not set."
    exit 1
fi

cd Project || exit 1

AssetBundlerBatch="$O3DE_SDK/install/bin/Linux/profile/Default/AssetBundlerBatch"

cmake -B build/linux -S . -G "Ninja Multi-Config" -DLY_DISABLE_TEST_MODULES=ON -DLY_STRIP_DEBUG_SYMBOLS=ON -DAZ_USE_PHYSX5:BOOL=ON
cmake -B build/linux_mono -S . -G "Ninja Multi-Config" -DLY_DISABLE_TEST_MODULES=ON -DLY_STRIP_DEBUG_SYMBOLS=ON -DAZ_USE_PHYSX5:BOOL=ON -DLY_MONOLITHIC_GAME=1

cmake --build build/linux --config profile --target RAIROSBotXLDemo.Assets
cmake --build build/linux_mono --config profile --target RAIROSBotXLDemo.GameLauncher -j

${AssetBundlerBatch} assetLists --assetListFile `pwd`/AssetBundling/AssetLists/engine_linux.assetlist --platform linux --project-path `pwd` --allowOverwrites --addDefaultSeedListFiles;
${AssetBundlerBatch} bundles --maxSize 2048 --platform linux --project-path `pwd` --allowOverwrites --outputBundlePath `pwd`/AssetBundling/Bundles/engine_linux.pak --assetListFile `pwd`/AssetBundling/AssetLists/engine_linux.assetlist;

${AssetBundlerBatch} seeds --seedListFile `pwd`/AssetBundling/SeedLists/husarion.seed --addSeed `pwd`/Cache/linux/levels/loft/loft.spawnable --project-path `pwd`
${AssetBundlerBatch} assetLists --assetListFile `pwd`/AssetBundling/AssetLists/game_linux.assetlist --platform linux --project-path `pwd` --allowOverwrites --seedListFile `pwd`/AssetBundling/SeedLists/husarion.seed;
${AssetBundlerBatch} bundles --maxSize 2048 --platform linux --project-path `pwd` --allowOverwrites --outputBundlePath `pwd`/AssetBundling/Bundles/game_linux.pak --assetListFile `pwd`/AssetBundling/AssetLists/game_linux.assetlist;

mkdir -p release/Cache/linux
cp -r ./AssetBundling/Bundles/* release/Cache/linux
cp -r ./build/linux_mono/bin/profile/* release

echo "Done. Release can be found in 'release' folder."
