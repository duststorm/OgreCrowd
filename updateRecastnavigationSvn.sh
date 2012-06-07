#!/bin/bash

##
# Run this script after updating recastnavigation svn to integrate new
# version of recastnavigation with ogreRecast.
# 
# The directory tree is supposed to be as following:
# projectRoot/
#	recastnavigation/	(This folder contains an svn checkout of recastnavigation)
#		Detour
#		DetourCrowd
#		Recast
#		RecastDemo
#		...
#	ogreRecast/		(This project git/hg checkout. This script should be run from this path.)
#		src/
#		include/
#		...
#
# This script expects you did an svn update of recastnavigation before running.
# Note that newer versions of recastnavigation might introduce incompatibility
# with the current ogreRecast code.
##
pushd ../

# Copy neccessary source files over to ogreRecast project
cp recastnavigation/Recast/Source/*cpp ogreRecast/src/Recast/
cp recastnavigation/Detour/Source/*cpp ogreRecast/src/Detour/
cp recastnavigation/DetourCrowd/Source/*cpp ogreRecast/src/DetourCrowd/
cp recastnavigation/DetourTileCache/Source/*cpp ogreRecast/src/DetourTileCache/
cp recastnavigation/Recast/Include/*h ogreRecast/include/Recast/
cp recastnavigation/Detour/Include/*h ogreRecast/include/Detour
cp recastnavigation/DetourCrowd/Include/*h ogreRecast/include/DetourCrowd/
cp recastnavigation/DetourTileCache/Include/*h ogreRecast/include/DetourTileCache/

# Apply a small patch that fixes include paths
patch -p0 < ogreRecast/patchRecastnavigationSvn.patch

popd
