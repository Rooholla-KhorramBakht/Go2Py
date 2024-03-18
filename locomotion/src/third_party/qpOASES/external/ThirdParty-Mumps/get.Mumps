#!/bin/sh

# Set the following to the latest MUMPS version.
#  THERE MUST BE NO SPACE BEFORE AND AFTER THE EQUAL (=) OPERATOR.
mumps_ver=5.5.0

set -e

wgetcount=`which wget 2>/dev/null | wc -l`
if test ! $wgetcount = 1; then
  curlcount=`which curl 2>/dev/null | wc -l`
  if test ! $curlcount = 1; then
    fetchcount=`which fetch 2>/dev/null | wc -l`
    if test ! $fetchcount = 1; then
      echo "None of the utilities, wget, curl, or fetch found in PATH. Cannot download source."
      exit -1
    else
      wgetcmd=fetch
    fi
  else
    wgetcmd="curl -L -O"
  fi
else
wgetcmd="wget"
fi

echo " "
echo "Running script for downloading the source code for MUMPS"
echo " "

rm -f MUMPS*.tgz

echo "Downloading the source code from coin-or-tools.github.io..."
if $wgetcmd http://coin-or-tools.github.io/ThirdParty-Mumps/MUMPS_${mumps_ver}.tar.gz ;
then
  echo "Download finished."
else
  echo
  echo "Downloading from GitHub failed, trying mumps.enseeiht.fr..."
  if $wgetcmd http://mumps.enseeiht.fr/MUMPS_${mumps_ver}.tar.gz ;
  then
    echo "Download finished."
  else
    echo "Download failed...exiting"
  fi
fi

echo "Uncompressing the tarball..."
gunzip -f MUMPS_${mumps_ver}.tar.gz

echo "Unpacking the source code..."
tar xf MUMPS_${mumps_ver}.tar

echo "Deleting the tar file..."
rm MUMPS_${mumps_ver}.tar

rm -rf MUMPS
mv MUMPS_${mumps_ver} MUMPS

echo " "
echo "Done downloading the source code for MUMPS."
echo " "
echo "Apply a patch to improve MPI compatibility."
echo " "

patch -p0 < mumps_mpi.patch
mv MUMPS/libseq/mpi.h MUMPS/libseq/mumps_mpi.h

echo " "
echo "Verify that there are no error message in the output above."
